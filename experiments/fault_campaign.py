# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Fault injection and recovery on bridges, cantilevers, and the shelter.

Every probability in this file is configuration. It was not measured on
a robot and it was not tuned after looking at the makespan. ``k = 1``.
Sag is ``mass·g/k``. Joint demand is ``mass·g``. Neither is a fit.

    python main.py faults
    python main.py faults smoke

The crane walks the floor (reach 4, climb 0). Two robots. The shelter
schedule is the 256-cell stand-in with two feed stations and two robots
of each role, so a feed outage has somewhere to go. Gates stay on:
cantilever, sag, and joint utilization at the finished demand.
"""

from __future__ import annotations

import csv
import platform
import random
import shutil
import socket
import threading
from http.server import ThreadingHTTPServer, SimpleHTTPRequestHandler
from pathlib import Path

from experiments.joint_limits import peak_forces
from experiments.stability_cost import build_bridge, build_cantilever, start_cells
from sim.armadas import (
    G,
    K,
    MASS,
    ActionTimes,
    feed_station_cells,
    shelter_model,
    simulate,
)
from sim.faults import FaultInjector, FaultRates
from sim.geometry3d import CubicLattice3D
from sim.metrics import RunLog
from sim.precision import armadas_precision
from sim.scaffold import CRANE, caps_for_finished, geometry_factory
from sim.stability import JointCap, JointStrength
from sim.swarm import Swarm, SwarmStuck
from sim.world import DEFECT

DOCS = Path("docs")
RATES = (0.0, 0.05, 0.10, 0.20)
SEEDS = (0, 1, 2)
STRUCTURES = ("bridge", "cantilever", "shelter")
RATE_SOURCE = "configuration"
MAX_TICKS = 2500
N_ROBOTS = 2
FACTORY = geometry_factory(CRANE)

# Durations are configuration, same quantum idea as the shelter's 20 s
# step: a few steps, not a measured repair time.
STALL_TICKS = 3
DOWN_TICKS = 6
OUTAGE_TICKS = 4
OUTAGE_S = 40.0

TRIAL_FIELDS = (
    "structure",
    "seed",
    "fault_rate",
    "rate_source",
    "k",
    "mass",
    "g",
    "completed",
    "makespan",
    "makespan_unit",
    "baseline_makespan",
    "overhead",
    "n_faults",
    "n_cascades",
    "n_scaffold",
    "n_capture",
    "n_drop",
    "n_stall",
    "n_robot_fail",
    "n_fastener",
    "n_feed_outage",
    "recovery_n",
    "recovery_min",
    "recovery_p50",
    "recovery_p90",
    "recovery_max",
    "sag_cap_mg_over_k",
    "joint_compression_mg",
    "joint_shear_mg",
    "n_voxels",
    "n_robots",
    "tick_cap",
)

EVENT_FIELDS = (
    "structure",
    "seed",
    "fault_rate",
    "rate_source",
    "kind",
    "channel_rate",
    "recovery",
    "cascade",
    "recovery_time",
    "time_unit",
    "tick",
    "time_s",
    "cell",
    "joint",
    "utilization",
    "k",
)


def rates_for(rate: float) -> FaultRates:
    """One configured probability on every channel. Not a measurement."""
    return FaultRates(
        capture=rate,
        drop=rate,
        stall=rate,
        robot_fail=rate,
        fastener=rate,
        feed_outage=rate,
        stall_ticks=STALL_TICKS,
        down_ticks=DOWN_TICKS,
        outage_ticks=OUTAGE_TICKS,
        outage_s=OUTAGE_S,
    )


def _joint_cap(forces: dict) -> JointCap:
    """Finished demand. A component near zero is left unchecked."""
    compression = forces["compression"] if forces["compression"] > 1e-8 else None
    shear = forces["shear"] if forces["shear"] > 1e-8 else None
    return JointCap(
        JointStrength(axial_compression=compression, shear=shear),
        safety_factor=1.0,
        k=1.0,
        mass=1.0,
        g=1.0,
    )


def _percentile(values: list[float], fraction: float) -> float:
    if not values:
        raise ValueError("empty")
    ordered = sorted(values)
    if len(ordered) == 1:
        return float(ordered[0])
    index = (len(ordered) - 1) * fraction
    lo = int(index)
    hi = min(lo + 1, len(ordered) - 1)
    weight = index - lo
    return float(ordered[lo] * (1.0 - weight) + ordered[hi] * weight)


def _fmt(value) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, float):
        return f"{value:.6g}"
    return str(value)


def _count(events, kind: str) -> int:
    return sum(1 for event in events if event.kind == kind)


def _world_for(structure: str):
    if structure == "bridge":
        return build_bridge(5)
    if structure == "cantilever":
        return build_cantilever(3)
    raise KeyError(structure)


def _caps(world):
    frame = CubicLattice3D(world).stability_policy(0)
    policy, sag = caps_for_finished(world, frame)
    cells = [
        (int(level), int(row), int(col))
        for level, row, col in zip(*world.blueprint.nonzero())
    ]
    # peak_forces reads the occupied set. The finished blueprint is the
    # demand the cap is copied from. k = mass = g = 1.
    filled = world.copy()
    for cell in cells:
        if filled.is_empty(cell):
            filled.place_voxel(cell)
    forces = peak_forces(
        [
            (int(level), int(row), int(col))
            for level, row, col in zip(*filled.occupancy.nonzero())
            if int(filled.occupancy[level, row, col]) == 2
        ],
        policy,
    )
    return policy, sag, _joint_cap(forces), forces, len(cells)


def run_swarm_trial(structure: str, rate: float, seed: int) -> dict:
    world, depot = _world_for(structure)
    policy, sag, joints, forces, n_voxels = _caps(world)
    faults = FaultInjector(rates_for(rate), seed)
    precision = armadas_precision(distribution="zero")
    starts = start_cells(world, depot, N_ROBOTS, seed)
    swarm = Swarm(
        world,
        depot,
        starts=starts,
        geometry_factory=FACTORY,
        stability=policy,
        sag=sag,
        joints=joints,
        faults=faults,
        precision=precision,
        rng=random.Random(seed),
        inspect_enabled=True,
    )
    stuck = ""
    try:
        while not swarm.done and swarm.tick_count < MAX_TICKS:
            swarm.tick()
    except SwarmStuck as exc:
        stuck = str(exc)
    completed = bool(swarm.done and world.complete and not stuck)
    makespan = swarm.tick_count if completed else None
    return _row(
        structure=structure,
        rate=rate,
        seed=seed,
        completed=completed,
        makespan=makespan,
        unit="ticks",
        faults=faults,
        sag_cap=sag.max_sag,
        forces=forces,
        n_voxels=n_voxels,
        n_robots=N_ROBOTS,
        capped=swarm.tick_count >= MAX_TICKS and not completed,
        stuck=stuck,
    )


def run_shelter_trial(rate: float, seed: int) -> dict:
    model = shelter_model()
    faults = FaultInjector(rates_for(rate), seed)
    precision = armadas_precision(distribution="zero")
    forces = peak_forces(model.cells, model.policy)
    joints = _joint_cap(forces)
    try:
        result = simulate(
            model,
            n_feeds=2,
            robots_per_role=2,
            times=ActionTimes(),
            stations=feed_station_cells(2),
            faults=faults,
            precision=precision,
            joint_cap=joints,
        )
        completed = result.n_placed == len(model.order)
        makespan = result.makespan_s
        stuck = ""
    except RuntimeError as exc:
        completed = False
        makespan = None
        stuck = str(exc)
    return _row(
        structure="shelter",
        rate=rate,
        seed=seed,
        completed=completed,
        makespan=makespan,
        unit="seconds",
        faults=faults,
        sag_cap=model.finished_sag,
        forces=forces,
        n_voxels=len(model.cells),
        n_robots=6,
        capped=False,
        stuck=stuck,
    )


def _row(
    *,
    structure,
    rate,
    seed,
    completed,
    makespan,
    unit,
    faults: FaultInjector,
    sag_cap,
    forces,
    n_voxels,
    n_robots,
    capped,
    stuck,
) -> dict:
    times = faults.recovery_times()
    events = []
    for event in faults.events:
        events.append(
            {
                "structure": structure,
                "seed": seed,
                "fault_rate": rate,
                "rate_source": RATE_SOURCE,
                "kind": event.kind,
                "channel_rate": event.rate,
                "recovery": event.recovery,
                "cascade": int(event.cascade),
                "recovery_time": "" if event.recovery_time is None else event.recovery_time,
                "time_unit": unit,
                "tick": "" if event.tick is None else event.tick,
                "time_s": "" if event.time_s is None else event.time_s,
                "cell": "" if event.cell is None else ",".join(str(v) for v in event.cell),
                "joint": ""
                if event.joint is None
                else "|".join(",".join(str(v) for v in side) for side in event.joint),
                "utilization": "" if event.utilization is None else event.utilization,
                "k": K,
            }
        )
    return {
        "structure": structure,
        "seed": seed,
        "fault_rate": rate,
        "rate_source": RATE_SOURCE,
        "k": K,
        "mass": MASS,
        "g": G,
        "completed": int(completed),
        "makespan": "" if makespan is None else makespan,
        "makespan_unit": unit,
        "baseline_makespan": "",
        "overhead": "",
        "n_faults": len(faults.events),
        "n_cascades": faults.n_cascades,
        "n_scaffold": _count(faults.events, "scaffold"),
        "n_capture": _count(faults.events, "capture"),
        "n_drop": _count(faults.events, "drop"),
        "n_stall": _count(faults.events, "stall"),
        "n_robot_fail": _count(faults.events, "robot_fail"),
        "n_fastener": _count(faults.events, "fastener"),
        "n_feed_outage": _count(faults.events, "feed_outage"),
        "recovery_n": len(times),
        "recovery_min": "" if not times else min(times),
        "recovery_p50": "" if not times else _percentile(times, 0.50),
        "recovery_p90": "" if not times else _percentile(times, 0.90),
        "recovery_max": "" if not times else max(times),
        "sag_cap_mg_over_k": sag_cap,
        "joint_compression_mg": forces["compression"],
        "joint_shear_mg": forces["shear"],
        "n_voxels": n_voxels,
        "n_robots": n_robots,
        "tick_cap": int(capped),
        "stuck": stuck,
        "events": events,
    }


def _apply_baseline(rows: list[dict]) -> None:
    """Overhead against the rate-0 median of the same structure.

    Rate 0 is the unfaulted run of this same configuration. The fault
    rate itself stays an input.
    """
    baselines: dict[str, list[float]] = {}
    for row in rows:
        if row["fault_rate"] == 0.0 and row["completed"] and row["makespan"] != "":
            baselines.setdefault(row["structure"], []).append(float(row["makespan"]))
    median = {
        name: _percentile(values, 0.5) for name, values in baselines.items()
    }
    for row in rows:
        base = median.get(row["structure"])
        row["baseline_makespan"] = "" if base is None else base
        if base and row["completed"] and row["makespan"] != "":
            row["overhead"] = float(row["makespan"]) / base - 1.0
        elif row["fault_rate"] == 0.0 and row["completed"]:
            row["overhead"] = 0.0


def iter_jobs(smoke: bool = False):
    if smoke:
        yield ("cantilever", 0.05, 0)
        return
    for structure in STRUCTURES:
        for rate in RATES:
            for seed in SEEDS:
                yield (structure, rate, seed)


def run_rows(smoke: bool = False) -> list[dict]:
    rows = []
    jobs = list(iter_jobs(smoke))
    for index, (structure, rate, seed) in enumerate(jobs, start=1):
        if structure == "shelter":
            row = run_shelter_trial(rate, seed)
        else:
            row = run_swarm_trial(structure, rate, seed)
        rows.append(row)
        print(
            f"[{index}/{len(jobs)}] {structure} rate={rate} seed={seed} "
            f"done={row['completed']} makespan={row['makespan']} "
            f"faults={row['n_faults']} cascades={row['n_cascades']}",
            flush=True,
        )
    _apply_baseline(rows)
    return rows


def _write_dict_csv(path: Path, fields: tuple[str, ...], rows: list[dict]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _fmt(row.get(key, "")) for key in fields})


def write_csv(rows: list[dict], out_dir: Path) -> tuple[Path, Path]:
    trials = out_dir / "fault_campaign.csv"
    events = out_dir / "fault_events.csv"
    _write_dict_csv(trials, TRIAL_FIELDS, rows)
    flat = [event for row in rows for event in row["events"]]
    _write_dict_csv(events, EVENT_FIELDS, flat)
    return trials, events


def _num(row, key) -> float | None:
    value = row.get(key, "")
    if value == "" or value is None:
        return None
    return float(value)


def _median(values: list[float]) -> float:
    return _percentile(values, 0.5)


def _spread(values: list[float]) -> str:
    if not values:
        return "n=0"
    return (
        f"median {_median(values):.4g} "
        f"(min {_min(values):.4g}, max {_max(values):.4g}, n={len(values)})"
    )


def _min(values: list[float]) -> float:
    return min(values)


def _max(values: list[float]) -> float:
    return max(values)


def summarize(rows: list[dict]) -> str:
    lines = [
        "Fault injection and recovery. Fault rates are configuration, not a measurement.",
        f"k = {K}, mass = {MASS}, g = {G}. Sag is mass·g/k. Joint demand is mass·g. Stiffness is not fit.",
        f"Channels share one configured probability in {{{', '.join(str(r) for r in RATES)}}}.",
        f"Seeds {list(SEEDS)}. Stall {STALL_TICKS} ticks, robot down {DOWN_TICKS} ticks, "
        f"depot outage {OUTAGE_TICKS} ticks, shelter outage {OUTAGE_S} s. Those durations are configuration.",
        f"Python {platform.python_version()}.",
        "",
    ]
    for structure in STRUCTURES:
        group = [row for row in rows if row["structure"] == structure]
        if not group:
            continue
        unit = group[0]["makespan_unit"]
        lines.append(f"## {structure} ({unit})")
        for rate in sorted({float(row["fault_rate"]) for row in group}):
            cell = [row for row in group if float(row["fault_rate"]) == rate]
            done = [row for row in cell if row["completed"]]
            overhead = [_num(row, "overhead") for row in done]
            overhead = [value for value in overhead if value is not None]
            cascades = [float(row["n_cascades"]) for row in cell]
            faults = [float(row["n_faults"]) for row in cell]
            spans = [_num(row, "makespan") for row in done]
            spans = [value for value in spans if value is not None]
            recovery = []
            for row in cell:
                for event in row["events"]:
                    if event["recovery_time"] != "":
                        recovery.append(float(event["recovery_time"]))
            scaffold = sum(int(row["n_scaffold"]) for row in cell)
            lines.append(
                f"  rate {rate:.2f} (configuration): "
                f"completed {len(done)}/{len(cell)}, "
                f"makespan {_spread(spans)}, "
                f"overhead {_spread(overhead)}, "
                f"faults {_spread(faults)}, "
                f"cascades {_spread(cascades)}, "
                f"recovery {unit} {_spread(recovery)}, "
                f"scaffold insertions {scaffold}"
            )
        lines.append("")
    lines.append(
        "Recovery: capture and fastener re-place through inspect/remove; "
        "drop and stall re-route; robot failure and feed outage reassign; "
        "scaffold is inserted only when the plain order is gone."
    )
    return "\n".join(lines) + "\n"


def _style(ax) -> None:
    ax.spines["top"].set_visible(False)
    ax.spines["right"].set_visible(False)
    ax.grid(True, axis="y", color="#e2e8f0")
    ax.tick_params(labelsize=8)


def write_charts(rows: list[dict], out_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    colors = {"bridge": "#1d4ed8", "cantilever": "#b45309", "shelter": "#0f766e"}
    paths = []

    def series(structure: str, rate: float, key: str, completed_only: bool = False):
        chosen = [
            row
            for row in rows
            if row["structure"] == structure and float(row["fault_rate"]) == rate
        ]
        if completed_only:
            chosen = [row for row in chosen if row["completed"]]
        values = []
        for row in chosen:
            number = _num(row, key)
            if number is not None:
                values.append(number)
        return values

    rates = sorted({float(row["fault_rate"]) for row in rows})
    structures = [name for name in STRUCTURES if any(row["structure"] == name for row in rows)]

    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for name in structures:
        ys = []
        for rate in rates:
            cell = [
                row
                for row in rows
                if row["structure"] == name and float(row["fault_rate"]) == rate
            ]
            ys.append(sum(int(row["completed"]) for row in cell) / len(cell))
        ax.plot(rates, ys, marker="o", color=colors[name], label=name)
    ax.set_ylim(-0.05, 1.05)
    ax.set_xlabel("fault rate (configuration, not measured)")
    ax.set_ylabel("completion rate")
    ax.set_title("Completion vs configured fault rate. k = 1, not fitted.")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)
    path = out_dir / "fault_completion.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    paths.append(path)

    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for name in structures:
        xs, ys = [], []
        for rate in rates:
            values = series(name, rate, "overhead", completed_only=True)
            if values:
                xs.append(rate)
                ys.append(_median(values))
        ax.plot(xs, ys, marker="o", color=colors[name], label=name)
    ax.axhline(0.0, color="#94a3b8", linewidth=0.8)
    ax.set_xlabel("fault rate (configuration, not measured)")
    ax.set_ylabel("makespan overhead (median)")
    ax.set_title("Makespan / rate-0 median − 1. Completed trials. k = 1.")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)
    path = out_dir / "fault_overhead.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    paths.append(path)

    fig, ax = plt.subplots(figsize=(6.4, 3.6))
    for name in structures:
        xs, ys = [], []
        for rate in rates:
            values = series(name, rate, "n_cascades")
            if values:
                xs.append(rate)
                ys.append(_median(values))
        ax.plot(xs, ys, marker="o", color=colors[name], label=name)
    ax.set_xlabel("fault rate (configuration, not measured)")
    ax.set_ylabel("cascades per trial (median)")
    ax.set_title("Cascades while a recovery is still open. k = 1.")
    ax.legend(frameon=False, fontsize=8)
    _style(ax)
    path = out_dir / "fault_cascades.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    paths.append(path)

    fig, axes = plt.subplots(1, len(structures), figsize=(3.2 * len(structures), 3.6), squeeze=False)
    for ax, name in zip(axes[0], structures):
        data = []
        labels = []
        for rate in rates:
            times = []
            for row in rows:
                if row["structure"] != name or float(row["fault_rate"]) != rate:
                    continue
                for event in row["events"]:
                    if event["recovery_time"] != "":
                        times.append(float(event["recovery_time"]))
            if times:
                data.append(times)
                labels.append(f"{rate:g}")
        if data:
            ax.boxplot(data, tick_labels=labels, showfliers=False)
        unit = next(row["makespan_unit"] for row in rows if row["structure"] == name)
        ax.set_title(name, fontsize=9)
        ax.set_xlabel("fault rate (config)")
        ax.set_ylabel(f"recovery time ({unit})")
        _style(ax)
    fig.suptitle("Recovery time. Rates are configuration. k = 1, not fitted.", fontsize=10)
    path = out_dir / "fault_recovery.png"
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)
    paths.append(path)
    return paths


def build_recovery_log(path: Path) -> dict:
    """Span-5 bridge, one scripted capture, stopped on the inspect tick.

    The capture is armed after four good voxels, so the picture is a
    bridge underway and not the first joint. The log is the viewer's
    input. Deformation stays off. The red cell is the defect the
    inspect/remove path is clearing. Boxes are the viewer's 0.96 inset,
    so face neighbors keep a 0.04 gap and do not overlap.
    """
    from sim.world import VOXEL

    world, depot = build_bridge(5)
    policy, sag, joints, _forces, _n = _caps(world)
    faults = FaultInjector(FaultRates(), seed=4)
    starts = start_cells(world, depot, 2, 4)
    swarm = Swarm(
        world,
        depot,
        starts=starts,
        geometry_factory=FACTORY,
        stability=policy,
        sag=sag,
        joints=joints,
        faults=faults,
        precision=armadas_precision(distribution="zero"),
        rng=random.Random(4),
    )
    log = RunLog()
    log.meta["fault"] = "capture"
    log.meta["rate_source"] = RATE_SOURCE
    log.log_tick(0, world, robots=swarm.robots)
    frame = None
    armed = False
    for _ in range(MAX_TICKS):
        if swarm.done:
            break
        if not armed and int((world.occupancy == VOXEL).sum()) >= 4:
            faults.script.append("capture")
            armed = True
        swarm.tick()
        log.log_tick(swarm.tick_count, world, robots=swarm.robots)
        defective = [
            tuple(int(v) for v in cell)
            for cell in zip(*((world.occupancy == DEFECT).nonzero()))
        ]
        inspecting = any(robot.state == "INSPECT" for robot in swarm.robots)
        if armed and defective and inspecting:
            frame = len(log.frames) - 1
            break
    if frame is None:
        raise RuntimeError("recovery frame was not reached")
    path.parent.mkdir(parents=True, exist_ok=True)
    log.save(path)
    return {
        "log": str(path),
        "frame": frame,
        "ticks": swarm.tick_count,
        "faults": [(event.kind, event.cell, event.recovery) for event in faults.events],
    }


def _free_port() -> int:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return int(sock.getsockname()[1])


def render_still(root: Path, info: dict, destinations: list[Path]) -> Path:
    """Headless viewer frame. Boxes are the viewer's 0.96 inset."""
    from playwright.sync_api import sync_playwright

    port = _free_port()
    handler = lambda *args, **kwargs: SimpleHTTPRequestHandler(  # noqa: E731
        *args, directory=str(root), **kwargs
    )
    httpd = ThreadingHTTPServer(("127.0.0.1", port), handler)
    thread = threading.Thread(target=httpd.serve_forever, daemon=True)
    thread.start()
    log_rel = Path(info["log"]).resolve().relative_to(root.resolve()).as_posix()
    url = (
        f"http://127.0.0.1:{port}/replay_viewer.html?log={log_rel}"
        f"&frame={info['frame']}&color=material&deform=0"
        f"&theta=1.45&phi=1.25&radius=8"
    )
    try:
        with sync_playwright() as playwright:
            browser = playwright.chromium.launch(
                executable_path=shutil.which("google-chrome")
                or shutil.which("google-chrome-stable")
                or shutil.which("chromium")
                or shutil.which("chromium-browser"),
                args=["--no-sandbox", "--disable-dev-shm-usage"],
            )
            page = browser.new_page(viewport={"width": 1280, "height": 720})
            page.goto(url, wait_until="domcontentloaded")
            page.wait_for_function(
                "() => window.assemblerReplay && window.assemblerReplay.ready() "
                "&& window.assemblerReplay.nFrames() > 0",
                timeout=45_000,
            )
            page.evaluate(
                """async ({frame}) => {
                    const v = window.assemblerReplay;
                    v.setPlaying(false);
                    v.setColor('material');
                    v.setDeform(0);
                    v.setFrame(frame);
                    await new Promise(r => requestAnimationFrame(() => requestAnimationFrame(r)));
                }""",
                {"frame": info["frame"]},
            )
            png = page.screenshot()
            browser.close()
    finally:
        httpd.shutdown()
    for directory in destinations:
        directory.mkdir(parents=True, exist_ok=True)
        (directory / "fault_recovery_still.png").write_bytes(png)
    return destinations[0] / "fault_recovery_still.png"


def write_outputs(rows: list[dict], out_dir: Path, *, still: bool) -> dict:
    trials, events = write_csv(rows, out_dir)
    text = summarize(rows)
    summary = out_dir / "fault_summary.txt"
    summary.write_text(text)
    charts = write_charts(rows, out_dir)
    still_path = None
    if still:
        root = Path(__file__).resolve().parents[1]
        info = build_recovery_log(root / "runs" / "fault_recovery.json")
        still_path = render_still(
            root,
            info,
            [out_dir, Path("/opt/cursor/artifacts")],
        )
        print(f"still frame {info['frame']} faults {info['faults']}", flush=True)
    print(text, flush=True)
    return {
        "trials": trials,
        "events": events,
        "summary": summary,
        "charts": charts,
        "still": still_path,
        "rows": rows,
    }


def run_campaign(out_dir: Path = DOCS) -> dict:
    return write_outputs(run_rows(False), out_dir, still=True)


def run_smoke(out_dir: Path) -> dict:
    return write_outputs(run_rows(True), out_dir, still=False)


def main(argv: list[str] | None = None) -> None:
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    smoke = "smoke" in args
    if smoke:
        run_smoke(DOCS / "fault_smoke")
    else:
        run_campaign(DOCS)


if __name__ == "__main__":
    main()
