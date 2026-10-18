# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Cost and effect of the structural stability gate (v0.2 milestone 3).

The gate is off unless a caller passes a :class:`~sim.stability.StabilityPolicy`.
This campaign turns it on. For each blueprint, start seed, and robot count
it compares:

* gate off (``stability=None``)
* gate on at ``max_cantilever`` 0, 1, 2, 3, and ``None`` (connected to the
  base, no span limit)

and records plan feasibility, planning wall time, build makespan, idle
robot-ticks, kickbacks, and peak :func:`~sim.stability.estimate_sag`
during construction.

Stiffness is the model default ``k = 1`` (and ``mass = g = 1``). Sag is
in units of ``mass·g/k``. It has not been fit to a printed specimen.

Feasibility is one ``plan_build_order`` search capped at
:data:`MAX_NODES`. A search that hits the cap is **undecided**, not
infeasible. Named shapes are small enough that a real rejection finishes
inside the cap; the 17-voxel hollow box at ``max_cantilever = 0`` does
not, and the table says so.

Seeds choose robot start cells only (named blueprints are fixed). Fuzz
seeds choose the blueprint as well. Starts are drawn with
``random.Random.randrange``, which is stable across the Python versions
this repo tests. ``random.sample`` is not.

Run the campaign (writes ``docs/stability_cost.csv`` and the charts):

    python main.py stability

Smoke run used by the tests (seconds, not the published numbers):

    python main.py stability smoke
"""

from __future__ import annotations

import csv
import platform
import random
import time
from collections import defaultdict
from pathlib import Path
from typing import Callable, Iterable

from sim.choreographer import _BUILDABILITY_CACHE
from sim.geometry3d import CubicLattice3D
from sim.metrics import RunLog, StructureLog
from sim.planner import SearchBudgetExceeded, plan_build_order
from sim.stability import estimate_sag, is_stable, occupied_voxels
from sim.swarm import Swarm, SwarmStuck
from sim.world3d import World3D

# Search budget for the feasibility call. Successful orders on these
# blueprints return on the first heuristic path (tens of nodes). Proved
# rejections of the bridges and cantilevers also finish well under this.
# The hollow box at max_cantilever 0 does not: those rows stay undecided.
MAX_NODES = 200
MAX_TICKS = 4_000

# Uncalibrated spring. Do not treat a sag number as meters.
K = 1.0
MASS = 1.0
G = 1.0

START_SEEDS = (0, 1, 2)
ROBOT_COUNTS = (1, 2)
FUZZ_SEEDS = (1001, 1002, 1003, 1004)
FUZZ_START_SEEDS = (0, 1)

# condition id, label stored in the CSV, policy argument.
# GATE_OFF is not a cantilever value: the planner is not given a policy.
GATE_OFF = "off"
CONDITIONS: tuple[tuple[str, int | None], ...] = (
    (GATE_OFF, None),
    ("0", 0),
    ("1", 1),
    ("2", 2),
    ("3", 3),
    ("none", None),
)

Condition = tuple[str, int | None]
Builder = Callable[[], tuple[World3D, tuple[int, int, int]]]

CSV_FIELDS = (
    "blueprint",
    "family",
    "detail",
    "blueprint_seed",
    "start_seed",
    "n_robots",
    "n_voxels",
    "depot",
    "gate",
    "max_cantilever",
    "status",
    "finished_stable",
    "plan_s",
    "completed",
    "makespan",
    "idle_robot_ticks",
    "kickbacks",
    "peak_sag",
    "final_sag",
    "sag_errors",
    "sag_samples",
    "max_nodes",
    "k",
    "mass",
    "g",
    "python",
)


def _depot_and_world(levels: int, rows: int, cols: int) -> tuple[World3D, tuple[int, int, int]]:
    return World3D(levels, rows, cols), (1, 1, 1)


def build_wall() -> tuple[World3D, tuple[int, int, int]]:
    """4-wide, 1-deep, 3-tall wall. Every voxel can sit on a column."""
    world, depot = _depot_and_world(5, 6, 10)
    world.blueprint[1:4, 2:3, 3:7] = True
    return world, depot


def build_hollow_box() -> tuple[World3D, tuple[int, int, int]]:
    """Smallest hollow box the stock helper will cut: 3×3×2, roof on.

    The interior roof cell is one horizontal step from a wall. Limit 0
    cannot accept the finished roof. The order search does not prove
    that inside :data:`MAX_NODES`.
    """
    world, depot = _depot_and_world(4, 7, 9)
    world.set_box_blueprint(width=3, depth=3, height=2, left=3, front=2, hollow=True)
    return world, depot


def build_bridge(span: int = 5) -> tuple[World3D, tuple[int, int, int]]:
    """One-voxel-wide deck at level 2 between two piers ``span`` columns apart.

    Level 2 keeps the deck off the ground plane (a level-1 deck would be
    planted and would not sag). ``span`` is the column gap between piers,
    not the free-voxel count.
    """
    world, depot = _depot_and_world(4, 7, span + 8)
    row = 2
    c0, c1 = 3, 3 + span
    world.blueprint[1, row, c0] = True
    world.blueprint[1, row, c1] = True
    world.blueprint[2, row, c0 : c1 + 1] = True
    return world, depot


def build_cantilever(length: int) -> tuple[World3D, tuple[int, int, int]]:
    """Planted root, one column voxel, then ``length`` voxels straight out.

    The tip's overhang is ``length``. The arm is at level 2 so it is not
    fastened to the floor.
    """
    world, depot = _depot_and_world(4, 6, length + 8)
    row, col = 2, 3
    world.blueprint[1, row, col] = True
    world.blueprint[2, row, col : col + 1 + length] = True
    return world, depot


def build_fuzz(seed: int) -> tuple[World3D, tuple[int, int, int], str, str]:
    """One small random shape. Returns world, depot, blueprint name, detail.

    The four campaign seeds are one wall, one bridge, one cantilever, and
    one ledge; the seed draws the dimensions. Kinds stay inside the size
    where a rejection proof fits in :data:`MAX_NODES`. The hollow box is
    a named shape, not a fuzz draw: its limit-0 search is the slow
    undecided case, and repeating it here would not add a new fact.
    """
    rng = random.Random(seed)
    # The published seeds are one of each kind. A seed outside that list
    # still draws a kind, so the helper stays usable on its own.
    if seed in FUZZ_SEEDS:
        kind = ("wall", "bridge", "cantilever", "ledge")[list(FUZZ_SEEDS).index(seed)]
    else:
        kind = ("wall", "bridge", "cantilever", "ledge")[rng.randrange(4)]
    if kind == "wall":
        width = 2 + rng.randrange(3)  # 2..4
        height = 2 + rng.randrange(2)  # 2..3
        world, depot = build_wall()
        world.blueprint[:] = False
        world.blueprint[1 : 1 + height, 2:3, 3 : 3 + width] = True
        return world, depot, f"fuzz_{seed}", f"wall {width}x1x{height}"
    if kind == "bridge":
        span = 4 + rng.randrange(3)  # 4..6
        world, depot = build_bridge(span)
        return world, depot, f"fuzz_{seed}", f"bridge span {span}"
    if kind == "cantilever":
        length = 2 + rng.randrange(3)  # 2..4
        world, depot = build_cantilever(length)
        return world, depot, f"fuzz_{seed}", f"cantilever {length}"
    # Ledge: a 3×2 wall plus a 2-voxel nose. Tip overhang is 2.
    world, depot = _depot_and_world(4, 5, 12)
    world.blueprint[1:3, 2, 3:6] = True
    world.blueprint[2, 2, 6:8] = True
    return world, depot, f"fuzz_{seed}", "ledge nose 2"


def named_shapes() -> list[tuple[str, str, str, int | None, Builder]]:
    """(name, title, detail, blueprint_seed, builder)."""
    return [
        ("wall", "wall 4×1×3", "wall 4x1x3", None, build_wall),
        ("hollow_box", "hollow box 3×3×2", "hollow 3x3x2", None, build_hollow_box),
        ("bridge", "bridge span 5", "bridge span 5", None, lambda: build_bridge(5)),
        (
            "long_bridge",
            "bridge span 6",
            "bridge span 6",
            None,
            lambda: build_bridge(6),
        ),
        ("cantilever", "cantilever 3", "cantilever 3", None, lambda: build_cantilever(3)),
        (
            "cantilever_long",
            "cantilever 4",
            "cantilever 4",
            None,
            lambda: build_cantilever(4),
        ),
    ]


def finished_stable(world: World3D, max_cantilever: int | None) -> bool:
    """Whether the finished blueprint, all at once, passes the screen.

    This is not an order. A ``False`` here means no construction order
    can be stable. A ``True`` does not mean an order exists: a prefix
    can still overhang farther than the finished structure.
    """
    policy = CubicLattice3D(world).stability_policy(max_cantilever)
    cells = [tuple(int(v) for v in idx) for idx in zip(*world.blueprint.nonzero())]
    return is_stable(cells, policy)


def start_cells(world: World3D, depot: tuple[int, int, int], n: int, seed: int) -> list[tuple[int, int, int]]:
    """``n`` distinct level-1 cells beside the blueprint, not on it and not under it."""
    footprint = {
        (int(r), int(c))
        for _level, r, c in zip(*world.blueprint.nonzero())
    }
    ground = [
        (1, r, c)
        for r in range(world.rows)
        for c in range(world.cols)
        if (r, c) not in footprint and (1, r, c) != depot
    ]
    if n > len(ground):
        raise RuntimeError(f"need {n} starts, only {len(ground)} open ground cells")
    rng = random.Random(seed)
    chosen: list[tuple[int, int, int]] = []
    while len(chosen) < n:
        cell = ground[rng.randrange(len(ground))]
        if cell not in chosen:
            chosen.append(cell)
    return chosen


def _policy(world: World3D, condition: str, limit: int | None):
    if condition == GATE_OFF:
        return None
    return CubicLattice3D(world).stability_policy(max_cantilever=limit)


def _blank_numbers() -> dict:
    return {
        "completed": "",
        "makespan": "",
        "idle_robot_ticks": "",
        "kickbacks": "",
        "peak_sag": "",
        "final_sag": "",
        "sag_errors": "",
        "sag_samples": "",
    }


def _sag_from_log(log: RunLog) -> tuple[float | None, int, int]:
    """Peak downward sag over samples that solved, plus error and sample counts."""
    peak: float | None = None
    errors = 0
    samples = 0
    for frame in log.frames:
        sample = frame.get("structure")
        if not sample:
            continue
        if "error" in sample:
            errors += 1
            continue
        samples += 1
        sag = float(sample["worst_sag"])
        peak = sag if peak is None else max(peak, sag)
    return peak, errors, samples


def _drive_swarm(
    world: World3D,
    depot: tuple[int, int, int],
    starts: list[tuple[int, int, int]],
    stability,
    measure,
    seed: int,
) -> dict:
    """Build with the gate the planner already accepted. Returns build metrics."""
    _BUILDABILITY_CACHE.clear()
    swarm = Swarm(
        world,
        depot,
        starts=starts,
        geometry_factory=CubicLattice3D,
        stability=stability,
        defect_p=0.0,
        rng=random.Random(seed),
        inspect_enabled=True,
    )
    log = RunLog(
        structure=StructureLog(measure, k=K, mass=MASS, g=G, pitch=1.0, every=1)
    )
    log.log_tick(0, world, robots=swarm.robots)
    idle = 0
    stuck = ""
    try:
        while not swarm.settled and swarm.tick_count < MAX_TICKS:
            swarm.tick()
            idle += sum(1 for robot in swarm.robots if robot.state == "IDLE")
            log.log_tick(swarm.tick_count, world, robots=swarm.robots)
    except SwarmStuck as exc:
        stuck = str(exc)
    peak, errors, samples = _sag_from_log(log)
    final: float | None
    try:
        final = float(estimate_sag(occupied_voxels(world), measure, k=K, mass=MASS, g=G).worst_sag)
    except ValueError:
        final = None
    completed = bool(world.complete and swarm.settled and not stuck)
    return {
        "completed": completed,
        "makespan": swarm.tick_count if completed else "",
        "idle_robot_ticks": idle,
        "kickbacks": swarm.kickbacks,
        "peak_sag": "" if peak is None else peak,
        "final_sag": "" if final is None else final,
        "sag_errors": errors,
        "sag_samples": samples,
        "stuck": stuck,
    }


def run_trial(
    *,
    blueprint: str,
    family: str,
    detail: str,
    blueprint_seed: int | None,
    title: str,
    builder: Builder,
    condition: str,
    limit: int | None,
    start_seed: int,
    n_robots: int,
    max_nodes: int = MAX_NODES,
) -> dict:
    """One cell of the campaign. ``title`` is for the chart, not the CSV."""
    world, depot = builder()
    n_voxels = int(world.blueprint_count)
    stability = _policy(world, condition, limit)
    measure = CubicLattice3D(world).stability_policy(max_cantilever=None)
    if condition == GATE_OFF:
        finished = ""
    else:
        finished = "yes" if finished_stable(world, limit) else "no"

    scratch = world.copy()
    t0 = time.perf_counter()
    try:
        plan = plan_build_order(
            scratch,
            depot,
            CubicLattice3D,
            max_nodes=max_nodes,
            stability=stability,
        )
    except SearchBudgetExceeded:
        plan = "budget"
    plan_s = time.perf_counter() - t0

    if plan == "budget":
        status = "undecided"
    elif plan is None:
        status = "infeasible"
    else:
        status = "feasible"

    row = {
        "blueprint": blueprint,
        "family": family,
        "detail": detail,
        "title": title,
        "blueprint_seed": "" if blueprint_seed is None else blueprint_seed,
        "start_seed": start_seed,
        "n_robots": n_robots,
        "n_voxels": n_voxels,
        "depot": ",".join(str(v) for v in depot),
        "gate": "off" if condition == GATE_OFF else "on",
        "max_cantilever": "" if condition == GATE_OFF else ("none" if limit is None else str(limit)),
        "condition": condition,
        "status": status,
        "finished_stable": finished,
        "plan_s": plan_s,
        "max_nodes": max_nodes,
        "k": K,
        "mass": MASS,
        "g": G,
        "python": platform.python_version(),
        "stuck": "",
    }
    row.update(_blank_numbers())
    if status != "feasible":
        return row

    starts = start_cells(world, depot, n_robots, start_seed)
    built = _drive_swarm(world, depot, starts, stability, measure, start_seed)
    row["stuck"] = built.pop("stuck")
    row.update(built)
    return row


def iter_named_jobs() -> Iterable[dict]:
    for name, title, detail, blueprint_seed, builder in named_shapes():
        for condition, limit in CONDITIONS:
            for n_robots in ROBOT_COUNTS:
                for seed in START_SEEDS:
                    yield {
                        "blueprint": name,
                        "family": "named",
                        "detail": detail,
                        "title": title,
                        "blueprint_seed": blueprint_seed,
                        "builder": builder,
                        "condition": condition,
                        "limit": limit,
                        "start_seed": seed,
                        "n_robots": n_robots,
                    }


def iter_fuzz_jobs() -> Iterable[dict]:
    for blueprint_seed in FUZZ_SEEDS:
        _world, _depot, name, detail = build_fuzz(blueprint_seed)
        title = f"fuzz {detail}"

        def builder(seed: int = blueprint_seed) -> tuple[World3D, tuple[int, int, int]]:
            world, depot, _name, _detail = build_fuzz(seed)
            return world, depot

        for condition, limit in CONDITIONS:
            for n_robots in ROBOT_COUNTS:
                for seed in FUZZ_START_SEEDS:
                    yield {
                        "blueprint": name,
                        "family": "fuzz",
                        "detail": detail,
                        "title": title,
                        "blueprint_seed": blueprint_seed,
                        "builder": builder,
                        "condition": condition,
                        "limit": limit,
                        "start_seed": seed,
                        "n_robots": n_robots,
                    }


def _job_to_trial(job: dict, max_nodes: int) -> dict:
    return run_trial(
        blueprint=job["blueprint"],
        family=job["family"],
        detail=job["detail"],
        blueprint_seed=job["blueprint_seed"],
        title=job["title"],
        builder=job["builder"],
        condition=job["condition"],
        limit=job["limit"],
        start_seed=job["start_seed"],
        n_robots=job["n_robots"],
        max_nodes=max_nodes,
    )


def run_jobs(jobs: Iterable[dict], *, max_nodes: int = MAX_NODES, progress: bool = True) -> list[dict]:
    rows = []
    job_list = list(jobs)
    for index, job in enumerate(job_list, start=1):
        row = _job_to_trial(job, max_nodes)
        rows.append(row)
        if progress:
            sag = row["peak_sag"]
            sag_s = "" if sag == "" else f" peak={float(sag):.3f}"
            makespan = row["makespan"]
            tick_s = "" if makespan == "" else f" ticks={makespan}"
            print(
                f"[{index}/{len(job_list)}] {row['blueprint']} "
                f"gate={row['condition']} N={row['n_robots']} "
                f"seed={row['start_seed']} {row['status']} "
                f"plan={row['plan_s']:.3f}s{tick_s}{sag_s}",
                flush=True,
            )
    return rows


def smoke_jobs() -> list[dict]:
    """Wall (feasible at limit 0) and a length-3 cantilever (not).

    One robot, one start seed, two gate settings each. The cantilever
    rejection is a full proof, so the smoke run covers both statuses.
    """
    shapes = {
        "wall": named_shapes()[0],
        "cantilever": named_shapes()[4],
    }
    picked = (
        ("wall", GATE_OFF, None),
        ("wall", "0", 0),
        ("cantilever", "0", 0),
        ("cantilever", "none", None),
    )
    jobs = []
    for name, condition, limit in picked:
        _blueprint, title, detail, blueprint_seed, builder = shapes[name]
        jobs.append(
            {
                "blueprint": name,
                "family": "named",
                "detail": detail,
                "title": title,
                "blueprint_seed": blueprint_seed,
                "builder": builder,
                "condition": condition,
                "limit": limit,
                "start_seed": 0,
                "n_robots": 1,
            }
        )
    return jobs


def _csv_value(value) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.6f}"
    return str(value)


def write_csv(rows: list[dict], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv_value(row.get(key, "")) for key in CSV_FIELDS})
    return path


def _completed(row: dict) -> bool:
    return row.get("completed") is True


def _nums(rows: list[dict], key: str) -> list[float]:
    out = []
    for row in rows:
        value = row.get(key, "")
        if value == "" or value is None:
            continue
        out.append(float(value))
    return out


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    n = len(ordered)
    mid = n // 2
    if n % 2:
        return ordered[mid]
    return (ordered[mid - 1] + ordered[mid]) / 2


def _spread(values: list[float]) -> str:
    if not values:
        return "n=0"
    return (
        f"n={len(values)} median={_median(values):.4g} "
        f"min={min(values):.4g} max={max(values):.4g}"
    )


def summarize(rows: list[dict]) -> str:
    """Human-readable spreads. This is what the writeup is checked against."""
    lines = [
        f"trials={len(rows)} python={platform.python_version()} "
        f"max_nodes={MAX_NODES} k={K} mass={MASS} g={G}",
        f"named start seeds={list(START_SEEDS)} robots={list(ROBOT_COUNTS)} "
        f"fuzz blueprint seeds={list(FUZZ_SEEDS)} "
        f"fuzz start seeds={list(FUZZ_START_SEEDS)}",
        "Sag is estimate_sag worst_sag at k=1 (model units of mass*g/k), "
        "not a calibrated displacement.",
    ]
    groups: dict[tuple, list[dict]] = defaultdict(list)
    for row in rows:
        groups[(row["family"], row["blueprint"], row["condition"])].append(row)
    current_bp = None
    for (family, blueprint, condition), group in groups.items():
        if blueprint != current_bp:
            detail = group[0]["detail"]
            voxels = group[0]["n_voxels"]
            lines.append(f"\n{family} {blueprint} ({detail}, {voxels} voxels)")
            current_bp = blueprint
        counts = defaultdict(int)
        for row in group:
            counts[row["status"]] += 1
        finished = group[0]["finished_stable"]
        lines.append(
            f"  {condition}: status {dict(counts)} finished_stable={finished or 'n/a'} "
            f"plan_s[{_spread(_nums(group, 'plan_s'))}]"
        )
        done = [row for row in group if _completed(row)]
        if done:
            lines.append(
                f"    completed {len(done)}/{len(group)} "
                f"makespan[{_spread(_nums(done, 'makespan'))}] "
                f"kickbacks[{_spread(_nums(done, 'kickbacks'))}] "
                f"idle[{_spread(_nums(done, 'idle_robot_ticks'))}] "
                f"peak_sag[{_spread(_nums(done, 'peak_sag'))}] "
                f"final_sag[{_spread(_nums(done, 'final_sag'))}] "
                f"sag_errors[{_spread(_nums(done, 'sag_errors'))}]"
            )
            for n_robots in sorted({int(row["n_robots"]) for row in done}):
                subset = [row for row in done if int(row["n_robots"]) == n_robots]
                lines.append(
                    f"      N={n_robots} "
                    f"makespan[{_spread(_nums(subset, 'makespan'))}] "
                    f"kickbacks[{_spread(_nums(subset, 'kickbacks'))}] "
                    f"idle[{_spread(_nums(subset, 'idle_robot_ticks'))}] "
                    f"peak_sag[{_spread(_nums(subset, 'peak_sag'))}]"
                )
        incomplete = [row for row in group if row["status"] == "feasible" and not _completed(row)]
        for row in incomplete:
            lines.append(
                f"    INCOMPLETE N={row['n_robots']} seed={row['start_seed']} "
                f"stuck={row.get('stuck', '')}"
            )
    return "\n".join(lines)


def _condition_order() -> list[str]:
    return [condition for condition, _limit in CONDITIONS]


def _by_blueprint(rows: list[dict]) -> list[tuple[str, str, list[dict]]]:
    """Named shapes in campaign order, then fuzz shapes in seed order."""
    order: list[tuple[str, str]] = []
    seen = set()
    for _name, title, _detail, _seed, _builder in named_shapes():
        order.append((_name, title))
        seen.add(_name)
    fuzz = []
    for row in rows:
        if row["blueprint"] in seen:
            continue
        if row["blueprint"] not in {name for name, _title in fuzz}:
            fuzz.append((row["blueprint"], row["title"]))
    # Named first, but drop named shapes that this row set does not contain
    # (the smoke run). Fuzz follows.
    present = {row["blueprint"] for row in rows}
    ordered = [(name, title) for name, title in order if name in present]
    ordered.extend(fuzz)
    grouped = []
    for name, title in ordered:
        grouped.append((name, title, [row for row in rows if row["blueprint"] == name]))
    return grouped


def _style(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.6, zorder=0)
    ax.tick_params(labelsize=8)


def _plot_metric(
    rows: list[dict],
    path: Path,
    *,
    key: str,
    ylabel: str,
    title: str,
    completed_only: bool,
) -> Path:
    """Small multiples of the named shapes. Every trial is a point; the tick is the median.

    Fuzz rows stay in the CSV. Pooling their sag would mix different shapes.
    Infeasible and undecided categories are labeled, not drawn as zero.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    named = [row for row in rows if row["family"] == "named"]
    panels = _by_blueprint(named or rows)
    # Charts in the writeup are the named shapes. A smoke run may be smaller.
    ncols = 3 if len(panels) > 1 else 1
    nrows = (len(panels) + ncols - 1) // ncols
    fig, axes = plt.subplots(
        nrows, ncols, figsize=(3.4 * ncols, 2.8 * nrows), squeeze=False
    )
    conditions = _condition_order()
    x_labels = ["off", "0", "1", "2", "3", "∞"]
    colors = {1: "#93c5fd", 2: "#1d4ed8", 3: "#1e3a8a"}
    seen_labels: set[str] = set()

    for index, (_name, title_text, group) in enumerate(panels):
        ax = axes[index // ncols][index % ncols]
        _style(ax)
        xs, plotted = [], False
        ymax = 0.0
        for xpos, condition in enumerate(conditions):
            cell = [row for row in group if row["condition"] == condition]
            if completed_only:
                cell = [row for row in cell if _completed(row)]
            values_rows = []
            for row in cell:
                raw = row.get(key, "")
                if raw == "" or raw is None:
                    continue
                values_rows.append(row)
            if not values_rows:
                statuses = {row["status"] for row in group if row["condition"] == condition}
                if statuses == {"infeasible"}:
                    label = "rejected"
                elif statuses == {"undecided"}:
                    label = "undecided"
                elif not statuses:
                    continue
                else:
                    label = ",".join(sorted(statuses))
                ax.text(
                    xpos, 0.5, label, transform=ax.get_xaxis_transform(),
                    ha="center", va="center", fontsize=7, color="#6b7280",
                )
                continue
            values = [float(row[key]) for row in values_rows]
            ymax = max(ymax, max(values))
            median = _median(values)
            ax.plot(
                [xpos - 0.28, xpos + 0.28], [median, median],
                color="#111827", linewidth=1.6, zorder=4, solid_capstyle="butt",
            )
            for row, value in zip(values_rows, values):
                # Stable jitter from the seed, so a replot does not move the dots.
                jitter = ((int(row["start_seed"]) * 5 + int(row["n_robots"])) % 5 - 2) * 0.06
                n = int(row["n_robots"])
                label = f"N={n}"
                ax.scatter(
                    xpos + jitter, value, s=18, color=colors.get(n, "#2563eb"),
                    zorder=3, label=label if label not in seen_labels else None,
                )
                seen_labels.add(label)
            plotted = True
            xs.append(xpos)
        ax.set_xlim(-0.6, len(conditions) - 0.4)
        ax.set_xticks(range(len(conditions)), x_labels)
        ax.set_title(title_text, loc="left", fontsize=9)
        if plotted:
            ax.set_ylim(0, ymax * 1.18 if ymax > 0 else 1)
        else:
            ax.set_ylim(0, 1)
        if index // ncols == nrows - 1:
            ax.set_xlabel("max cantilever", fontsize=8)
        if index % ncols == 0:
            ax.set_ylabel(ylabel, fontsize=8)

    for index in range(len(panels), nrows * ncols):
        axes[index // ncols][index % ncols].set_axis_off()

    fig.suptitle(title, x=0.01, ha="left", fontsize=11)
    unique = {}
    for ax in axes.ravel():
        handles, labels = ax.get_legend_handles_labels()
        for handle, label in zip(handles, labels):
            unique.setdefault(label, handle)
    if unique:
        fig.legend(
            unique.values(), unique.keys(),
            loc="upper right", bbox_to_anchor=(0.99, 0.99),
            frameon=False, fontsize=8, ncol=len(unique),
        )
    fig.tight_layout(rect=(0, 0, 1, 0.90))
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def _plot_feasibility(rows: list[dict], path: Path) -> Path:
    """Fraction of trials whose search proved a feasible order.

    Undecided trials are written in the cell and are not counted as
    rejections. Color is the proved-feasible share of decided trials.
    """
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    panels = _by_blueprint(rows)
    # Aggregate fuzz into one row so the named mechanism stays readable,
    # and keep per-fuzz detail in the CSV.
    named = [(name, title, group) for name, title, group in panels if group and group[0]["family"] == "named"]
    fuzz_rows = [row for row in rows if row["family"] == "fuzz"]
    if fuzz_rows:
        named.append(("fuzz", "fuzz (pooled)", fuzz_rows))
    conditions = _condition_order()
    fig, ax = plt.subplots(figsize=(7.2, 0.55 * len(named) + 1.4))
    for y, (_name, title, group) in enumerate(named):
        for x, condition in enumerate(conditions):
            cell = [row for row in group if row["condition"] == condition]
            if not cell:
                continue
            feasible = sum(1 for row in cell if row["status"] == "feasible")
            infeasible = sum(1 for row in cell if row["status"] == "infeasible")
            undecided = sum(1 for row in cell if row["status"] == "undecided")
            decided = feasible + infeasible
            if decided == 0:
                color = "#e5e7eb"
                text = f"{undecided} und."
            else:
                share = feasible / decided
                # Rejected cells are light red, proved-feasible cells are blue.
                color = (
                    0.99 - share * 0.84,
                    0.89 - share * 0.34,
                    0.89 + share * 0.03,
                )
                text = f"{feasible}/{decided}"
                if undecided:
                    text = f"{feasible}/{decided}\n+{undecided} und."
            ax.add_patch(
                Rectangle((x, y), 1, 1, facecolor=color, edgecolor="white", linewidth=2)
            )
            ax.text(x + 0.5, y + 0.5, text, ha="center", va="center", fontsize=7.5, color="#111827")
    ax.set_xlim(0, len(conditions))
    ax.set_ylim(0, len(named))
    ax.set_xticks([i + 0.5 for i in range(len(conditions))], ["off", "0", "1", "2", "3", "∞"])
    ax.set_yticks([i + 0.5 for i in range(len(named))], [title for _n, title, _g in named])
    ax.tick_params(length=0)
    ax.set_xlabel("max cantilever  (∞ = gate on, no span limit)")
    ax.set_title(
        "Plan feasibility  —  proved feasible / decided    "
        "undecided is not a rejection",
        loc="left", fontsize=11,
    )
    for spine in ax.spines.values():
        spine.set_visible(False)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=150)
    plt.close(fig)
    return path


def write_charts(rows: list[dict], out_dir: Path) -> list[Path]:
    out_dir.mkdir(parents=True, exist_ok=True)
    paths = [
        _plot_metric(
            rows, out_dir / "stability_peak_sag.png",
            key="peak_sag", ylabel="peak sag (mg/k)",
            title="Peak sag during construction — k = 1, not fitted to a printed part",
            completed_only=True,
        ),
        _plot_metric(
            rows, out_dir / "stability_makespan.png",
            key="makespan", ylabel="ticks to complete",
            title="Build makespan — no defects, inspect on",
            completed_only=True,
        ),
        _plot_metric(
            rows, out_dir / "stability_kickbacks.png",
            key="kickbacks", ylabel="kickbacks",
            title="Kickbacks — a refused placement is handed back",
            completed_only=True,
        ),
        _plot_metric(
            rows, out_dir / "stability_idle.png",
            key="idle_robot_ticks", ylabel="idle robot-ticks",
            title="Idle robot-ticks — summed over robots after each tick",
            completed_only=True,
        ),
        _plot_metric(
            rows, out_dir / "stability_planning.png",
            key="plan_s", ylabel="seconds",
            title=f"Planning wall time — one search, capped at {MAX_NODES} nodes",
            completed_only=False,
        ),
        _plot_feasibility(rows, out_dir / "stability_feasibility.png"),
    ]
    return paths


def load_rows(path: str | Path) -> list[dict]:
    """Read a campaign CSV back into the row dicts the chart code expects."""
    rows = []
    with Path(path).open(newline="", encoding="utf-8") as handle:
        for raw in csv.DictReader(handle):
            row = dict(raw)
            gate = row["gate"]
            limit = row["max_cantilever"]
            if gate == "off":
                row["condition"] = GATE_OFF
            elif limit == "none":
                row["condition"] = "none"
            else:
                row["condition"] = limit
            row["title"] = row["detail"] if row["family"] == "named" else f"fuzz {row['detail']}"
            if row["completed"] == "1":
                row["completed"] = True
            elif row["completed"] == "0":
                row["completed"] = False
            else:
                row["completed"] = ""
            for key in ("plan_s", "peak_sag", "final_sag"):
                if row.get(key, "") != "":
                    row[key] = float(row[key])
            for key in (
                "makespan", "idle_robot_ticks", "kickbacks", "sag_errors",
                "sag_samples", "n_voxels", "n_robots", "start_seed",
            ):
                if row.get(key, "") != "":
                    row[key] = int(float(row[key]))
            rows.append(row)
    return rows


def write_outputs(rows: list[dict], out_dir: str | Path) -> dict:
    out = Path(out_dir)
    csv_path = write_csv(rows, out / "stability_cost.csv")
    summary = summarize(rows)
    summary_path = out / "stability_cost_summary.txt"
    summary_path.write_text(summary + "\n", encoding="utf-8")
    charts = write_charts(rows, out)
    print(summary, flush=True)
    print(f"csv: {csv_path}", flush=True)
    print(f"summary: {summary_path}", flush=True)
    for chart in charts:
        print(f"chart: {chart}", flush=True)
    return {"csv": csv_path, "summary": summary_path, "charts": charts, "rows": rows}


def run_campaign(out_dir: str | Path = "docs", *, max_nodes: int = MAX_NODES) -> dict:
    jobs = list(iter_named_jobs()) + list(iter_fuzz_jobs())
    rows = run_jobs(jobs, max_nodes=max_nodes, progress=True)
    return write_outputs(rows, out_dir)


def run_smoke(out_dir: str | Path = "runs/stability_smoke", *, max_nodes: int = MAX_NODES) -> dict:
    rows = run_jobs(smoke_jobs(), max_nodes=max_nodes, progress=True)
    return write_outputs(rows, out_dir)


def main(argv: list[str] | None = None) -> None:
    args = list(argv or [])
    if args and args[0] == "smoke":
        out = args[1] if len(args) > 1 else "runs/stability_smoke"
        run_smoke(out)
        return
    if args and args[0] == "replot":
        csv_path = Path(args[1] if len(args) > 1 else "docs/stability_cost.csv")
        out = csv_path.parent
        rows = load_rows(csv_path)
        write_charts(rows, out)
        summary = summarize(rows)
        (out / "stability_cost_summary.txt").write_text(summary + "\n", encoding="utf-8")
        print(summary, flush=True)
        return
    out = args[0] if args else "docs"
    run_campaign(out)


if __name__ == "__main__":
    import sys

    main(sys.argv[1:])
