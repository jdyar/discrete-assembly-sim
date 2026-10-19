# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Run the public benchmark suite and write a results table.

    python main.py bench
    python main.py bench smoke
    python main.py bench bridge-span-4 fault-tower

Every scenario is a ``1.0.0`` document under ``benchmarks/tasks``.
The stock planner is the one already in this repository: build-order
search, the scaffold search, the three-role shelter schedule, and the
2D wall robot. No second planner is added here.

``k = mass = g = 1``. Sag is ``mass*g/k``. Joint utilization is demand
divided by the finished structure's own peak demand at that ``k``.
Neither number is a fit, and neither is a measured connector strength.
The derived ARMADAS face compression of 1057.2365952 N is not applied.

Fault rates are configuration. A scripted channel is a draw with its
rate left at zero. Recovery names are the ones in :mod:`sim.faults`.
"""

from __future__ import annotations

import csv
import statistics
from pathlib import Path
from typing import Any

from sim.armadas import shelter_cells, shelter_model, shelter_seed, simulate
from sim.cuboct import CuboctahedralLattice3D
from sim.faults import FaultInjector, FaultRates
from sim.format import (
    assert_unfitted_suite,
    events_from_injector,
    events_from_swarm,
    gate_event,
    load,
    make_event_log,
    pose_node,
    world_from_blueprint,
)
from sim.geometry3d import CubicLattice3D, MotionModel
from sim.inchworm import InchwormConfig, OnLatticeGeometry
from sim.metrics import RunLog, StructureLog
from sim.planner import SearchBudgetExceeded, plan_build_order, stability_precheck
from sim.scaffold import caps_for_finished, finished_cells, plan_with_scaffold
from sim.scenarios import SUITE
from sim.stability import (
    JointCap,
    JointStrength,
    StabilityPolicy,
    estimate_sag,
    is_stable,
    joint_loads,
    joint_utilizations,
    occupied_voxels,
    sag_within,
)
from sim.swarm import Swarm, SwarmStuck
from sim.world import DEFECT, EMPTY, SCAFFOLD, VOXEL

SMOKE = (
    "wall-2d",
    "fault-wall",
    "hollow-roof-limit-0",
    "cuboct-block",
    "bridge-span-4",
    "cantilever-arm",
    "tower",
    "fault-tower",
    "scaffold-span-4-crane",
)

COLUMNS = (
    "id",
    "family",
    "status",
    "completion",
    "makespan",
    "makespan_unit",
    "peak_sag",
    "peak_sag_units",
    "joint_utilization",
    "scaffold_count",
    "faults",
    "recoveries",
    "cascades",
    "unrecovered",
    "recovery_time",
    "gate_accepts",
    "gate_refusals",
    "k",
    "fitted",
    "n_target",
    "n_structural",
    "note",
)

_SAG_UNITS = "mass*g/k"
_NOTE = (
    "k = 1 is not a fit. Sag is mass*g/k. "
    "Joint utilization is demand over the finished structure's own peak "
    "demand at that k, not a measured strength. "
    "Fault rates are configuration."
)


def tasks_dir(root: Path | None = None) -> Path:
    return (Path("benchmarks") if root is None else Path(root)) / "tasks"


def load_scenario(ident: str, root: Path | None = None) -> dict:
    return load(tasks_dir(root) / f"{ident}.json")


def run_ids(idents: list[str], root: Path | None = None) -> list[dict]:
    """Run each scenario id. The event log is stashed on the row."""
    rows = []
    for ident in idents:
        scenario = load_scenario(ident, root)
        if scenario["id"] != ident:
            raise RuntimeError(f"{ident} file id is {scenario['id']}")
        rows.append(run_scenario(scenario))
    return rows


def run_scenario(scenario: dict) -> dict:
    """One row. Physics uses ``k = mass = g = 1``."""
    assert_unfitted_suite(scenario["blueprint"])
    runner = scenario["task"]["runner"]
    if runner == "precheck":
        row = _precheck(scenario)
    elif runner == "wall2d":
        row = _wall2d(scenario)
    elif runner == "armadas":
        row = _shelter(scenario)
    elif runner == "scaffold":
        row = _scaffold(scenario)
    elif runner == "swarm":
        row = _swarm(scenario)
    else:
        raise RuntimeError(f"unknown runner {runner}")
    row["event_log"] = make_event_log(
        scenario_id=scenario["id"],
        clock=row.pop("_clock"),
        makespan=float(row["makespan"] or 0),
        events=row.pop("_events"),
    )
    return row


def write_table(rows: list[dict], csv_path: Path, markdown_path: Path | None = None) -> None:
    """CSV of every column, and a markdown table of the reported ones."""
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow({key: _csv(row.get(key)) for key in COLUMNS})
    if markdown_path is not None:
        markdown_path.write_text(markdown_table(rows), encoding="utf-8")


def markdown_table(rows: list[dict]) -> str:
    """The columns a lab compares: completion through fault recovery."""
    header = (
        "| id | status | completion | makespan | peak sag | "
        "joint utilization | scaffold | faults | recoveries | cascades |\n"
        "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |\n"
    )
    lines = [header]
    for row in rows:
        unit = row["makespan_unit"]
        makespan = _blank(row["makespan"])
        if makespan != "" and unit == "second":
            makespan = f"{float(row['makespan']) / 3600:.3f} h"
        elif makespan != "":
            makespan = f"{makespan} {unit}"
        sag = _blank(row["peak_sag"])
        if sag != "":
            sag = f"{sag} {_SAG_UNITS}"
        recovered = row["recoveries"]
        lines.append(
            "| {id} | {status} | {completion} | {makespan} | {sag} | "
            "{util} | {scaffold} | {faults} | {recovered} | {cascades} |\n".format(
                id=row["id"],
                status=row["status"],
                completion=_blank(row["completion"]),
                makespan=makespan,
                sag=sag,
                util=_blank(row["joint_utilization"]),
                scaffold=_blank(row["scaffold_count"]),
                faults=_blank(row["faults"]),
                recovered=_blank(recovered),
                cascades=_blank(row["cascades"]),
            )
        )
    lines.append(
        "\n`k = mass = g = 1`. Sag is mass·g/k. "
        "Joint utilization uses the finished structure's own peak demand "
        "as capacity. Stiffness is not a fit. Fault rates are configuration. "
        "Shelter makespan is hours; other rows are ticks.\n"
    )
    return "".join(lines)


def replay_completion(scenario: dict, event_log: dict) -> float:
    """Completion after replaying placements and removals onto the blueprint.

    A defect is replayed as a voxel of the logged type. The final
    completion matches a run that replaced every bad bond.
    """
    world = world_from_blueprint(scenario["blueprint"])
    for event in event_log["events"]:
        if event["type"] == "placement":
            cell = tuple(event["cell"])
            if int(world.occupancy[cell]) == EMPTY:
                world.place_voxel(cell, scaffold=event["voxel_type"] == "scaffold")
        elif event["type"] == "removal":
            cell = tuple(event["cell"])
            if int(world.occupancy[cell]) in (VOXEL, DEFECT, SCAFFOLD):
                world.remove_voxel(cell)
    return _completion(world, scenario["blueprint"])


# --- runners ----------------------------------------------------------------


def _precheck(scenario: dict) -> dict:
    world = world_from_blueprint(scenario["blueprint"])
    policy, _sag = _caps(world, scenario["task"])
    screen = stability_precheck(world, policy)
    if screen.rejected:
        events = [
            gate_event(
                t=0,
                gate="stability",
                verdict="refuse",
                cell=screen.cell,
                reason=screen.reason or "cantilever",
            )
        ]
        status = "infeasible"
    else:
        events = [
            gate_event(t=0, gate="stability", verdict="accept", reason="precheck")
        ]
        status = "feasible"
    return _row(
        scenario,
        world,
        status=status,
        makespan=0,
        unit="tick",
        events=events,
        scaffold_count=0,
    )


def _wall2d(scenario: dict) -> dict:
    from main import run_slice1, run_slice2

    world = world_from_blueprint(scenario["blueprint"])
    robots = scenario["robots"]
    if robots["defect_p"] > 0:
        log, robot = run_slice2(
            world,
            defect_p=float(robots["defect_p"]),
            seed=int(robots["rng_seed"]),
            correction=bool(robots["inspect"]),
            max_ticks=int(scenario["task"].get("max_ticks", 4000)),
            verbose=False,
        )
    else:
        log, robot = run_slice1(world, verbose=False)
    events = _events_2d(log)
    status = "complete" if world.complete else "incomplete"
    return _row(
        scenario,
        world,
        status=status,
        makespan=log.ticks,
        unit="tick",
        events=events,
        scaffold_count=0,
        peak_sag=None,
        utilization=None,
        extra="2D wall: the sag model is the 3D spring network, so peak sag is blank.",
    )


def _shelter(scenario: dict) -> dict:
    wanted = {tuple(cell) for cell in shelter_cells()}
    documented = {
        tuple(item["index"])
        for item in scenario["blueprint"]["cells"]
    }
    if documented != wanted:
        raise RuntimeError("shelter scenario cells are not shelter_cells()")
    if tuple(tuple(cell) for cell in scenario["blueprint"]["seed"]) != tuple(
        tuple(cell) for cell in shelter_seed()
    ):
        raise RuntimeError("shelter seed does not match shelter_seed()")
    task = scenario["task"]
    model = shelter_model()
    result = simulate(
        model,
        n_feeds=int(task.get("n_feeds", 1)),
        robots_per_role=int(task.get("robots_per_role", 1)),
    )
    events = []
    for time_s, role, action, index in result.events:
        if role != "crane" or action != "place":
            continue
        cell = model.order[index]
        events.append(
            {
                "t": float(time_s),
                "type": "placement",
                "cell": [int(v) for v in cell],
                "role": role,
                "voxel_type": "structural",
            }
        )
        events.append(
            gate_event(
                t=float(time_s),
                gate="stability",
                verdict="accept",
                cell=cell,
                reason="prefix",
            )
        )
        events.append(
            gate_event(
                t=float(time_s),
                gate="sag",
                verdict="accept",
                cell=cell,
                reason="prefix",
            )
        )
    world = world_from_blueprint(scenario["blueprint"])
    for cell in model.order:
        world.place_voxel(tuple(cell))
    strength = _reference_strength(world, model.policy)
    utilization = _prefix_utilization(model.seed, model.order, model.policy, strength)
    # The schedule's peak sag is the prefix peak at k = 1, voxels only.
    return _row(
        scenario,
        world,
        status="complete" if world.complete else "incomplete",
        makespan=float(result.makespan_s),
        unit="second",
        events=events,
        scaffold_count=0,
        peak_sag=float(result.peak_sag),
        utilization=utilization,
    )


def _swarm(scenario: dict) -> dict:
    return _build(scenario, scaffold=False)


def _scaffold(scenario: dict) -> dict:
    return _build(scenario, scaffold=True)


def _build(scenario: dict, *, scaffold: bool) -> dict:
    task = scenario["task"]
    world = world_from_blueprint(scenario["blueprint"])
    policy, sag = _caps(world, task)
    factory = _factory(scenario, policy)
    robot = scenario["robots"]["robots"][0]
    depot = pose_node(robot["depot"])
    start = pose_node(robot["start"])
    max_nodes = int(task.get("max_nodes", 40000))
    max_ticks = int(task.get("max_ticks", 8000))
    joints = _joints(scenario)
    events: list[dict] = []
    plan_scaffold = ()
    try:
        if scaffold:
            plan = plan_with_scaffold(
                world,
                depot,
                factory,
                policy,
                sag,
                max_nodes=max_nodes,
                joints=joints,
            )
        else:
            plan = plan_build_order(
                world,
                depot,
                factory,
                max_nodes=max_nodes,
                stability=policy,
                sag=sag,
                joints=joints,
            )
    except SearchBudgetExceeded as exc:
        return _row(
            scenario,
            world,
            status="undecided",
            makespan=0,
            unit="tick",
            events=[
                gate_event(
                    t=0,
                    gate="buildability",
                    verdict="refuse",
                    reason=str(exc),
                )
            ],
            scaffold_count=None,
        )
    if plan is None:
        events.extend(_refusal(world, policy, sag))
        return _row(
            scenario,
            world,
            status="infeasible",
            makespan=0,
            unit="tick",
            events=events,
            scaffold_count=0 if not scaffold else None,
        )
    if scaffold:
        plan_scaffold = tuple(plan.scaffold)
        events.extend(_prefix_gates(world, [task.cell for task in plan.place], policy, sag))
        operations = plan.operations()
        scaffold_count = int(plan.scaffold_count)
    else:
        events.extend(_prefix_gates(world, [task.cell for task in plan], policy, sag))
        operations = None
        scaffold_count = 0
    live = world.copy()
    measure = policy if policy is not None else _frame(world)
    strength = _reference_strength(live, measure)
    structure = StructureLog(
        measure, k=1.0, mass=1.0, g=1.0, pitch=1.0, every=1, strength=strength
    )
    injector = _injector(scenario["robots"])
    swarm = Swarm(
        live,
        depot,
        [start],
        geometry_factory=factory,
        stability=policy,
        sag=sag,
        joints=joints,
        faults=injector,
        operations=operations,
        scaffold_cells=plan_scaffold,
        inspect_enabled=bool(scenario["robots"]["inspect"]),
        defect_p=0.0,
    )
    log = RunLog(structure=structure)
    log.log_tick(0, live, robots=swarm.robots)
    stuck = ""
    try:
        while swarm.tick_count < max_ticks:
            if swarm.done:
                break
            if swarm.settled and swarm.tick_count > 0:
                break
            swarm.tick()
            log.log_tick(swarm.tick_count, live, robots=swarm.robots)
    except SwarmStuck as exc:
        stuck = str(exc)
    events.extend(events_from_swarm(swarm))
    if injector is not None:
        events.extend(events_from_injector(injector))
    events.sort(key=lambda item: (float(item["t"]), item["type"]))
    peak, utilization = _from_log(log)
    if live.complete and not any(int(live.occupancy[cell]) == SCAFFOLD for cell in plan_scaffold):
        status = "complete" if swarm.done and not stuck else "incomplete"
    elif stuck:
        status = "incomplete"
    else:
        status = "incomplete"
    if status == "complete" and not swarm.done:
        status = "incomplete"
    return _row(
        scenario,
        live,
        status=status,
        makespan=swarm.tick_count,
        unit="tick",
        events=events,
        scaffold_count=scaffold_count,
        peak_sag=peak,
        utilization=utilization,
        extra=stuck,
    )


# --- measurements -----------------------------------------------------------


def _caps(world, task):
    gate = task["gate"]
    if gate == "off" or world.occupancy.ndim != 3:
        return None, None
    frame = CubicLattice3D(world).stability_policy(0)
    if gate == "cantilever-0":
        return frame, None
    if gate == "finished":
        return caps_for_finished(world, frame)
    raise RuntimeError(f"unknown gate {gate}")


def _frame(world) -> StabilityPolicy:
    if world.occupancy.ndim == 3:
        return StabilityPolicy(up_axis=0, up_sign=1, ground_plane=0, max_cantilever=None)
    return StabilityPolicy(up_axis=0, up_sign=-1, ground_plane=world.rows - 1, max_cantilever=None)


def _factory(scenario: dict, policy):
    kind = scenario["blueprint"]["lattice"]["type"]
    robot = scenario["robots"]["robots"][0]
    if robot["locomotion"] == "inchworm":
        gait = InchwormConfig(**robot["gait"])

        def factory(world, gait=gait, policy=policy):
            return OnLatticeGeometry(world, gait, policy)

        return factory
    motion = MotionModel(**robot["motion"])
    if kind == "cuboctahedral":

        def factory(world, motion=motion):
            return CuboctahedralLattice3D(world, motion)

        return factory

    def factory(world, motion=motion):
        return CubicLattice3D(world, motion)

    return factory


def _joints(scenario: dict) -> JointCap | None:
    # The fault-tower fixture names the face spring against a loose cap.
    # 10 is above the self-weight of two voxels at k = 1, so the cap
    # does not decide the order. It is not a measured strength.
    if scenario["id"] != "fault-tower":
        return None
    return JointCap(JointStrength(axial_compression=10.0, shear=10.0))


def _injector(robots: dict) -> FaultInjector | None:
    block = robots.get("faults")
    if not block:
        return None
    rates = block.get("rates") or {}
    return FaultInjector(
        FaultRates(
            capture=float(rates.get("capture", 0.0)),
            drop=float(rates.get("drop", 0.0)),
            stall=float(rates.get("stall", 0.0)),
            robot_fail=float(rates.get("robot_fail", 0.0)),
            fastener=float(rates.get("fastener", 0.0)),
            feed_outage=float(rates.get("feed_outage", 0.0)),
            stall_ticks=int(rates.get("stall_ticks", 3)),
            down_ticks=int(rates.get("down_ticks", 8)),
            outage_ticks=int(rates.get("outage_ticks", 6)),
            outage_s=float(rates.get("outage_s", 40.0)),
        ),
        seed=int(block["seed"]),
        script=list(block.get("script") or []),
    )


def _refusal(world, policy, sag) -> list[dict]:
    events = []
    if policy is not None:
        screen = stability_precheck(world, policy)
        if screen.rejected:
            events.append(
                gate_event(
                    t=0,
                    gate="stability",
                    verdict="refuse",
                    cell=screen.cell,
                    reason=screen.reason or "cantilever",
                )
            )
            return events
    if sag is not None and policy is not None:
        cells = finished_cells(world) if world.occupancy.ndim == 3 else occupied_voxels(world)
        if not sag_within(cells, policy, sag):
            events.append(gate_event(t=0, gate="sag", verdict="refuse", reason="finished sag"))
            return events
    events.append(
        gate_event(t=0, gate="buildability", verdict="refuse", reason="no order")
    )
    return events


def _prefix_gates(world, order, policy, sag) -> list[dict]:
    if policy is None and sag is None:
        return []
    built = set(occupied_voxels(world))
    events = []
    for cell in order:
        built.add(tuple(int(v) for v in cell))
        if policy is not None:
            ok = is_stable(built, policy)
            events.append(
                gate_event(
                    t=0,
                    gate="stability",
                    verdict="accept" if ok else "refuse",
                    cell=cell,
                    reason="prefix",
                )
            )
        if sag is not None and policy is not None:
            ok = sag_within(built, policy, sag)
            events.append(
                gate_event(
                    t=0,
                    gate="sag",
                    verdict="accept" if ok else "refuse",
                    cell=cell,
                    reason="prefix",
                )
            )
    return events


def _reference_strength(world, policy) -> JointStrength | None:
    if world.occupancy.ndim != 3:
        return None
    cells = finished_cells(world)
    if len(cells) < 2:
        return None
    try:
        estimate = estimate_sag(cells, policy, k=1.0, mass=1.0, g=1.0)
    except ValueError:
        return None
    loads = joint_loads(estimate, k=1.0)
    if not loads:
        return None
    compression = max(max(-load.axial, 0.0) for load in loads)
    tension = max(max(load.axial, 0.0) for load in loads)
    shear = max(load.shear for load in loads)

    def positive(value: float) -> float | None:
        return float(value) if value > 1e-9 else None

    strength = JointStrength(positive(compression), positive(tension), positive(shear))
    if (
        strength.axial_compression is None
        and strength.axial_tension is None
        and strength.shear is None
    ):
        return None
    return strength


def _prefix_utilization(seed, order, policy, strength: JointStrength | None) -> float | None:
    """Peak demand/capacity over the placement order, finished capacity."""
    if strength is None:
        return None
    built = {tuple(cell) for cell in seed}
    peak = _utilization(built, policy, strength)
    for cell in order:
        built.add(tuple(cell))
        value = _utilization(built, policy, strength)
        if value is None:
            continue
        peak = value if peak is None else max(peak, value)
    return peak


def _utilization(cells, policy, strength: JointStrength | None) -> float | None:
    if strength is None or len(tuple(cells)) < 2:
        return None
    try:
        estimate = estimate_sag(cells, policy, k=1.0, mass=1.0, g=1.0)
    except ValueError:
        return None
    uses = joint_utilizations(joint_loads(estimate, k=1.0), strength)
    if not uses:
        return None
    return max(item.utilization for item in uses)


def _from_log(log: RunLog) -> tuple[float | None, float | None]:
    peak = None
    util = None
    for frame in log.frames:
        sample = frame.get("structure")
        if not sample or "error" in sample:
            continue
        sag = float(sample["worst_sag"])
        peak = sag if peak is None else max(peak, sag)
        if "worst_utilization" in sample:
            value = float(sample["worst_utilization"])
            util = value if util is None else max(util, value)
    return peak, util


def _completion(world, blueprint: dict) -> float:
    target = [
        tuple(item["index"])
        for item in blueprint["cells"]
        if item["voxel_type"] in ("structural", "seed")
    ]
    if not target:
        return 0.0
    present = sum(1 for cell in target if int(world.occupancy[cell]) == VOXEL)
    return present / len(target)


def _events_2d(log: RunLog) -> list[dict]:
    events: list[dict] = []
    # A cell can fail more than once. Each defect stays open until the
    # replacement voxel lands, in order.
    pending: dict[tuple, list[int]] = {}
    previous = None
    for frame in log.frames:
        grid = frame["occupancy"]
        tick = int(frame["tick"])
        if previous is None:
            previous = grid
            continue
        for row, (old_row, new_row) in enumerate(zip(previous, grid)):
            for col, (old, new) in enumerate(zip(old_row, new_row)):
                cell = [row, col]
                key = (row, col)
                if old == EMPTY and new in (VOXEL, DEFECT):
                    events.append(
                        {
                            "t": tick,
                            "type": "placement",
                            "cell": cell,
                            "robot": "r0",
                            "voxel_type": "structural",
                        }
                    )
                if new == DEFECT and old != DEFECT:
                    pending.setdefault(key, []).append(tick)
                    events.append(
                        {
                            "t": tick,
                            "type": "fault",
                            "fault": "defect",
                            "cell": cell,
                            "robot": "r0",
                            "reason": "re-place",
                        }
                    )
                if old == DEFECT and new == EMPTY:
                    events.append(
                        {
                            "t": tick,
                            "type": "removal",
                            "cell": cell,
                            "robot": "r0",
                            "voxel_type": "structural",
                            "reason": "defect",
                        }
                    )
                if pending.get(key) and old == EMPTY and new == VOXEL:
                    # A later attempt can itself be a defect. The good
                    # voxel closes every open fault on this cell, which
                    # is what finish_cell does for the 3D channels.
                    while pending[key]:
                        opened = pending[key].pop(0)
                        events.append(
                            {
                                "t": tick,
                                "type": "recovery",
                                "action": "re-place",
                                "fault": "defect",
                                "cell": cell,
                                "robot": "r0",
                                "reason": f"ticks {tick - opened}",
                            }
                        )
                    del pending[key]
        previous = grid
    return events


def _fault_numbers(events: list[dict]) -> dict[str, Any]:
    faults = [event for event in events if event["type"] == "fault" and not event.get("cascade")]
    cascades = [event for event in events if event["type"] == "fault" and event.get("cascade")]
    recoveries = [event for event in events if event["type"] == "recovery"]
    times = []
    for recovery in recoveries:
        reason = recovery.get("reason") or ""
        if reason.startswith("ticks "):
            times.append(float(reason.split()[1]))
    # Injector recoveries carry cause indexes into the injector list,
    # which is not the merged event list. Pair a recovery with the
    # latest earlier fault of the same channel and cell.
    if not times:
        for recovery in recoveries:
            candidates = [
                event
                for event in faults + cascades
                if event.get("fault") == recovery.get("fault")
                and event.get("cell") == recovery.get("cell")
                and float(event["t"]) <= float(recovery["t"])
            ]
            if candidates:
                times.append(float(recovery["t"]) - float(candidates[-1]["t"]))
    unrecovered = len(faults) + len(cascades) - len(recoveries)
    if unrecovered < 0:
        unrecovered = 0
    return {
        "faults": len(faults),
        "cascades": len(cascades),
        "recoveries": len(recoveries),
        "unrecovered": unrecovered,
        "recovery_time": "" if not times else statistics.median(times),
    }


def _gate_numbers(events: list[dict]) -> tuple[int, int]:
    accepts = sum(
        1
        for event in events
        if event["type"] == "gate" and event.get("verdict") == "accept"
    )
    refuses = sum(
        1
        for event in events
        if event["type"] == "gate" and event.get("verdict") == "refuse"
    )
    return accepts, refuses


def _row(
    scenario: dict,
    world,
    *,
    status: str,
    makespan: float,
    unit: str,
    events: list[dict],
    scaffold_count: int | None,
    peak_sag: float | None = None,
    utilization: float | None = None,
    extra: str = "",
) -> dict:
    blueprint = scenario["blueprint"]
    model = blueprint["model"]
    structural = sum(1 for item in blueprint["cells"] if item["voxel_type"] == "structural")
    target = sum(
        1
        for item in blueprint["cells"]
        if item["voxel_type"] in ("structural", "seed")
    )
    faults = _fault_numbers(events)
    accepts, refuses = _gate_numbers(events)
    note = _NOTE
    if extra:
        note = f"{note} {extra}"
    return {
        "id": scenario["id"],
        "family": scenario["family"],
        "status": status,
        "completion": _completion(world, blueprint),
        "makespan": makespan,
        "makespan_unit": unit,
        "peak_sag": peak_sag,
        "peak_sag_units": "" if peak_sag is None else _SAG_UNITS,
        "joint_utilization": utilization,
        "scaffold_count": scaffold_count,
        "faults": faults["faults"],
        "recoveries": faults["recoveries"],
        "cascades": faults["cascades"],
        "unrecovered": faults["unrecovered"],
        "recovery_time": faults["recovery_time"],
        "gate_accepts": accepts,
        "gate_refusals": refuses,
        "k": model["k"],
        "fitted": model["fitted"],
        "n_target": target,
        "n_structural": structural,
        "note": note,
        "_clock": unit if unit == "second" else "tick",
        "_events": events,
    }


def _csv(value: Any) -> str:
    if value is None or value is False and value != 0:
        return ""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, float):
        return format(value, ".6g")
    return str(value)


def _blank(value: Any) -> str:
    if value is None or value == "":
        return ""
    if isinstance(value, float):
        return format(value, ".6g")
    return str(value)


def main(argv: list[str] | None = None) -> None:
    args = list(argv or [])
    if not args:
        idents = list(SUITE)
    elif args == ["smoke"]:
        idents = list(SMOKE)
    else:
        unknown = [item for item in args if item not in SUITE]
        if unknown:
            raise SystemExit(f"unknown scenario: {', '.join(unknown)}")
        idents = args
    rows = run_ids(idents)
    text = markdown_table(rows)
    print(text)
    if idents == list(SUITE):
        write_table(rows, Path("benchmarks/results.csv"), Path("benchmarks/results.md"))
        print("wrote benchmarks/results.csv and benchmarks/results.md")
    else:
        write_table(rows, Path("benchmarks/results-partial.csv"))
        print("wrote benchmarks/results-partial.csv")
