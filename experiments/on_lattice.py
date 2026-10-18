# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Crane stance versus on-lattice inchworm, with the robot's weight on.

The default geometry still stands in an empty cell and reaches out.
This campaign opts into :class:`sim.inchworm.OnLatticeGeometry` and
compares it with that default on three blueprints (a cantilever, a
bridge, a tower). Stiffness is the unfitted default ``k = mass = g = 1``.
Sag is in units of ``mass·g/k``, not millimeters.

What is measured, and what is only configuration
------------------------------------------------
The gait is the cubic-face abstraction in ``sim/inchworm.py``: stride
1, span 2, place reach 1, top and side faces, flat and both corner
classes. Those numbers are not a joint-trajectory fit. Side faces are
on because a level change with stride 1 needs the side face as a place
to stop. Inverted faces stay off.

Robot-to-voxel mass ratios 1, 3, and 10 are a sweep. They are not a
published hardware ratio. The only published pair this repository will
quote is SOLL-E (Park et al., IROS 2023, NTRS 20230011353): cargo
robot 4.8 kg, crane robot 4.4 kg, voxel mass approximately 380 g, so
about 12.6 and about 11.6. Those two are not extra rows. BILL-E's
prototype is 520 g (Jenett and Cheung, AIAA 2017-1876, Table I) and
that paper does not give a voxel mass, so no BILL-E ratio is stated.

The crane has no foot cells. Its weight is not applied, and a sag cap
on the policy does not move its stance check. ``peak_sag_robots`` and
``peak_sag_voxels`` are the same number on a crane row. Saying so is
the comparison, not a missing load path.

Gates, for one robot, three entry seeds (n = 3):

* ``off`` — no ``StabilityPolicy``. On-lattice placement is still
  face-adjacent, because that is the gait, not the gate.
* ``cantilever`` — ``max_cantilever`` equals the finished structure's
  worst overhang. The finished blueprint passes the precheck. A
  one-sided prefix that overhangs farther is rejected.
* ``sag`` — ``max_cantilever`` is ``None`` (connected to the base is
  enough) and ``max_sag`` is the worst sag of the finished structure
  with one carried robot standing on a legal stance. The finished
  structure passes that cap within 1e-6. A prefix that sags more,
  including the robot, does not. The crane ignores the cap.

Inspection is off and there are no defects. Makespan is walk, pick,
and place. The equal foot split is the model in
:func:`sim.stability.split_foot_loads`.

Run it:

    python main.py onlattice

Smoke (one cantilever, both modes, gate off):

    python main.py onlattice smoke
"""

from __future__ import annotations

import csv
import platform
import random
import time
from pathlib import Path

from sim.choreographer import _BUILDABILITY_CACHE
from sim.geometry3d import CubicLattice3D
from sim.inchworm import InchwormConfig, OnLatticeGeometry, entry_stances
from sim.metrics import RunLog, StructureLog
from sim.planner import SearchBudgetExceeded, plan_build_order
from sim.stability import (
    StabilityPolicy,
    assess_stability,
    estimate_sag,
    occupied_voxels,
    split_foot_loads,
)
from sim.swarm import Swarm, SwarmStuck
from sim.world import VOXEL
from sim.world3d import World3D

# Uncalibrated spring. Do not read a sag number as a length in meters.
K = 1.0
MASS = 1.0
G = 1.0
MAX_NODES = 4_000
MAX_TICKS = 2_000
N_SEEDS = 3
RATIOS = (1.0, 3.0, 10.0)
# SOLL-E, Park et al. Table I, stated only so a reader can place the
# sweep. Not simulated as its own condition. Voxel mass is "approximately
# 380 g" in that paper, so the ratios are approximate.
SOLL_E_CARGO_KG = 4.8
SOLL_E_CRANE_KG = 4.4
SOLL_E_VOXEL_G = 380.0
SOLL_E_CARGO_RATIO = SOLL_E_CARGO_KG * 1000.0 / SOLL_E_VOXEL_G
SOLL_E_CRANE_RATIO = SOLL_E_CRANE_KG * 1000.0 / SOLL_E_VOXEL_G

# Both modes share these voxels. The blueprint is only the cells the
# robot still has to place. Level-1 voxels on the ground plane are
# planted; the span or the tower is at level 2 and above so it sags.
SEED = ((1, 1, 1), (1, 1, 2))
CRANE_DEPOT = (1, 1, 0)
CRANE_STARTS = ((1, 1, 0), (1, 0, 1), (1, 2, 1))

GAIT = InchwormConfig(
    stride=1,
    span=2.0,
    place_reach=1,
    allow_top=True,
    allow_sides=True,
    allow_inverted=False,
    allow_flat=True,
    allow_convex=True,
    allow_concave=True,
)

FRAME = StabilityPolicy(
    up_axis=0, up_sign=1, ground_plane=0, max_cantilever=None
)

CSV_FIELDS = (
    "blueprint",
    "mode",
    "ratio",
    "gate",
    "seed_index",
    "n_voxels",
    "depot",
    "status",
    "reordered",
    "place_reordered",
    "plan_order",
    "place_order",
    "completed",
    "makespan",
    "idle_robot_ticks",
    "kickbacks",
    "peak_sag_robots",
    "peak_sag_voxels",
    "final_sag_voxels",
    "sag_errors",
    "sag_samples",
    "plan_s",
    "max_cantilever",
    "max_sag",
    "robot_mass",
    "carried_mass",
    "k",
    "mass",
    "g",
    "python",
    "note",
)


def build_cantilever() -> World3D:
    """Two planted seed voxels and a 3-voxel arm at level 2.

    The tip's overhang is 2. The arm is not on the ground plane, so
    the three level-2 voxels are the free chain (voxels-only tip sag
    6 at k = mass = g = 1).
    """
    world = World3D(5, 4, 8)
    for cell in SEED:
        world.place_voxel(cell)
    for col in (2, 3, 4):
        world.blueprint[2, 1, col] = True
    return world


def build_bridge() -> World3D:
    """Two planted piers, a seed beside the left pier, deck at level 2.

    Columns 3 and 6 are the piers. The deck is the four level-2 cells
    from pier to pier. Finished overhang is 1. A deck grown from the
    left pier alone reaches overhang 2 at column 5, before the far
    cap exists.
    """
    world = World3D(5, 4, 10)
    for cell in SEED:
        world.place_voxel(cell)
    world.place_voxel((1, 1, 3))
    world.place_voxel((1, 1, 6))
    for col in range(3, 7):
        world.blueprint[2, 1, col] = True
    return world


def build_tower() -> World3D:
    """A 3-voxel tower on the first seed voxel, with a buddy beside it.

    The buddy is the second foot. The cell directly above a foot is
    the exterior of that grip, so the robot cannot stand on the column
    top and place the next voxel there. Side faces are how it reaches.
    Finished overhang is 0.
    """
    world = World3D(6, 4, 6)
    for cell in SEED:
        world.place_voxel(cell)
    for level in (2, 3, 4):
        world.blueprint[level, 1, 1] = True
    return world


BUILDERS = {
    "cantilever": build_cantilever,
    "bridge": build_bridge,
    "tower": build_tower,
}


def gait_config() -> InchwormConfig:
    return GAIT


def _blueprint_cells(world: World3D) -> list[tuple]:
    return [tuple(int(x) for x in node) for node in zip(*world.blueprint.nonzero())]


def buried_tops(world: World3D) -> set[tuple]:
    """Seed tops a blueprint cell directly above will cover.

    A depot foot on that face is legal at the start and illegal after
    the placement, and the return-to-depot check then fails every
    approach. Those stances are not used as entries.
    """
    wanted = set(_blueprint_cells(world))
    buried = set()
    for cell in SEED:
        above = (cell[0] + 1, cell[1], cell[2])
        if above in wanted:
            buried.add(cell)
    return buried


def choose_entries(world: World3D, n: int = N_SEEDS) -> list:
    """``n`` entry stances on the seed, stable for a given occupancy."""
    geom = OnLatticeGeometry(world, GAIT)
    buried = buried_tops(world)
    found = []
    for stance in entry_stances(geom, SEED):
        if any(
            cell in buried and normal == (1, 0, 0)
            for cell, normal in (stance[1], stance[2])
        ):
            continue
        found.append(stance)
    if len(found) < n:
        raise RuntimeError(
            f"need {n} entry stances on the seed, found {len(found)}"
        )
    if len(found) == n:
        return found
    indexes = [0, len(found) // 2, len(found) - 1]
    return [found[i] for i in indexes[:n]]


def _factory(policy):
    config = GAIT

    def factory(world, config=config, policy=policy):
        return OnLatticeGeometry(world, config, policy)

    return factory


def _fmt_cell(cell) -> str:
    return "(" + ",".join(str(int(c)) for c in cell) + ")"


def _fmt_depot(node) -> str:
    faces = node[1:] if isinstance(node, tuple) and node and node[0] == "iw" else None
    if not faces:
        return _fmt_cell(node)
    parts = []
    for cell, normal in faces:
        parts.append(
            _fmt_cell(cell) + "/" + "(" + ",".join(str(int(c)) for c in normal) + ")"
        )
    return "iw:" + "+".join(parts)


def _fmt_order(tasks) -> str:
    if not tasks:
        return ""
    return ";".join(_fmt_cell(task.cell) for task in tasks)


def finished_overhang(world: World3D) -> int:
    done = world.copy()
    for cell in _blueprint_cells(world):
        if done.occupancy[cell] != VOXEL:
            done.place_voxel(cell)
    report = assess_stability(occupied_voxels(done), FRAME)
    if not report.stable and report.floating:
        raise RuntimeError(f"finished structure floats: {report.reason}")
    return int(report.worst_overhang)


def finished_world(world: World3D) -> World3D:
    done = world.copy()
    for cell in _blueprint_cells(world):
        if done.occupancy[cell] != VOXEL:
            done.place_voxel(cell)
    return done


def sag_cap(world: World3D, robot_mass: float, carried_mass: float) -> float:
    """Worst finished sag with one carried robot on a legal stance.

    The cap is what ``robot_stance_ok`` will see (equal split, carried
    mass on). Every legal stance on the finished structure is at or
    under it. A heavier prefix is not.
    """
    done = finished_world(world)
    geom = OnLatticeGeometry(done, GAIT)
    cells = occupied_voxels(done)
    worst = estimate_sag(cells, FRAME, k=K, mass=MASS, g=G).worst_sag
    for stance in geom.stances():
        loads = split_foot_loads(
            geom.foot_cells(stance),
            robot_mass=robot_mass,
            carried_mass=carried_mass,
            g=G,
        )
        sag = estimate_sag(
            cells, FRAME, k=K, mass=MASS, g=G, point_loads=loads
        ).worst_sag
        if sag > worst:
            worst = sag
    return float(worst)


def _policy(world: World3D, gate: str, robot_mass: float, carried_mass: float):
    """Planner policy for this gate. ``None`` leaves the gate off."""
    if gate == "off":
        return None
    if gate == "cantilever":
        return StabilityPolicy(
            up_axis=0,
            up_sign=1,
            ground_plane=0,
            max_cantilever=finished_overhang(world),
            max_sag=None,
            robot_mass=robot_mass,
            carried_mass=carried_mass,
        )
    if gate == "sag":
        return StabilityPolicy(
            up_axis=0,
            up_sign=1,
            ground_plane=0,
            max_cantilever=None,
            max_sag=sag_cap(world, robot_mass, carried_mass),
            robot_mass=robot_mass,
            carried_mass=carried_mass,
        )
    raise ValueError(f"unknown gate {gate}")


def _peaks(log: RunLog) -> tuple[float | None, float | None, int, int]:
    peak_r = None
    peak_v = None
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
        peak_r = sag if peak_r is None else max(peak_r, sag)
        if "worst_sag_voxels" in sample:
            alone = float(sample["worst_sag_voxels"])
        else:
            alone = sag
        peak_v = alone if peak_v is None else max(peak_v, alone)
    return peak_r, peak_v, errors, samples


def _drive(world, depot, starts, factory, policy, robot_mass, carried_mass, seed: int):
    _BUILDABILITY_CACHE.clear()
    swarm = Swarm(
        world,
        depot,
        starts=list(starts),
        geometry_factory=factory,
        stability=policy,
        defect_p=0.0,
        rng=random.Random(seed),
        inspect_enabled=False,
    )
    log = RunLog(
        structure=StructureLog(
            FRAME,
            k=K,
            mass=MASS,
            g=G,
            pitch=1.0,
            every=1,
            robot_mass=robot_mass,
            carried_mass=carried_mass,
        )
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
    places = [event[3] for event in swarm.events if event[2] == "place"]
    return log, swarm, idle, stuck, places


def _blank_numbers() -> dict:
    return {
        "reordered": "",
        "place_reordered": "",
        "plan_order": "",
        "place_order": "",
        "completed": "",
        "makespan": "",
        "idle_robot_ticks": "",
        "kickbacks": "",
        "peak_sag_robots": "",
        "peak_sag_voxels": "",
        "final_sag_voxels": "",
        "sag_errors": "",
        "sag_samples": "",
    }


def run_trial(
    blueprint: str,
    mode: str,
    ratio: float | None,
    gate: str,
    seed_index: int,
    *,
    off_orders: dict | None = None,
) -> dict:
    """One cell. ``off_orders`` collects gate-off plans so later gates can say if they moved."""
    world = BUILDERS[blueprint]()
    n_voxels = int(world.blueprint_count)
    if mode == "crane":
        robot_mass = 0.0
        carried_mass = 0.0
        depot = CRANE_DEPOT
        starts = [CRANE_STARTS[seed_index]]
        factory = CubicLattice3D
        note = "crane has no foot cells; robot weight is not applied"
    elif mode == "on-lattice":
        robot_mass = float(ratio)
        carried_mass = MASS
        entries = choose_entries(world)
        depot = entries[seed_index]
        starts = [depot]
        note = (
            "inchworm stride=1 span=2 place_reach=1 sides=on inverted=off; "
            "mass split equally across feet"
        )
    else:
        raise ValueError(mode)

    # The sag cap depends on robot_mass. Crane passes 0, so the cap is
    # the voxels-only finished sag, and the stance check does not use it.
    policy = _policy(world, gate, robot_mass, carried_mass)
    if mode == "on-lattice":
        factory = _factory(policy)

    scratch = world.copy()
    t0 = time.perf_counter()
    try:
        plan = plan_build_order(
            scratch, depot, factory, max_nodes=MAX_NODES, stability=policy
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
    order = "" if not isinstance(plan, list) else _fmt_order(plan)
    key = (blueprint, mode, "" if ratio is None else ratio, seed_index)
    if gate == "off" and off_orders is not None and status == "feasible":
        off_orders[key] = order
    reordered = ""
    if gate != "off" and off_orders is not None and status == "feasible":
        reference = off_orders.get(key)
        if reference is not None:
            reordered = "yes" if order != reference else "no"

    limit = ""
    cap = ""
    if policy is not None:
        limit = "none" if policy.max_cantilever is None else str(policy.max_cantilever)
        cap = "" if policy.max_sag is None else float(policy.max_sag)

    row = {
        "blueprint": blueprint,
        "mode": mode,
        "ratio": "" if ratio is None else ratio,
        "gate": gate,
        "seed_index": seed_index,
        "n_voxels": n_voxels,
        "depot": _fmt_depot(depot),
        "status": status,
        "plan_s": plan_s,
        "max_cantilever": limit,
        "max_sag": cap,
        "robot_mass": robot_mass,
        "carried_mass": carried_mass,
        "k": K,
        "mass": MASS,
        "g": G,
        "python": platform.python_version(),
        "note": note,
    }
    row.update(_blank_numbers())
    row["reordered"] = reordered
    row["plan_order"] = order
    if status != "feasible":
        return row

    built_world = BUILDERS[blueprint]()
    log, swarm, idle, stuck, places = _drive(
        built_world,
        depot,
        starts,
        factory,
        policy,
        robot_mass,
        carried_mass,
        seed_index,
    )
    peak_r, peak_v, errors, samples = _peaks(log)
    try:
        final = float(
            estimate_sag(occupied_voxels(built_world), FRAME, k=K, mass=MASS, g=G).worst_sag
        )
    except ValueError:
        final = None
    completed = bool(built_world.complete and swarm.settled and not stuck)
    row["completed"] = "1" if completed else "0"
    row["makespan"] = swarm.tick_count if completed else ""
    row["idle_robot_ticks"] = idle
    row["kickbacks"] = swarm.kickbacks
    row["peak_sag_robots"] = "" if peak_r is None else peak_r
    row["peak_sag_voxels"] = "" if peak_v is None else peak_v
    row["final_sag_voxels"] = "" if final is None else final
    row["sag_errors"] = errors
    row["sag_samples"] = samples
    row["place_order"] = ";".join(_fmt_cell(cell) for cell in places)
    if stuck:
        row["note"] = note + f"; stuck: {stuck}"
    return row


def iter_jobs(smoke: bool = False):
    """Gate-off before the other gates, so the reorder flag has a reference."""
    shapes = ("cantilever",) if smoke else ("cantilever", "bridge", "tower")
    gates = ("off",) if smoke else ("off", "cantilever", "sag")
    seeds = (0,) if smoke else tuple(range(N_SEEDS))
    for blueprint in shapes:
        for seed_index in seeds:
            yield blueprint, "crane", None, "off", seed_index
            if not smoke:
                for gate in gates:
                    if gate == "off":
                        continue
                    yield blueprint, "crane", None, gate, seed_index
            yield blueprint, "on-lattice", 1.0, "off", seed_index
            if smoke:
                continue
            for ratio in RATIOS:
                for gate in gates:
                    if ratio == 1.0 and gate == "off":
                        continue
                    yield blueprint, "on-lattice", ratio, gate, seed_index


def run_jobs(jobs, *, progress: bool = True) -> list[dict]:
    rows = []
    off_orders: dict = {}
    job_list = list(jobs)
    for index, (blueprint, mode, ratio, gate, seed_index) in enumerate(job_list, start=1):
        row = run_trial(
            blueprint, mode, ratio, gate, seed_index, off_orders=off_orders
        )
        rows.append(row)
        if progress:
            sag = row["peak_sag_robots"]
            sag_s = "" if sag == "" else f" peak={float(sag):.3f}"
            ticks = row["makespan"]
            tick_s = "" if ticks == "" else f" ticks={ticks}"
            ratio_s = "crane" if ratio is None else f"r={ratio:g}"
            print(
                f"[{index}/{len(job_list)}] {blueprint} {mode} {ratio_s} "
                f"gate={gate} seed={seed_index} {row['status']} "
                f"plan={row['plan_s']:.3f}s{tick_s}{sag_s}",
                flush=True,
            )
    _mark_place_reorder(rows)
    return rows


def _mark_place_reorder(rows: list[dict]) -> None:
    """Compare the order cells were actually placed with the gate-off build.

    The search order and the claim order are not the same thing. Nearest
    claim inside the lookahead window can place a supported cell before
    the tip the search listed first. Both are recorded.
    """
    reference = {
        (row["blueprint"], row["mode"], row["ratio"], row["seed_index"]): row["place_order"]
        for row in rows
        if row["gate"] == "off" and row["place_order"]
    }
    for row in rows:
        if row["gate"] == "off":
            row["place_reordered"] = "no" if row["place_order"] else ""
            continue
        if row["status"] != "feasible" or not row["place_order"]:
            continue
        key = (row["blueprint"], row["mode"], row["ratio"], row["seed_index"])
        prior = reference.get(key)
        if prior is not None:
            row["place_reordered"] = "yes" if row["place_order"] != prior else "no"


def _num(row, key):
    raw = row.get(key, "")
    if raw == "" or raw is None:
        return None
    return float(raw)


def _median(values: list[float]) -> float:
    ordered = sorted(values)
    mid = len(ordered) // 2
    if len(ordered) % 2:
        return ordered[mid]
    return 0.5 * (ordered[mid - 1] + ordered[mid])


def _spread(values: list[float], digits: int = 3) -> str:
    if not values:
        return "n=0"
    lo, hi = min(values), max(values)
    med = _median(values)
    return (
        f"n={len(values)} median {med:.{digits}f} "
        f"({lo:.{digits}f}–{hi:.{digits}f})"
    )


def summarize(rows: list[dict]) -> str:
    lines = [
        "On-lattice inchworm versus crane stance.",
        f"k = mass = g = 1. Sag is model units of mass*g/k, not a length.",
        f"Python {rows[0]['python'] if rows else platform.python_version()}.",
        "One robot. Inspection off. No defects. N is the number of entry seeds.",
        "Ratios 1, 3, 10 are a sweep, not hardware.",
        (
            "SOLL-E (Park et al., IROS 2023, NTRS 20230011353) Table I: "
            f"cargo {SOLL_E_CARGO_KG} kg, crane {SOLL_E_CRANE_KG} kg, "
            f"voxel approximately {SOLL_E_VOXEL_G:.0f} g, so about "
            f"{SOLL_E_CARGO_RATIO:.1f} and {SOLL_E_CRANE_RATIO:.1f}. "
            "Not a row in this file. BILL-E Table I is 520 g and gives "
            "no voxel mass; no BILL-E ratio is stated."
        ),
        "Crane rows apply no robot weight (no foot cells).",
        "Gait config: stride 1, span 2, place_reach 1, sides on, inverted off.",
        "",
    ]
    shapes = []
    for row in rows:
        if row["blueprint"] not in shapes:
            shapes.append(row["blueprint"])
    for shape in shapes:
        lines.append(shape)
        group = [row for row in rows if row["blueprint"] == shape]
        for mode in ("crane", "on-lattice"):
            for ratio in (None, 1.0, 3.0, 10.0) if mode == "on-lattice" else (None,):
                if mode == "crane" and ratio is not None:
                    continue
                for gate in ("off", "cantilever", "sag"):
                    cell = [
                        row for row in group
                        if row["mode"] == mode
                        and row["gate"] == gate
                        and (
                            (ratio is None and row["ratio"] == "")
                            or row["ratio"] == ratio
                            or (ratio is not None and row["ratio"] != "" and float(row["ratio"]) == ratio)
                        )
                    ]
                    if not cell:
                        continue
                    label = "crane" if mode == "crane" else f"on-lattice ratio {ratio:g}"
                    feasible = sum(1 for row in cell if row["status"] == "feasible")
                    infeasible = sum(1 for row in cell if row["status"] == "infeasible")
                    undecided = sum(1 for row in cell if row["status"] == "undecided")
                    done = [row for row in cell if str(row["completed"]) in ("1", "True")]
                    peaks_r = [v for v in (_num(row, "peak_sag_robots") for row in done) if v is not None]
                    peaks_v = [v for v in (_num(row, "peak_sag_voxels") for row in done) if v is not None]
                    spans = [v for v in (_num(row, "makespan") for row in done) if v is not None]
                    errors = [v for v in (_num(row, "sag_errors") for row in done) if v is not None]
                    reordered = [row["reordered"] for row in cell if row["reordered"] in ("yes", "no")]
                    n_yes = sum(1 for flag in reordered if flag == "yes")
                    placed = [
                        row["place_reordered"]
                        for row in cell
                        if row["place_reordered"] in ("yes", "no") and row["gate"] != "off"
                    ]
                    n_place = sum(1 for flag in placed if flag == "yes")
                    lines.append(
                        f"  {label} gate={gate}: "
                        f"feasible {feasible}/{len(cell)}, "
                        f"infeasible {infeasible}, undecided {undecided}, "
                        f"completed {len(done)}/{len(cell)}"
                    )
                    if peaks_r:
                        lines.append(f"    peak sag with robots {_spread(peaks_r)}")
                        lines.append(f"    peak sag voxels alone {_spread(peaks_v)}")
                    if spans:
                        lines.append(f"    makespan ticks {_spread(spans, 0)}")
                    if errors and any(v > 0 for v in errors):
                        lines.append(f"    sag errors {_spread(errors, 0)}")
                    if reordered:
                        lines.append(
                            f"    search order changed vs gate-off: {n_yes}/{len(reordered)}"
                        )
                    if placed:
                        lines.append(
                            f"    place order changed vs gate-off: {n_place}/{len(placed)}"
                        )
                    orders = sorted({row["plan_order"] for row in cell if row["plan_order"]})
                    if len(orders) == 1:
                        lines.append(f"    plan order {orders[0]}")
                    elif orders:
                        lines.append(f"    plan orders differ across seeds ({len(orders)})")
                    placed_orders = sorted(
                        {row["place_order"] for row in cell if row["place_order"]}
                    )
                    if len(placed_orders) == 1 and placed_orders != orders:
                        lines.append(f"    place order {placed_orders[0]}")
                    elif len(placed_orders) == 1 and orders:
                        lines.append("    place order matches the plan")
                    elif len(placed_orders) > 1:
                        lines.append(
                            f"    place orders differ across seeds ({len(placed_orders)})"
                        )
        lines.append("")
    return "\n".join(lines).rstrip() + "\n"


def _style(ax) -> None:
    ax.spines[["top", "right"]].set_visible(False)
    ax.grid(axis="y", color="#e5e7eb", linewidth=0.6, zorder=0)
    ax.tick_params(labelsize=8)


def _completed(row) -> bool:
    return str(row.get("completed")) in ("1", "True")


def write_charts(rows: list[dict], out_dir: Path) -> list[Path]:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle

    out_dir.mkdir(parents=True, exist_ok=True)
    paths = []
    shapes = []
    for row in rows:
        if row["blueprint"] not in shapes:
            shapes.append(row["blueprint"])
    if not shapes:
        return paths

    # Peak sag at gate off: crane, then the three ratios. Points are seeds.
    fig, axes = plt.subplots(1, len(shapes), figsize=(3.6 * len(shapes), 3.4), squeeze=False)
    for index, shape in enumerate(shapes):
        ax = axes[0][index]
        _style(ax)
        series = [("crane", None, "crane")]
        series += [(f"{ratio:g}", ratio, f"ratio {ratio:g}") for ratio in RATIOS]
        ymax = 0.0
        for xpos, (_label, ratio, legend) in enumerate(series):
            cell = [
                row for row in rows
                if row["blueprint"] == shape
                and row["gate"] == "off"
                and _completed(row)
                and (
                    (ratio is None and row["mode"] == "crane")
                    or (
                        row["mode"] == "on-lattice"
                        and row["ratio"] != ""
                        and float(row["ratio"]) == ratio
                    )
                )
            ]
            for key, color, marker, name in (
                ("peak_sag_voxels", "#94a3b8", "o", "voxels alone"),
                ("peak_sag_robots", "#1d4ed8", "s", "with robots"),
            ):
                values = [(row, _num(row, key)) for row in cell]
                values = [(row, v) for row, v in values if v is not None]
                if not values:
                    continue
                ymax = max(ymax, max(v for _row, v in values))
                med = _median([v for _row, v in values])
                ax.plot(
                    [xpos - 0.22, xpos + 0.22], [med, med],
                    color="#111827", linewidth=1.4, zorder=4, solid_capstyle="butt",
                )
                for row, value in values:
                    jitter = (int(row["seed_index"]) - 1) * 0.06
                    shift = -0.08 if key == "peak_sag_voxels" else 0.08
                    ax.scatter(
                        xpos + shift + jitter, value, s=22, color=color,
                        marker=marker, zorder=3,
                        label=name if index == 0 and xpos == 0 else None,
                    )
        ax.set_xticks(range(len(series)), [label for label, _r, _n in series])
        ax.set_title(shape, loc="left", fontsize=10)
        if index == 0:
            ax.set_ylabel("peak sag during the build (mass·g/k)", fontsize=8)
        ax.set_ylim(0, ymax * 1.18 if ymax > 0 else 1)
    handles, labels = axes[0][0].get_legend_handles_labels()
    if handles:
        fig.legend(handles, labels, loc="upper right", frameon=False, fontsize=8, ncol=2)
    fig.suptitle(
        "Gate off. Crane weight is not in the model. k = 1, unfitted.",
        x=0.01, ha="left", fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.88))
    peak_path = out_dir / "on_lattice_peak_sag.png"
    fig.savefig(peak_path, dpi=150)
    plt.close(fig)
    paths.append(peak_path)

    # Makespan of completed builds. Infeasible gates are labeled.
    fig, axes = plt.subplots(1, len(shapes), figsize=(3.6 * len(shapes), 3.4), squeeze=False)
    gates = ("off", "cantilever", "sag")
    for index, shape in enumerate(shapes):
        ax = axes[0][index]
        _style(ax)
        tracks = [("crane", None, "#64748b")]
        colors = {1.0: "#93c5fd", 3.0: "#2563eb", 10.0: "#1e3a8a"}
        for ratio in RATIOS:
            tracks.append((f"r{ratio:g}", ratio, colors[ratio]))
        ymax = 0.0
        for xpos, gate in enumerate(gates):
            for track_i, (label, ratio, color) in enumerate(tracks):
                cell = [
                    row for row in rows
                    if row["blueprint"] == shape and row["gate"] == gate
                    and (
                        (ratio is None and row["mode"] == "crane")
                        or (
                            ratio is not None
                            and row["mode"] == "on-lattice"
                            and row["ratio"] != ""
                            and float(row["ratio"]) == ratio
                        )
                    )
                ]
                done = [row for row in cell if _completed(row)]
                values = [v for v in (_num(row, "makespan") for row in done) if v is not None]
                slot = xpos + (track_i - 1.5) * 0.16
                if not values:
                    if cell and all(row["status"] == "infeasible" for row in cell):
                        ax.text(
                            slot, 0.08, "rej", transform=ax.get_xaxis_transform(),
                            ha="center", va="bottom", fontsize=6, color="#9ca3af", rotation=90,
                        )
                    continue
                ymax = max(ymax, max(values))
                med = _median(values)
                ax.plot(
                    [slot - 0.05, slot + 0.05], [med, med],
                    color="#111827", linewidth=1.2, zorder=4,
                )
                for row, value in zip(done, values):
                    jitter = (int(row["seed_index"]) - 1) * 0.02
                    ax.scatter(
                        slot + jitter, value, s=16, color=color, zorder=3,
                        label=label if index == 0 and xpos == 0 else None,
                    )
        ax.set_xticks(range(len(gates)), gates)
        ax.set_title(shape, loc="left", fontsize=10)
        if index == 0:
            ax.set_ylabel("makespan (ticks)", fontsize=8)
        ax.set_ylim(0, ymax * 1.2 if ymax > 0 else 1)
    handles, labels = axes[0][0].get_legend_handles_labels()
    # One label per track; the loop above may repeat. Dict keeps order.
    unique = dict(zip(labels, handles))
    if unique:
        fig.legend(
            unique.values(), unique.keys(),
            loc="upper right", frameon=False, fontsize=8, ncol=4,
        )
    fig.suptitle(
        "Makespan. Rejected orders are marked, not drawn as zero. Inspect off.",
        x=0.01, ha="left", fontsize=11,
    )
    fig.tight_layout(rect=(0, 0, 1, 0.86))
    span_path = out_dir / "on_lattice_makespan.png"
    fig.savefig(span_path, dpi=150)
    plt.close(fig)
    paths.append(span_path)

    # Feasibility: fraction of seeds whose search returned an order.
    fig, ax = plt.subplots(figsize=(8.2, 0.55 * len(shapes) + 1.6))
    columns = [("crane", None, gate) for gate in gates]
    for ratio in RATIOS:
        for gate in gates:
            columns.append(("on-lattice", ratio, gate))
    for y, shape in enumerate(shapes):
        for x, (mode, ratio, gate) in enumerate(columns):
            cell = [
                row for row in rows
                if row["blueprint"] == shape and row["mode"] == mode and row["gate"] == gate
                and (
                    (ratio is None and row["ratio"] == "")
                    or (ratio is not None and row["ratio"] != "" and float(row["ratio"]) == ratio)
                )
            ]
            if not cell:
                continue
            feasible = sum(1 for row in cell if row["status"] == "feasible")
            share = feasible / len(cell)
            color = (1.0 - share, 0.45 + 0.4 * share, 0.35)
            ax.add_patch(Rectangle((x - 0.5, y - 0.4), 1, 0.8, color=color, zorder=1))
            ax.text(x, y, f"{feasible}/{len(cell)}", ha="center", va="center", fontsize=7, zorder=2)
    ax.set_xlim(-0.5, len(columns) - 0.5)
    ax.set_ylim(-0.5, len(shapes) - 0.5)
    ax.set_yticks(range(len(shapes)), shapes)
    xlabels = []
    for mode, ratio, gate in columns:
        if mode == "crane":
            xlabels.append(f"crane\n{gate}")
        else:
            xlabels.append(f"r{ratio:g}\n{gate}")
    ax.set_xticks(range(len(columns)), xlabels, fontsize=7)
    ax.tick_params(length=0)
    for spine in ax.spines.values():
        spine.set_visible(False)
    ax.set_title(
        "Seeds with a proved build order. Color is the feasible share. k = 1.",
        loc="left", fontsize=11,
    )
    fig.tight_layout()
    feas_path = out_dir / "on_lattice_feasibility.png"
    fig.savefig(feas_path, dpi=150)
    plt.close(fig)
    paths.append(feas_path)
    return paths


def write_csv(rows: list[dict], path: Path) -> Path:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=CSV_FIELDS, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)
    return path


def write_outputs(rows: list[dict], out_dir: str | Path) -> dict:
    out = Path(out_dir)
    csv_path = write_csv(rows, out / "on_lattice.csv")
    summary = summarize(rows)
    summary_path = out / "on_lattice_summary.txt"
    summary_path.write_text(summary, encoding="utf-8")
    charts = write_charts(rows, out)
    print(summary, flush=True)
    print(f"csv: {csv_path}", flush=True)
    print(f"summary: {summary_path}", flush=True)
    for chart in charts:
        print(f"chart: {chart}", flush=True)
    return {"csv": csv_path, "summary": summary_path, "charts": charts, "rows": rows}


def run_smoke(out_dir: str | Path) -> dict:
    """Cantilever, gate off, crane and on-lattice ratio 1, one seed."""
    return write_outputs(run_jobs(iter_jobs(smoke=True)), out_dir)


def run_campaign(out_dir: str | Path = "docs") -> dict:
    return write_outputs(run_jobs(iter_jobs(smoke=False)), out_dir)


def build_demo_log():
    """A short cantilever the viewer can show with feet on the arm.

    Ratio 3 is inside the sweep, not a hardware claim. The log carries
    robot point loads so displacement colors include them.
    """
    world = build_cantilever()
    depot = choose_entries(world)[0]
    factory = _factory(None)
    log, swarm, _idle, stuck, _places = _drive(
        world, depot, [depot], factory, None, robot_mass=3.0, carried_mass=1.0, seed=0
    )
    if stuck or not world.complete:
        raise RuntimeError(f"on-lattice demo did not finish: {stuck}")
    # The walk that places the tip often stops with one foot still on
    # the seed. Append a legal both-feet-down pose on two adjacent
    # level-2 tops so a still shows the grippers on the arm, not a
    # diagonal reach. The stance comes from the gait, not from a
    # hand-placed coordinate.
    geom = OnLatticeGeometry(world, GAIT)
    tops = [
        stance
        for stance in geom.stances()
        if stance[1][1] == (1, 0, 0)
        and stance[2][1] == (1, 0, 0)
        and stance[1][0][0] == 2
        and stance[2][0][0] == 2
        and abs(stance[1][0][2] - stance[2][0][2]) == 1
    ]
    if not tops:
        raise RuntimeError("finished arm has no adjacent top-face stance")
    robot = swarm.robots[0]
    robot.pos = tops[0]
    robot.carrying = False
    robot.state = "IDLE"
    log.log_tick(swarm.tick_count + 1, world, robots=swarm.robots)
    log.meta["locomotion"] = "inchworm"
    log.meta["robot_mass_ratio"] = 3.0
    return log


def main(argv: list[str] | None = None) -> None:
    import sys

    args = list(sys.argv[1:] if argv is None else argv)
    if args and args[0] == "smoke":
        out = args[1] if len(args) > 1 else "runs/on_lattice_smoke"
        run_smoke(out)
        return
    out = args[0] if args else "docs"
    run_campaign(out)


if __name__ == "__main__":
    main()
