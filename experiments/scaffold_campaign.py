# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Scaffold count and measured makespan on bridges of span 4–8.

One plan per (span, gait). The search minimizes scaffold count, then
``sequence_cost`` (a single-robot depot-round-trip score, not a proved
lower bound on swarm ticks). It does not simulate traffic. Each plan is
then built by a swarm at three robot/voxel ratios and three start
seeds, and the table records the tick count that swarm actually took.

Ratios are robots per voxel, approximately 1, 1/3, and 1/10::

    n_robots = max(1, round(n_voxels / divisor))  for divisor in {1, 3, 10}

Caps are the finished structure's own overhang and its ``worst_sag``
at ``k = mass = g = 1``. That stiffness is not a fit. Sag is in units
of ``mass·g/k``. The inchworm is :class:`~sim.inchworm.OnLatticeGeometry`
with the gait in :mod:`experiments.on_lattice`. It starts on a planted
level-1 pad beside the near pier. That pad is not scaffold and it does
not sit under the deck. The crane walks the unmodified bridge and
starts on open level-1 ground.

The ratio here is a robot count, ``n_robots / n_voxels``. It is not the
robot-mass / voxel-mass ratio in the on-lattice campaign.

    python main.py scaffold
"""

from __future__ import annotations

import csv
import random
import sys
import time
from pathlib import Path

from experiments.on_lattice import GAIT
from experiments.stability_cost import build_bridge, start_cells
from sim.geometry3d import CubicLattice3D
from sim.inchworm import OnLatticeGeometry, make_stance
from sim.planner import SearchBudgetExceeded
from sim.scaffold import CRANE, caps_for_finished, geometry_factory, plan_with_scaffold
from sim.swarm import Swarm, SwarmStuck
from sim.world import EMPTY, SCAFFOLD

ROOT = Path(__file__).resolve().parents[1]
DOCS = ROOT / "docs"
SPANS = (4, 5, 6, 7, 8)
DIVISORS = (1, 3, 10)
SEEDS = (0, 1, 2)
GAITS = ("crane", "inchworm")
MAX_NODES = 40_000
MAX_TICKS = 4_000

COLUMNS = (
    "span",
    "gait",
    "n_voxels",
    "divisor",
    "robot_per_voxel",
    "n_robots",
    "seed",
    "scaffold_count",
    "scaffold_cells",
    "sequence_cost",
    "makespan",
    "kickbacks",
    "completed",
    "status",
    "max_cantilever",
    "max_sag",
    "finished_sag",
    "k",
    "mass",
    "g",
    "plan_seconds",
    "swarm_seconds",
)


def n_robots_for(n_voxels: int, divisor: int) -> int:
    """Robots for a robot/voxel ratio of about ``1/divisor``."""
    return max(1, round(n_voxels / divisor))


def plant_entry_pad(world) -> tuple:
    """Plant the level-1 pad the inchworm enters on. Returns those cells.

    The pad is every level-1 cell in a column left of the near pier,
    plus the cells in the pier's column that are not the pier and not
    under a blueprint cell. Nothing is planted in a column under the
    deck, and nothing is planted on a blueprint cell. The voxels stay
    for the whole build. They are not scaffold.
    """
    piers = [
        (int(level), int(row), int(col))
        for level, row, col in zip(*world.blueprint.nonzero())
        if int(level) == 1
    ]
    if not piers:
        raise RuntimeError("bridge has no level-1 pier")
    near = min(cell[2] for cell in piers)
    planted = []
    for row in range(world.rows):
        for col in range(near):
            cell = (1, row, col)
            if world.occupancy[cell] != EMPTY or world.blueprint[cell]:
                continue
            world.place_voxel(cell)
            planted.append(cell)
        beside = (1, row, near)
        if world.occupancy[beside] != EMPTY or world.blueprint[beside]:
            continue
        if any(world.blueprint[level, row, near] for level in range(world.levels)):
            continue
        world.place_voxel(beside)
        planted.append(beside)
    if not planted:
        raise RuntimeError("entry pad is empty")
    return tuple(planted)


def _top_face(cell) -> tuple:
    return (cell, (1, 0, 0))


def disjoint_top_stances(geometry, cells) -> list:
    """Both-feet-on-top stances that do not share a face.

    Pairing is consecutive rows inside one column. Face centers of
    neighboring tops are one pitch apart, inside the campaign span of
    2. The depot is not included; callers that need a depot take one
    stance out of this list first.
    """
    by_col: dict[int, list] = {}
    for cell in cells:
        by_col.setdefault(int(cell[2]), []).append(cell)
    stances = []
    for col in sorted(by_col):
        column = sorted(by_col[col], key=lambda cell: int(cell[1]))
        for left, right in zip(column[0::2], column[1::2]):
            stance = make_stance(_top_face(left), _top_face(right))
            if geometry.is_footing(stance):
                stances.append(stance)
    return stances


def inchworm_factory(policy):
    """Geometry factory for the published on-lattice gait and this cap."""

    def factory(world, policy=policy):
        return OnLatticeGeometry(world, GAIT, policy)

    return factory


def stance_starts(pool: list, n: int, seed: int) -> list:
    """``n`` distinct stances from ``pool``. ``Random.randrange``."""
    if n > len(pool):
        raise RuntimeError(f"need {n} entry stances, only {len(pool)}")
    rng = random.Random(seed)
    chosen: list = []
    while len(chosen) < n:
        stance = pool[rng.randrange(len(pool))]
        if stance not in chosen:
            chosen.append(stance)
    return chosen


def _cells(scaffold) -> str:
    return ";".join(",".join(str(v) for v in cell) for cell in scaffold)


def plan_span(span: int, gait: str) -> dict:
    """One scaffold plan. The crane world is not modified.

    The inchworm world gains the entry pad before the caps are measured,
    so the cap is the structure that is still there after removal.
    """
    world, depot = build_bridge(span)
    pad: tuple = ()
    start_pool: list = []
    if gait == "inchworm":
        pad = plant_entry_pad(world)
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
        factory = inchworm_factory(policy)
        stances = disjoint_top_stances(factory(world), pad)
        if not stances:
            raise RuntimeError(f"span {span} entry pad has no stance")
        depot = stances[0]
        start_pool = stances[1:]
    elif gait == "crane":
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
        factory = geometry_factory(CRANE)
    else:
        raise ValueError(f"unknown gait {gait}")
    t0 = time.perf_counter()
    status = "ok"
    plan = None
    try:
        plan = plan_with_scaffold(
            world, depot, factory, policy, sag, max_nodes=MAX_NODES
        )
    except SearchBudgetExceeded as exc:
        status = "undecided"
        detail = str(exc)
    else:
        detail = ""
        if plan is None:
            status = "infeasible"
    seconds = time.perf_counter() - t0
    return {
        "world": world,
        "depot": depot,
        "factory": factory,
        "policy": policy,
        "sag": sag,
        "plan": plan,
        "status": status,
        "detail": detail,
        "plan_seconds": seconds,
        "n_voxels": int(world.blueprint_count),
        "gait": gait,
        "pad": pad,
        "start_pool": start_pool,
    }


def drive(case: dict, divisor: int, seed: int) -> dict:
    """Build ``case`` with one start draw. Returns a CSV row."""
    n_voxels = case["n_voxels"]
    n_robots = n_robots_for(n_voxels, divisor)
    ratio = n_robots / n_voxels
    policy = case["policy"]
    sag = case["sag"]
    plan = case["plan"]
    row = {
        "span": "",
        "gait": "",
        "n_voxels": n_voxels,
        "divisor": divisor,
        "robot_per_voxel": f"{ratio:.4f}",
        "n_robots": n_robots,
        "seed": seed,
        "scaffold_count": "" if plan is None else plan.scaffold_count,
        "scaffold_cells": "" if plan is None else _cells(plan.scaffold),
        "sequence_cost": "" if plan is None else plan.sequence_cost,
        "makespan": "",
        "kickbacks": "",
        "completed": False,
        "status": case["status"],
        "max_cantilever": policy.max_cantilever,
        "max_sag": format(float(sag.max_sag), ".12g"),
        "finished_sag": format(float(sag.max_sag), ".12g"),
        "k": sag.k,
        "mass": sag.mass,
        "g": sag.g,
        "plan_seconds": f"{case['plan_seconds']:.3f}",
        "swarm_seconds": "",
    }
    if plan is None:
        return row
    world = case["world"].copy()
    gait_name = case["gait"]
    try:
        if gait_name == "inchworm":
            starts = stance_starts(case["start_pool"], n_robots, seed)
        else:
            starts = start_cells(world, case["depot"], n_robots, seed)
    except RuntimeError as exc:
        row["status"] = "no_starts"
        row["swarm_seconds"] = str(exc)
        return row
    swarm = Swarm(
        world,
        case["depot"],
        starts,
        defect_p=0.0,
        inspect_enabled=True,
        geometry_factory=case["factory"],
        stability=policy,
        sag=sag,
        operations=plan.operations(),
        scaffold_cells=plan.scaffold,
    )
    stuck = ""
    t0 = time.perf_counter()
    try:
        while not swarm.done and swarm.tick_count < MAX_TICKS:
            swarm.tick()
    except SwarmStuck as exc:
        stuck = str(exc)
    row["swarm_seconds"] = f"{time.perf_counter() - t0:.3f}"
    row["kickbacks"] = swarm.kickbacks
    left = any(world.occupancy[cell] == SCAFFOLD for cell in plan.scaffold)
    finished = bool(swarm.done and world.complete and not left and not stuck)
    row["completed"] = finished
    if finished:
        row["status"] = "ok"
        row["makespan"] = swarm.tick_count
    elif stuck:
        row["status"] = "stuck"
        row["makespan"] = swarm.tick_count
    else:
        row["status"] = "incomplete"
        row["makespan"] = swarm.tick_count
    return row


def _summary(rows: list[dict]) -> str:
    lines = [
        "Scaffold campaign. Caps are the finished structure's overhang and",
        "worst_sag at k = mass = g = 1 (units of mass·g/k; stiffness not fit).",
        "The inchworm is OnLatticeGeometry with the on-lattice campaign gait.",
        "It enters on a planted level-1 pad that is not under the deck.",
        "The crane walks the unmodified bridge. sequence_cost is a",
        "single-robot travel score, not swarm ticks. makespan is the",
        "measured swarm tick count. The table is seed 0; the CSV has",
        "seeds 0, 1, and 2.",
        "robot_per_voxel is n_robots / n_voxels, with",
        "n_robots = max(1, round(n_voxels / divisor)). It is not the",
        "robot-mass / voxel-mass ratio in the on-lattice campaign.",
        "",
    ]
    header = (
        f"{'span':>4} {'gait':<9} {'div':>3} {'n':>3} {'scf':>3} "
        f"{'cost':>5} {'ticks':>6} {'kick':>5} {'status':<12} {'sag cap':>8}"
    )
    lines.append(header)
    for row in rows:
        if int(row["seed"]) != 0:
            continue
        lines.append(
            f"{row['span']:>4} {row['gait']:<9} {row['divisor']:>3} {row['n_robots']:>3} "
            f"{str(row['scaffold_count']):>3} {str(row['sequence_cost']):>5} "
            f"{str(row['makespan']):>6} {str(row['kickbacks']):>5} "
            f"{row['status']:<12} {row['max_sag']:>8}"
        )
    lines.append("")
    ok = sum(1 for row in rows if row["status"] == "ok")
    lines.append(f"rows {len(rows)}  ok {ok}")
    failed = [row for row in rows if row["status"] != "ok"]
    for row in failed:
        lines.append(
            f"not ok: span {row['span']} {row['gait']} div {row['divisor']} "
            f"seed {row['seed']} n={row['n_robots']} {row['status']} "
            f"ticks {row['makespan']} kick {row['kickbacks']}"
        )
    by_gait: dict[str, set] = {}
    for row in rows:
        by_gait.setdefault(row["gait"], set()).add(
            (row["span"], row["scaffold_count"], row["scaffold_cells"])
        )
    for gait, seen in sorted(by_gait.items()):
        lines.append(f"{gait}:")
        for span, count, cells in sorted(seen, key=lambda item: item[0]):
            lines.append(f"  span {span}: scaffold {count} [{cells}]")
    return "\n".join(lines) + "\n"


def run(spans=SPANS, gaits=GAITS, divisors=DIVISORS, seeds=SEEDS) -> list[dict]:
    rows: list[dict] = []
    for span in spans:
        for gait in gaits:
            print(f"plan span {span} {gait}", flush=True)
            case = plan_span(span, gait)
            plan = case["plan"]
            print(
                f"  {case['status']} scaffold="
                f"{None if plan is None else plan.scaffold_count} "
                f"cells={None if plan is None else _cells(plan.scaffold)} "
                f"cost={None if plan is None else plan.sequence_cost} "
                f"cant={case['policy'].max_cantilever} sag={case['sag'].max_sag:.12g} "
                f"in {case['plan_seconds']:.2f}s",
                flush=True,
            )
            for divisor in divisors:
                for seed in seeds:
                    row = drive(case, divisor, seed)
                    row["span"] = span
                    row["gait"] = gait
                    rows.append(row)
                    print(
                        f"  div {divisor} seed {seed} n={row['n_robots']} "
                        f"{row['status']} ticks={row['makespan']} "
                        f"kick={row['kickbacks']} swarm={row['swarm_seconds']}s",
                        flush=True,
                    )
    return rows


def write(rows: list[dict], csv_path: Path, summary_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    summary_path.write_text(_summary(rows))


def main(argv: list[str] | None = None) -> None:
    argv = list(sys.argv[1:] if argv is None else argv)
    if argv and argv[0] == "smoke":
        rows = run(spans=(5,), gaits=("inchworm",), divisors=(10,), seeds=(0,))
    else:
        rows = run()
    csv_path = DOCS / "scaffold_campaign.csv"
    summary_path = DOCS / "scaffold_campaign_summary.txt"
    write(rows, csv_path, summary_path)
    print(_summary(rows), flush=True)
    print(f"wrote {csv_path}", flush=True)
    print(f"wrote {summary_path}", flush=True)


if __name__ == "__main__":
    main()
