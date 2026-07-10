# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Entry point / run configs.

Default run (Slice 2): full 6x4 wall with defective placements (p = 0.12,
fixed seed) and error correction — inspect, remove, replan, replace. Run
log goes to runs/latest.json for the replay viewer.

    python main.py            # slice 2: build with defects + repair
    python main.py yield      # slice 2 experiment: yield-vs-p chart
    python main.py slice1     # clean build, no defects, + ticks chart
    python main.py slice0     # the old ugly loop (no robot)
"""

from __future__ import annotations

import random

from sim.metrics import RunLog, ticks_per_voxel_chart, yield_vs_p_chart
from sim.planner import plan_build_order, validate_plan
from sim.render import render_ascii
from sim.robot import IDLE, Robot
from sim.world import World

WORLD_ROWS = 7  # 6 rows of air + 1 ground row
WORLD_COLS = 10
WALL_WIDTH = 6
WALL_HEIGHT = 4
WALL_LEFT = 2


def build_world() -> World:
    """The hardcoded Slice 0 scenario: a 6x4 wall on the ground."""
    world = World(rows=WORLD_ROWS, cols=WORLD_COLS)
    world.set_wall_blueprint(width=WALL_WIDTH, height=WALL_HEIGHT, left=WALL_LEFT)
    return world


def run(world: World, verbose: bool = True) -> RunLog:
    """Fill one blueprint cell per tick until the blueprint is complete."""
    log = RunLog()
    log.log_tick(0, world)  # initial state, first replay frame
    if verbose:
        print("tick 0 (empty world):")
        print(render_ascii(world))
    for tick, cell in enumerate(world.blueprint_cells(), start=1):
        world.place_voxel(cell)
        rec = log.log_tick(tick, world)
        if verbose:
            print(f"\ntick {rec.tick}  progress {rec.progress:.0%}:")
            print(render_ascii(world))
    return log


DEPOT = (5, 0)  # grip cell at ground level, left of the wall


def run_robot(
    world: World,
    tasks: list,
    depot: tuple[int, int] = DEPOT,
    max_ticks: int = 2000,
) -> tuple[RunLog, Robot]:
    """Execute a task queue with one robot, logging every tick."""
    robot = Robot(pos=depot, depot=depot, tasks=tasks)
    log = RunLog()
    log.meta["depot"] = list(depot)
    log.log_tick(0, world, robot)
    for tick in range(1, max_ticks + 1):
        robot.tick(world)
        log.log_tick(tick, world, robot)
        if robot.state == IDLE and not robot.tasks:
            break
    return log, robot


def run_slice1(world: World, verbose: bool = True) -> tuple[RunLog, Robot]:
    """Slice 1: plan the full wall, validate, then build it."""
    plan = plan_build_order(world, DEPOT)
    if plan is None:
        raise RuntimeError("blueprint is unbuildable under current rules")
    ok, why = validate_plan(world, plan, DEPOT)
    if not ok:
        raise RuntimeError(f"planner emitted an invalid sequence: {why}")
    log, robot = run_robot(world, plan)
    log.meta["plan"] = [[list(t.cell), list(t.approach)] for t in plan]
    if verbose:
        print(render_ascii(world))
        placed = world.built_count
        print(
            f"\nfull wall: placed={placed}/{len(plan)} ticks={log.ticks} "
            f"moves={robot.moves} idle={robot.idle_ticks} "
            f"ticks/voxel={log.ticks / max(placed, 1):.1f}"
        )
    return log, robot


def run_slice2(
    world: World,
    defect_p: float,
    seed: int | None = None,
    correction: bool = True,
    max_ticks: int = 4000,
    verbose: bool = True,
) -> tuple[RunLog, Robot]:
    """Slice 2: plan/execute/replan with defective placements.

    The robot inspects each placement (1 tick); on a defect it removes the
    part (1 tick), halts, and this executor replans from the current world —
    replanning costs 0 ticks (the planner is offboard software, per the
    ARMADAS split between robot firmware and planning stack). With
    ``correction=False`` the robot places blind: the baseline for the chart.
    """
    robot = Robot(
        pos=DEPOT,
        depot=DEPOT,
        defect_p=defect_p,
        rng=random.Random(seed),
        inspect_enabled=correction,
    )
    plan = plan_build_order(world, DEPOT)
    ok, why = validate_plan(world, plan, DEPOT)
    if not ok:
        raise RuntimeError(f"invalid plan: {why}")
    robot.load_plan(plan)

    log = RunLog()
    log.meta.update(depot=list(DEPOT), defect_p=defect_p, seed=seed,
                    correction=correction)
    log.log_tick(0, world, robot)
    replans = 0
    tick = 0
    for tick in range(1, max_ticks + 1):
        robot.tick(world)
        log.log_tick(tick, world, robot)
        if robot.needs_replan:
            robot.load_plan(plan_build_order(world, DEPOT))
            replans += 1
        elif robot.state == IDLE and not robot.tasks:
            break
    log.meta.update(replans=replans, defects=robot.defects_found)
    if verbose:
        print(render_ascii(world))
        print(
            f"\nslice 2 (p={defect_p}, correction={correction}): "
            f"yield={world.built_count}/{world.blueprint_count} "
            f"defects={robot.defects_found} replans={replans} "
            f"ticks={tick} moves={robot.moves}"
        )
    return log, robot


def yield_experiment(
    n_seeds: int = 10, max_ticks: int = 4000, verbose: bool = True
) -> str:
    """Sweep p from 0 to 0.15, n_seeds runs per point, correction on & off.

    Yield = good voxels / blueprint size at end of run. Produces the Slice 2
    chart (runs/yield_vs_p.png) and prints the p = 0.08 spec check.
    """
    ps = [round(i * 0.01, 2) for i in range(16)]  # 0.00 .. 0.15
    corrected: dict[float, list[float]] = {p: [] for p in ps}
    baseline: dict[float, list[float]] = {p: [] for p in ps}
    for p in ps:
        for s in range(n_seeds):
            seed = 1000 * s + int(p * 100)
            for correction, bucket in ((True, corrected), (False, baseline)):
                world = build_world()
                run_slice2(world, p, seed=seed, correction=correction,
                           max_ticks=max_ticks, verbose=False)
                bucket[p].append(world.built_count / world.blueprint_count)
        if verbose:
            c = sum(corrected[p]) / n_seeds
            b = sum(baseline[p]) / n_seeds
            print(f"p={p:.2f}  corrected={c:.3f}  baseline={b:.3f}")
    chart = yield_vs_p_chart(ps, corrected, baseline, "runs/yield_vs_p.png")
    at_spec = sum(corrected[0.08]) / n_seeds
    print(f"\nspec check: corrected yield at p=0.08 -> {at_spec:.1%} "
          f"({'PASS' if at_spec >= 0.99 else 'FAIL'} vs >=99%)")
    return str(chart)


def main() -> None:
    import sys

    mode = sys.argv[1] if len(sys.argv) > 1 else "slice2"
    world = build_world()
    if mode == "slice0":
        log = run(world)
        print(f"\ndone: complete={world.complete}  {log.summary()}")
    elif mode == "slice1":
        log, _robot = run_slice1(world)
        chart = ticks_per_voxel_chart(log, "runs/ticks_per_voxel.png")
        print(f"chart:   {chart}")
    elif mode == "yield":
        chart = yield_experiment()
        print(f"chart:   {chart}")
        return
    else:  # slice 2 demo: fixed seed chosen so the replay shows repairs
        log, _robot = run_slice2(world, defect_p=0.12, seed=7)
    saved = log.save("runs/latest.json")
    print(f"run log: {saved}  (open replay_viewer.html and drop it in,")
    print(f"         or: python -m sim.render {saved})")


if __name__ == "__main__":
    main()
