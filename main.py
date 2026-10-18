# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Entry point / run configs.

Default run (Slice 2): full 6x4 wall with defective placements (p = 0.12,
fixed seed) and error correction — inspect, remove, replan, replace. Run
log goes to runs/latest.json for the replay viewer.

    python main.py            # slice 2: build with defects + repair
    python main.py yield      # slice 2 experiment: yield-vs-p chart
    python main.py slice1     # clean build, no defects, + ticks chart
    python main.py slice0     # the old ugly loop (no robot)
    python main.py bridge     # stability gate on, strain log -> runs/bridge.json
    python main.py cuboct     # optional cuboctahedral voxels -> runs/cuboct.json
    python main.py stability # gate cost/effect campaign -> docs/stability_cost.*
    python main.py stability smoke
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


def build_swarm_world(cols: int = 14, wall_left: int = 4) -> World:
    """Wider stage for multi-robot runs: park space on both flanks."""
    world = World(rows=WORLD_ROWS, cols=cols)
    world.set_wall_blueprint(width=WALL_WIDTH, height=WALL_HEIGHT, left=wall_left)
    return world


SWARM_STARTS = [(5, 3), (5, 10), (5, 2), (5, 11), (5, 1)]  # alternating flanks


def run_swarm_build(
    world: World,
    n_robots: int = 2,
    defect_p: float = 0.0,
    seed: int | None = None,
    correction: bool = True,
    max_ticks: int = 8000,
    verbose: bool = True,
    stability=None,
    structure=None,
):
    """Slice 3: N robots, shared blueprint, choreographed by reservations."""
    from sim.swarm import Swarm

    swarm = Swarm(
        world,
        DEPOT,
        starts=SWARM_STARTS[:n_robots],
        defect_p=defect_p,
        rng=random.Random(seed),
        inspect_enabled=correction,
        stability=stability,
    )
    log = RunLog(structure=structure)
    log.meta.update(
        depot=list(DEPOT), n_robots=n_robots, defect_p=defect_p,
        seed=seed, correction=correction,
    )
    log.log_tick(0, world, robots=swarm.robots)
    while not swarm.settled and swarm.tick_count < max_ticks:
        swarm.tick()
        log.log_tick(swarm.tick_count, world, robots=swarm.robots)
    log.meta.update(
        kickbacks=swarm.kickbacks,
        defects=swarm.defects_found,
        places=sum(1 for e in swarm.events if e[2] == "place"),
    )
    if verbose:
        print(render_ascii(world))
        print(
            f"\nslice 3 ({n_robots} robots, p={defect_p}, "
            f"correction={correction}): "
            f"yield={world.built_count}/{world.blueprint_count} "
            f"defects={swarm.defects_found} kickbacks={swarm.kickbacks} "
            f"ticks={swarm.tick_count}"
        )
    return log, swarm


def build_world3d(levels: int = 5, rows: int = 10, cols: int = 12) -> "World3D":
    """Slice 4a stage: a hollow 4x4x3 box (walls + roof) mid-field."""
    from sim.world3d import World3D

    world = World3D(levels, rows, cols)
    world.set_box_blueprint(width=4, depth=4, height=3, left=4, front=3, hollow=True)
    return world


DEPOT3D = (1, 4, 1)  # ground-level grip cell, left of the box
SWARM3D_STARTS = [(1, 2, 1), (1, 7, 1), (1, 2, 10), (1, 7, 10), (1, 4, 10)]


def run_swarm3d_build(
    world,
    n_robots: int = 2,
    defect_p: float = 0.0,
    seed: int | None = None,
    correction: bool = True,
    max_ticks: int = 20_000,
    verbose: bool = True,
    geometry_factory=None,
    depot=None,
    starts=None,
    stability=None,
    structure=None,
    voxel_shape=None,
):
    """Slice 4a/4b: N robots, 3D structure, same choreographer.

    ``voxel_shape`` is written to the log only when it is not the cubic
    default. Omit it and the log matches the historical v3 contract.
    """
    from sim.geometry3d import CubicLattice3D
    from sim.render import render_ascii3d
    from sim.swarm import Swarm

    swarm = Swarm(
        world,
        depot or DEPOT3D,
        starts=(starts or SWARM3D_STARTS)[:n_robots],
        defect_p=defect_p,
        rng=random.Random(seed),
        inspect_enabled=correction,
        geometry_factory=geometry_factory or CubicLattice3D,
        stability=stability,
    )
    log = RunLog(structure=structure)
    log.meta.update(
        depot=list(depot or DEPOT3D), n_robots=n_robots, defect_p=defect_p,
        seed=seed, correction=correction,
    )
    if voxel_shape is not None:
        from sim.cuboct import annotate_run_log

        annotate_run_log(log, voxel_shape)
    log.log_tick(0, world, robots=swarm.robots)
    while not swarm.settled and swarm.tick_count < max_ticks:
        swarm.tick()
        log.log_tick(swarm.tick_count, world, robots=swarm.robots)
    log.meta.update(
        kickbacks=swarm.kickbacks,
        defects=swarm.defects_found,
        places=sum(1 for e in swarm.events if e[2] == "place"),
    )
    if verbose:
        print(render_ascii3d(world))
        print(
            f"\nslice 4a ({n_robots} robots, 3D box, p={defect_p}, "
            f"correction={correction}): "
            f"yield={world.built_count}/{world.blueprint_count} "
            f"defects={swarm.defects_found} kickbacks={swarm.kickbacks} "
            f"ticks={swarm.tick_count}"
        )
    return log, swarm


def speedup3d_experiment(
    ns: tuple = (1, 2, 3, 4, 5), out: str = "runs/speedup_3d.png"
):
    """Milestone chart: build ticks and speedup vs robot count in 3D.
    The congestion knee is where marginal speedup collapses."""
    from sim.metrics import speedup_vs_n_chart

    ticks: dict[int, int] = {}
    prev = None
    for n in ns:
        world = build_world3d()
        _log, swarm = run_swarm3d_build(world, n_robots=n, verbose=False)
        if not world.complete:
            raise RuntimeError(f"N={n} did not complete: {world.built_count}")
        ticks[n] = swarm.tick_count
        marginal = f" marginal x{prev / swarm.tick_count:.2f}" if prev else ""
        prev = swarm.tick_count
        print(f"N={n}: {swarm.tick_count} ticks "
              f"(speedup x{ticks[list(ticks)[0]] / swarm.tick_count:.2f}, "
              f"kickbacks={swarm.kickbacks}){marginal}")
    chart = speedup_vs_n_chart(ticks, out)
    print(f"chart: {chart}")
    return ticks


def reach3d_experiment(
    radii: tuple = (1, 2, 3, 4), out: str = "runs/ticks_vs_reach.png"
):
    """Slice 4c chart: same blueprint, same coordination code, reach as
    config — build ticks vs reach radius (2 robots, 3D box)."""
    from sim.geometry3d import CubicLattice3D, MotionModel
    from sim.metrics import ticks_vs_reach_chart

    ticks: dict[int, int] = {}
    for radius in radii:
        world = build_world3d()
        factory = lambda w, _r=radius: CubicLattice3D(w, MotionModel(_r))
        _log, swarm = run_swarm3d_build(
            world, n_robots=2, verbose=False, geometry_factory=factory
        )
        if not world.complete:
            raise RuntimeError(f"reach={radius} did not complete")
        ticks[radius] = swarm.tick_count
        print(f"reach={radius}: {swarm.tick_count} ticks "
              f"(kickbacks={swarm.kickbacks})")
    chart = ticks_vs_reach_chart(ticks, out)
    print(f"chart: {chart}")
    return ticks


def yield3d_experiment(
    ps: tuple = (0.0, 0.04, 0.08, 0.12),
    ns: tuple = (1, 2, 3),
    seeds: tuple = (1, 2, 3),
    out: str = "runs/yield3d.png",
):
    """4b done-criterion: yield vs defect rate at swarm scale, against
    the analytic no-correction bound. Correction must hold ~100% at
    every N — repair-through-live-reservations at swarm scale."""
    from sim.metrics import yield3d_chart

    corrected: dict[int, dict[float, list[float]]] = {n: {} for n in ns}
    for n in ns:
        for p in ps:
            vals = []
            for seed in seeds:
                world = build_world3d()
                _log, swarm = run_swarm3d_build(
                    world, n_robots=n, defect_p=p, seed=seed, verbose=False
                )
                vals.append(world.built_count / world.blueprint_count)
            corrected[n][p] = vals
            print(f"N={n} p={p}: yield={sum(vals)/len(vals):.3f}")
    # Measured baseline (no correction), N=2: defects stand where they fall.
    baseline: dict[float, list[float]] = {}
    for p in ps:
        vals = []
        for seed in seeds:
            world = build_world3d()
            _log, _swarm = run_swarm3d_build(
                world, n_robots=2, defect_p=p, seed=seed,
                correction=False, verbose=False,
            )
            vals.append(world.built_count / world.blueprint_count)
        baseline[p] = vals
        print(f"baseline p={p}: yield={sum(vals)/len(vals):.3f}")
    chart = yield3d_chart(list(ps), corrected, baseline, out)
    print(f"chart: {chart}")


def knee_vs_size_experiment(
    ns: tuple = (1, 2, 3, 4, 5), out: str = "runs/knee_vs_size.png"
):
    """Congestion-knee scaling: speedup-vs-N for two structure sizes.
    The knee should move right as the structure (and its surface) grows."""
    from sim.metrics import knee_vs_size_chart
    from sim.world3d import World3D

    def small_world():
        return build_world3d()  # hollow 4x4x3 = 40 voxels

    def large_world():
        world = World3D(6, 12, 14)
        world.set_box_blueprint(
            width=6, depth=6, height=4, left=5, front=4, hollow=True
        )
        return world

    large_depot = (1, 5, 1)
    large_starts = [(1, 2, 1), (1, 9, 1), (1, 2, 12), (1, 9, 12), (1, 5, 12)]
    results: dict[str, dict[int, int]] = {}
    for label, factory, depot, starts in (
        ("40 voxels", small_world, None, None),
        ("96 voxels", large_world, large_depot, large_starts),
    ):
        ticks: dict[int, int] = {}
        for n in ns:
            world = factory()
            _log, swarm = run_swarm3d_build(
                world, n_robots=n, verbose=False, depot=depot, starts=starts
            )
            if not world.complete:
                raise RuntimeError(f"{label} N={n} incomplete")
            ticks[n] = swarm.tick_count
            print(f"{label} N={n}: {swarm.tick_count} ticks "
                  f"(x{ticks[ns[0]] / swarm.tick_count:.2f})")
        results[label] = ticks
    chart = knee_vs_size_chart(results, out)
    print(f"chart: {chart}")


# Two piers, deck at level 2 so the span is not fastened to the floor
# (every level-1 cell would be planted on the stock ground plane).
# Columns 3 and 9 are the piers; the midspan column is 6, three
# horizontal steps from either pier. Two rows so the deck reads as a
# deck. 4 pier voxels + 14 deck voxels.
BRIDGE_ROWS = (2, 3)
BRIDGE_PIER_COLS = (3, 9)
BRIDGE_DECK_COLS = range(3, 10)
BRIDGE_CANTILEVER = 3
BRIDGE_DEPOT = (1, 1, 1)
BRIDGE_STARTS = [(1, 5, 12)]


def build_bridge_world():
    """Small two-pier bridge. The deck is a span the sag model can show.

    Level 0 is the ground plane. Piers occupy level 1. The deck is level
    2, in the air, so only the pier tops are column-supported and the
    middle of the deck is the cantilever. ``max_cantilever`` has to be
    at least :data:`BRIDGE_CANTILEVER` or the stability gate refuses the
    span — which is the point of running this demo with the gate on.
    """
    from sim.world3d import World3D

    world = World3D(levels=4, rows=7, cols=14)
    for row in BRIDGE_ROWS:
        for col in BRIDGE_PIER_COLS:
            world.blueprint[1, row, col] = True
        for col in BRIDGE_DECK_COLS:
            world.blueprint[2, row, col] = True
    return world


def run_bridge(verbose: bool = True):
    """Build the bridge with the stability gate on and strain logging on.

    One robot, no defects. Stiffness, mass, and g stay at the model
    defaults (1), so displacements in the log are in units of ``mg/k``
    and have not been fit to a printed specimen. Writes ``runs/bridge.json``.
    """
    from sim.geometry3d import CubicLattice3D
    from sim.metrics import StructureLog

    world = build_bridge_world()
    policy = CubicLattice3D(world).stability_policy(BRIDGE_CANTILEVER)
    structure = StructureLog(policy, k=1.0, mass=1.0, g=1.0, pitch=1.0, every=1)
    log, swarm = run_swarm3d_build(
        world,
        n_robots=1,
        depot=BRIDGE_DEPOT,
        starts=BRIDGE_STARTS,
        stability=policy,
        structure=structure,
        verbose=False,
        max_ticks=8000,
    )
    if not world.complete:
        raise RuntimeError(
            f"bridge did not finish: {world.built_count}/{world.blueprint_count} "
            f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}"
        )
    saved = log.save("runs/bridge.json")
    samples = [f for f in log.frames if "structure" in f and "error" not in f["structure"]]
    last = samples[-1]["structure"] if samples else None
    if verbose:
        from sim.render import render_ascii3d

        print(render_ascii3d(world))
        print(
            f"bridge: placed={world.built_count}/{world.blueprint_count} "
            f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}"
        )
        print(f"bridge log: {saved}  ({len(samples)} structural samples)")
        if last is not None:
            print(
                f"final worst sag {last['worst_sag']:.4f} mg/k "
                f"at {last['worst_sag_cell']} "
                f"(model estimate, stiffness not fit)"
            )
        print("open replay_viewer.html and drop runs/bridge.json")
        print("  color: displacement or joint strain; geometry is as-built")
        print("  exaggerate slider (off): longest arrow as a fraction of the pitch")
    return log, swarm


def run_cuboct(verbose: bool = True):
    """Build the 2×2×2 cuboct demo. Cubic locomotion, cuboct solid.

    Writes ``runs/cuboct.json`` with ``meta.voxel_shape`` set so the
    replay viewer opens on cuboctahedra. The stability gate stays off.
    This is a geometry and joint-location run, not an ARMADAS stiffness
    measurement. The caller also stores the log at ``runs/latest.json``.
    """
    from sim.cuboct import (
        EXAMPLE_DEPOT,
        EXAMPLE_START,
        VOXEL_SHAPE_CUBOCTAHEDRAL,
        CuboctahedralLattice3D,
        example_world,
    )

    world = example_world()
    log, swarm = run_swarm3d_build(
        world,
        n_robots=1,
        depot=EXAMPLE_DEPOT,
        starts=[EXAMPLE_START],
        geometry_factory=CuboctahedralLattice3D,
        voxel_shape=VOXEL_SHAPE_CUBOCTAHEDRAL,
        verbose=False,
        max_ticks=8000,
    )
    if not world.complete:
        raise RuntimeError(
            f"cuboct demo did not finish: {world.built_count}/{world.blueprint_count} "
            f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}"
        )
    saved = log.save("runs/cuboct.json")
    if verbose:
        from sim.render import render_ascii3d

        print(render_ascii3d(world))
        print(
            f"cuboct: placed={world.built_count}/{world.blueprint_count} "
            f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}"
        )
        print(
            "voxel_shape=cuboctahedral (same 6-neighbor grid as cubic; "
            "joint sites are the square-face corners)"
        )
        print(f"cuboct log: {saved}")
        print("open replay_viewer.html and drop runs/cuboct.json")
        print("  voxel menu: cuboctahedron (log default) or cubic")
    return log, swarm


def main() -> None:
    import sys

    mode = sys.argv[1] if len(sys.argv) > 1 else "slice2"
    world = build_world()
    if mode == "swarm3d":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 2
        world = build_world3d()
        log, _swarm = run_swarm3d_build(world, n_robots=n)
    elif mode == "speedup3d":
        speedup3d_experiment()
        return
    elif mode == "reach3d":
        reach3d_experiment()
        return
    elif mode == "yield3d":
        yield3d_experiment()
        return
    elif mode == "knee3d":
        knee_vs_size_experiment()
        return
    elif mode == "bridge":
        log, _swarm = run_bridge()
    elif mode == "cuboct":
        log, _swarm = run_cuboct()
    elif mode == "stability":
        from experiments.stability_cost import main as stability_main

        stability_main(sys.argv[2:])
        return
    elif mode == "swarm":
        n = int(sys.argv[2]) if len(sys.argv) > 2 else 2
        world = build_swarm_world()
        log, _swarm = run_swarm_build(world, n_robots=n)
    elif mode == "slice0":
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
