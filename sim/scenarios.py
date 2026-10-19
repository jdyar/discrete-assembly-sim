# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Public scenarios as interchange documents.

Each document is built from a constructor that already exists: the
stability-campaign shapes, the on-lattice arm, bridge, and tower, the
scaffold campaign's spans, the 256-cell shelter stand-in, the cuboct
block, the 2D wall, and the two-voxel column the fault tests drive.
Nothing here is a second geometry.

``k = mass = g = 1`` on every document. That stiffness is not a fit.
Fault rates, when a scenario has them, are configuration.

    python -m sim.scenarios
"""

from __future__ import annotations

from pathlib import Path

from experiments.on_lattice import (
    GAIT,
    build_bridge as build_onlattice_bridge,
    build_cantilever as build_onlattice_cantilever,
    build_tower as build_onlattice_tower,
)
from experiments.scaffold_campaign import (
    disjoint_top_stances,
    inchworm_factory,
    plant_entry_pad,
    stance_starts,
)
from experiments.stability_cost import (
    build_bridge,
    build_cantilever,
    build_hollow_box,
    build_wall,
    start_cells,
)
from sim.armadas import (
    FASTEN_JOINT_S,
    FEED_LOAD_S,
    HANDOFF_S,
    PLACE_S,
    STEP_S,
    shelter_cells,
    shelter_seed,
)
from sim.cuboct import EXAMPLE_DEPOT, EXAMPLE_START, example_world
from sim.geometry3d import CubicLattice3D
from sim.scaffold import caps_for_finished
from sim.format import (
    FORMAT,
    FORMAT_VERSION,
    UNFITTED_NOTE,
    cell_robot,
    cubic_lattice,
    inchworm_robot,
    lattice_for_world,
    make_blueprint,
    make_robots,
    make_scenario,
    square_lattice,
)
from sim.inchworm import InchwormConfig
from sim.world3d import World3D

# The stability campaign's one-cell stepper. Reach, stride, and climb
# are configuration.
STEPPER = {"reach_radius": 1, "stride": 1, "climb": 1, "lattice_only": False}
# Ground-bound arm used by the scaffold and fault campaigns.
CRANE = {"reach_radius": 4, "stride": 1, "climb": 0, "lattice_only": False}
# Reach used by the fault-channel fixture so the upper cell is placeable.
FAULT_REACH = {"reach_radius": 2, "stride": 1, "climb": 1, "lattice_only": False}

NOTE = (
    "k = mass = g = 1. Displacement is mass*g/k and force is mass*g. "
    "Stiffness has not been fit to a printed specimen. "
    + UNFITTED_NOTE
)

SUITE = (
    "wall-2d",
    "wall-3d",
    "hollow-box",
    "hollow-roof-limit-0",
    "cuboct-block",
    "bridge-span-4",
    "bridge-span-5",
    "bridge-span-6",
    "bridge-span-7",
    "bridge-span-8",
    "cantilever-3",
    "cantilever-4",
    "cantilever-arm",
    "tower",
    "onlattice-bridge",
    "armadas-shelter",
    "scaffold-span-4-crane",
    "scaffold-span-4-inchworm",
    "scaffold-span-8-crane",
    "fault-wall",
    "fault-tower",
)


def gait_document(config: InchwormConfig | None = None) -> dict:
    """The published on-lattice gait, as data."""
    config = GAIT if config is None else config
    return {
        "stride": int(config.stride),
        "span": float(config.span),
        "place_reach": int(config.place_reach),
        "allow_top": bool(config.allow_top),
        "allow_sides": bool(config.allow_sides),
        "allow_inverted": bool(config.allow_inverted),
        "allow_flat": bool(config.allow_flat),
        "allow_convex": bool(config.allow_convex),
        "allow_concave": bool(config.allow_concave),
    }


def _role(name: str, description: str, **extra) -> dict:
    role = {"description": description, "mass": 0.0, "carried_mass": 0.0}
    role.update(extra)
    return {name: role}


def _stepper_robots(ident: str, world, depot, start, motion, description: str) -> dict:
    robot = cell_robot(
        ident="r0",
        role="builder",
        depot=depot,
        start=start,
        motion=motion,
        description=description,
    )
    return make_robots(
        ident=ident,
        robots=[robot],
        roles=_role("builder", description),
    )


def _swarm_task(**extra) -> dict:
    task = {
        "runner": "swarm",
        "gate": "finished",
        "scaffold": "none",
        "n_robots": 1,
        "max_ticks": 8000,
        "max_nodes": 40000,
        "rng_seed": 0,
    }
    task.update(extra)
    return task


def _from_world(
    ident: str,
    title: str,
    family: str,
    world,
    depot,
    *,
    lattice_kind: str = "cubic",
    motion=None,
    task=None,
    notes: str = NOTE,
    meta: dict | None = None,
    start=None,
    description: str = "One cell per step, reach 1, climb 1. Configuration, not a measured gait.",
) -> dict:
    motion = STEPPER if motion is None else motion
    if start is None:
        start = start_cells(world, depot, 1, 0)[0]
    blueprint = make_blueprint(
        ident=f"{ident}.blueprint",
        title=title,
        lattice=lattice_for_world(world, kind=lattice_kind),
        cells=_cells(world),
        meta=meta,
    )
    robots = _stepper_robots(
        f"{ident}.robots", world, depot, start, motion, description
    )
    return make_scenario(
        ident=ident,
        title=title,
        family=family,
        blueprint=blueprint,
        robots=robots,
        task=task if task is not None else _swarm_task(),
        notes=notes,
        meta=meta,
    )


def _cells(world):
    from sim.format import cells_from_world

    return cells_from_world(world)


def wall_2d(faults: bool = False) -> dict:
    """The 6×4 wall. ``faults`` turns on the Bernoulli bad bond, p = 0.12, seed 7."""
    from main import DEPOT, WALL_HEIGHT, WALL_LEFT, WALL_WIDTH, build_world

    world = build_world()
    ident = "fault-wall" if faults else "wall-2d"
    title = "6×4 wall with bad bonds" if faults else "6×4 wall"
    blueprint = make_blueprint(
        ident=f"{ident}.blueprint",
        title=title,
        lattice=square_lattice(world.rows, world.cols),
        cells=_cells(world),
        meta={"source": "main.build_world"},
    )
    robot = cell_robot(
        ident="r0",
        role="builder",
        depot=DEPOT,
        start=DEPOT,
        motion=STEPPER,
        description="Square-lattice surface robot. Reach is the 8-cell neighborhood.",
    )
    robots = make_robots(
        ident=f"{ident}.robots",
        robots=[robot],
        roles=_role(
            "builder",
            "Square-lattice surface robot. The 2D depot is a grip cell beside the wall.",
        ),
        defect_p=0.12 if faults else 0.0,
        inspect=True,
        rng_seed=7 if faults else 0,
    )
    return make_scenario(
        ident=ident,
        title=title,
        family="fault" if faults else "wall",
        blueprint=blueprint,
        robots=robots,
        task={
            "runner": "wall2d",
            "gate": "off",
            "scaffold": "none",
            "n_robots": 1,
            "max_ticks": 4000,
            "rng_seed": 7 if faults else 0,
        },
        notes=NOTE
        + (
            " defect_p 0.12 and rng_seed 7 are the slice-2 replay. "
            "The probability is configuration."
            if faults
            else " Gate off: this is the coordination wall, not a sag case."
        ),
        meta={
            "source": "main.build_world",
            "width": WALL_WIDTH,
            "height": WALL_HEIGHT,
            "left": WALL_LEFT,
        },
    )


def wall_3d() -> dict:
    world, depot = build_wall()
    return _from_world(
        "wall-3d",
        "Wall 4×1×3",
        "wall",
        world,
        depot,
        meta={"source": "experiments.stability_cost.build_wall"},
    )


def hollow_box() -> dict:
    world, depot = build_hollow_box()
    return _from_world(
        "hollow-box",
        "Hollow box 3×3×2",
        "hollow",
        world,
        depot,
        notes=NOTE + " Finished overhang is 1. The finished caps accept the roof.",
        meta={"source": "experiments.stability_cost.build_hollow_box"},
    )


def hollow_roof_limit_0() -> dict:
    world, depot = build_hollow_box()
    return _from_world(
        "hollow-roof-limit-0",
        "Hollow roof at cantilever limit 0",
        "hollow",
        world,
        depot,
        task={
            "runner": "precheck",
            "gate": "cantilever-0",
            "scaffold": "none",
            "n_robots": 1,
            "max_ticks": 0,
            "max_nodes": 200,
            "rng_seed": 0,
        },
        notes=NOTE
        + " The roof center has no column. Limit 0 is a proved refusal. No search.",
        meta={"source": "experiments.stability_cost.build_hollow_box"},
    )


def cuboct_block() -> dict:
    world = example_world()
    return _from_world(
        "cuboct-block",
        "Cuboctahedral 2×2×2 block",
        "cuboct",
        world,
        EXAMPLE_DEPOT,
        lattice_kind="cuboctahedral",
        start=EXAMPLE_START,
        task={
            "runner": "swarm",
            "gate": "off",
            "scaffold": "none",
            "n_robots": 1,
            "max_ticks": 8000,
            "max_nodes": 40000,
            "rng_seed": 0,
        },
        description="Cell robot on the cuboctahedral solid. The cell graph is the cubic 6-neighbor graph.",
        notes=NOTE
        + " The solid is a cuboctahedron on the cubic grid. It is not an ARMADAS stiffness.",
        meta={"source": "sim.cuboct.example_world"},
    )


def bridge_span(span: int) -> dict:
    world, depot = build_bridge(span)
    return _from_world(
        f"bridge-span-{span}",
        f"Bridge span {span}",
        "bridge",
        world,
        depot,
        task=_swarm_task(span=span),
        notes=NOTE
        + f" Span is the column gap between the piers. Deck at level 2. One robot, seed 0.",
        meta={"source": "experiments.stability_cost.build_bridge", "span": span},
    )


def cantilever_length(length: int) -> dict:
    world, depot = build_cantilever(length)
    return _from_world(
        f"cantilever-{length}",
        f"Cantilever length {length}",
        "cantilever",
        world,
        depot,
        task=_swarm_task(length=length),
        notes=NOTE
        + " The root voxel is part of the blueprint. The arm is at level 2.",
        meta={"source": "experiments.stability_cost.build_cantilever", "length": length},
    )


def cantilever_arm() -> dict:
    world = build_onlattice_cantilever()
    depot = (1, 1, 0)
    return _from_world(
        "cantilever-arm",
        "Cantilever arm, 3 free voxels",
        "cantilever",
        world,
        depot,
        start=(1, 0, 1),
        description="Cell robot. The two seed voxels are the on-lattice campaign's planted base.",
        notes=NOTE
        + " Three level-2 voxels. Voxels-only tip sag of that chain is 6 at k = 1. "
        "The robot's weight is not applied.",
        meta={"source": "experiments.on_lattice.build_cantilever"},
    )


def tower() -> dict:
    world = build_onlattice_tower()
    return _from_world(
        "tower",
        "Tower, 3 free voxels",
        "tower",
        world,
        (1, 1, 0),
        start=(1, 0, 1),
        description="Cell robot beside the planted seed. The tower is levels 2–4.",
        notes=NOTE
        + " Voxels-only compression of the 3-voxel tower is 6 at k = 1. "
        "The robot's weight is not applied.",
        meta={"source": "experiments.on_lattice.build_tower"},
    )


def onlattice_bridge() -> dict:
    world = build_onlattice_bridge()
    return _from_world(
        "onlattice-bridge",
        "On-lattice bridge geometry, cell robot",
        "bridge",
        world,
        (1, 1, 0),
        start=(1, 0, 1),
        description="Cell robot. The piers and the buddy seed are already placed.",
        notes=NOTE
        + " This is the on-lattice campaign's bridge, built by a robot that stands "
        "in an empty cell. The inchworm on this deck is the scaffold task, not this one.",
        meta={"source": "experiments.on_lattice.build_bridge"},
    )


def _armadas_roles() -> dict:
    # Step time is Park et al. 2023, "on the order of 20 seconds".
    # Load, handoff, place, and fasten are the same 20 s and were not fitted.
    return {
        "cargo": {
            "description": "Carries a voxel from a feed station to the cell.",
            "mass": 0.0,
            "carried_mass": 0.0,
            "step_s": STEP_S,
            "feed_load_s": FEED_LOAD_S,
            "time_source": "park-2023-order-of-20s",
        },
        "crane": {
            "description": "Takes the voxel off the cargo robot and places it.",
            "mass": 0.0,
            "carried_mass": 0.0,
            "step_s": STEP_S,
            "handoff_s": HANDOFF_S,
            "place_s": PLACE_S,
            "time_source": "config",
        },
        "fasten": {
            "description": "Closes each new face joint as its own task.",
            "mass": 0.0,
            "carried_mass": 0.0,
            "step_s": STEP_S,
            "fasten_joint_s": FASTEN_JOINT_S,
            "time_source": "config",
        },
    }


def armadas_shelter() -> dict:
    cells = []
    seed = set(shelter_seed())
    for cell in shelter_cells():
        kind = "seed" if cell in seed else "structural"
        cells.append((cell, kind))
    blueprint = make_blueprint(
        ident="armadas-shelter.blueprint",
        title="ARMADAS stand-in shelter",
        lattice=cubic_lattice(8, 8, 8),
        cells=cells,
        meta={
            "source": "sim.armadas.shelter_cells",
            "hardware_pitch_m": 0.3048,
            "n_voxels_published": 256,
            "note": (
                "256 cells, five-voxel seed, roof eave. Not the Fig. 5 "
                "coordinate list. Model pitch is 1. The published edge "
                "304.8 mm is not this pitch, because k = 1 is not a fit."
            ),
        },
    )
    robots = make_robots(
        ident="armadas-shelter.robots",
        robots=[],
        roles=_armadas_roles(),
        schedule="three-role",
        robots_per_role=1,
        n_feeds=1,
        defect_p=0.0,
        inspect=True,
        rng_seed=0,
    )
    return make_scenario(
        ident="armadas-shelter",
        title="ARMADAS stand-in shelter",
        family="shelter",
        blueprint=blueprint,
        robots=robots,
        task={
            "runner": "armadas",
            "gate": "finished",
            "scaffold": "none",
            "n_feeds": 1,
            "robots_per_role": 1,
            "rng_seed": 0,
        },
        notes=NOTE
        + " One feed and one robot per role. Step time is on the order of 20 s "
        "(Park et al. 2023), not a Gregg 2024 duration. Load, handoff, place, "
        "and fasten are that same 20 s and were not fitted.",
        meta={"source": "sim.armadas.shelter_cells"},
    )


def _scaffold_task(span: int) -> dict:
    return {
        "runner": "scaffold",
        "gate": "finished",
        "scaffold": "minimal",
        "n_robots": 1,
        "max_ticks": 4000,
        "max_nodes": 40000,
        "span": span,
        "rng_seed": 0,
    }


def scaffold_crane(span: int) -> dict:
    world, depot = build_bridge(span)
    return _from_world(
        f"scaffold-span-{span}-crane",
        f"Scaffold search, span {span}, crane",
        "scaffold",
        world,
        depot,
        motion=CRANE,
        task=_scaffold_task(span),
        description=(
            "Ground-bound arm, reach 4, climb 0. Walks the floor. "
            "Configuration from sim.scaffold.CRANE."
        ),
        notes=NOTE
        + " The search may insert support under the deck and then remove it. "
        "A crane on these spans has a plan with an empty support set.",
        meta={"source": "experiments.scaffold_campaign", "span": span, "gait": "crane"},
    )


def scaffold_inchworm(span: int = 4) -> dict:
    world, _depot = build_bridge(span)
    plant_entry_pad(world)
    policy, _sag = caps_for_finished(
        world, CubicLattice3D(world).stability_policy(0)
    )
    factory = inchworm_factory(policy)
    stances = disjoint_top_stances(factory(world), _pad_cells(world))
    if len(stances) < 2:
        raise RuntimeError(f"span {span} entry pad has fewer than 2 stances")
    depot = stances[0]
    start = stance_starts(stances[1:], 1, 0)[0]
    ident = f"scaffold-span-{span}-inchworm"
    blueprint = make_blueprint(
        ident=f"{ident}.blueprint",
        title=f"Scaffold search, span {span}, inchworm",
        lattice=lattice_for_world(world),
        cells=_cells(world),
        meta={
            "source": "experiments.scaffold_campaign",
            "span": span,
            "gait": "inchworm",
            "note": (
                "Seed cells are the planted entry pad. They are not scaffold "
                "and they do not sit under the deck."
            ),
        },
    )
    robot = inchworm_robot(
        ident="r0",
        role="inchworm",
        depot_stance=depot,
        start_stance=start,
        gait=gait_document(),
    )
    robots = make_robots(
        ident=f"{ident}.robots",
        robots=[robot],
        roles=_role(
            "inchworm",
            "On-lattice gait: stride 1, span 2, place reach 1, top and side faces. "
            "Configuration, not a BILL-E or SOLL-E joint trajectory.",
        ),
    )
    return make_scenario(
        ident=ident,
        title=f"Scaffold search, span {span}, inchworm",
        family="scaffold",
        blueprint=blueprint,
        robots=robots,
        task=_scaffold_task(span),
        notes=NOTE
        + " The inchworm cannot walk the floor to the far pier. The search "
        "adds the smallest support set under the blueprint, builds, and "
        "removes it inside the finished caps.",
        meta={"source": "experiments.scaffold_campaign", "span": span, "gait": "inchworm"},
    )


def _pad_cells(world) -> tuple:
    from sim.world import VOXEL

    cells = []
    solid = world.occupancy == VOXEL
    for raw in zip(*solid.nonzero()):
        cells.append(tuple(int(v) for v in raw))
    return tuple(cells)


def fault_tower() -> dict:
    """Two-voxel column. One scripted fastener failure, then re-place.

    The script is two ``fastener`` names because the ground course has
    no voxel neighbor and does not draw. Rates are zero; the script is
    the draw. ``rate_source`` is configuration.
    """
    world = World3D(4, 5, 6)
    world.blueprint[1, 2, 2] = True
    world.blueprint[2, 2, 2] = True
    depot = (1, 1, 1)
    blueprint = make_blueprint(
        ident="fault-tower.blueprint",
        title="Fault tower, scripted fastener",
        lattice=lattice_for_world(world),
        cells=_cells(world),
        meta={"source": "tests/test_faults.py tower fixture"},
    )
    robot = cell_robot(
        ident="r0",
        role="builder",
        depot=depot,
        start=depot,
        motion=FAULT_REACH,
        description="Reach 2 so the upper cell is placeable from level 1.",
    )
    robots = make_robots(
        ident="fault-tower.robots",
        robots=[robot],
        roles=_role(
            "builder",
            "Reach 2, climb 1. The fastener script is configuration, not a measured failure rate.",
        ),
        faults={
            "seed": 0,
            "rate_source": "configuration",
            "script": ["fastener", "fastener"],
            "rates": {
                "capture": 0.0,
                "drop": 0.0,
                "stall": 0.0,
                "robot_fail": 0.0,
                "fastener": 0.0,
                "feed_outage": 0.0,
            },
        },
    )
    return make_scenario(
        ident="fault-tower",
        title="Fault tower, scripted fastener",
        family="fault",
        blueprint=blueprint,
        robots=robots,
        task={
            "runner": "swarm",
            "gate": "finished",
            "scaffold": "none",
            "n_robots": 1,
            "max_ticks": 800,
            "max_nodes": 40000,
            "rng_seed": 0,
        },
        notes=NOTE
        + " One scripted fastener failure on the upper cell. Recovery is "
        "re-place through inspect, remove, and replace. The probability "
        "list is empty because the script is the draw. Rates are configuration.",
        meta={"source": "tests/test_faults.py"},
    )


def catalog() -> dict[str, dict]:
    """Every public scenario, keyed by id, in suite order."""
    documents = {
        "wall-2d": wall_2d(False),
        "wall-3d": wall_3d(),
        "hollow-box": hollow_box(),
        "hollow-roof-limit-0": hollow_roof_limit_0(),
        "cuboct-block": cuboct_block(),
        "cantilever-3": cantilever_length(3),
        "cantilever-4": cantilever_length(4),
        "cantilever-arm": cantilever_arm(),
        "tower": tower(),
        "onlattice-bridge": onlattice_bridge(),
        "armadas-shelter": armadas_shelter(),
        "scaffold-span-4-inchworm": scaffold_inchworm(4),
        "fault-wall": wall_2d(True),
        "fault-tower": fault_tower(),
    }
    for span in (4, 5, 6, 7, 8):
        documents[f"bridge-span-{span}"] = bridge_span(span)
    documents["scaffold-span-4-crane"] = scaffold_crane(4)
    documents["scaffold-span-8-crane"] = scaffold_crane(8)
    missing = [ident for ident in SUITE if ident not in documents]
    if missing:
        raise RuntimeError(f"suite ids without documents: {missing}")
    extra = [ident for ident in documents if ident not in SUITE]
    if extra:
        raise RuntimeError(f"documents missing from the suite: {extra}")
    return {ident: documents[ident] for ident in SUITE}


def write_catalog(root: Path | None = None) -> Path:
    """Write ``benchmarks/tasks/*.json`` and ``benchmarks/suite.json``."""
    from sim.format import canonical

    root = Path("benchmarks") if root is None else Path(root)
    tasks = root / "tasks"
    tasks.mkdir(parents=True, exist_ok=True)
    documents = catalog()
    for ident, document in documents.items():
        (tasks / f"{ident}.json").write_text(canonical(document), encoding="utf-8")
    suite = {
        "format": FORMAT,
        "format_version": FORMAT_VERSION,
        "description": (
            "Public benchmark suite. k = mass = g = 1 on every task. "
            "That stiffness is not a fit. Run with: python main.py bench"
        ),
        "tasks": list(SUITE),
    }
    (root / "suite.json").write_text(canonical(suite), encoding="utf-8")
    return root


def main() -> None:
    root = write_catalog()
    print(f"wrote {len(SUITE)} scenarios to {root}")


if __name__ == "__main__":
    main()
