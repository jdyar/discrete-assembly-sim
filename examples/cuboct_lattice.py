# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Cuboctahedral voxels on the cubic grid — worked blueprint.

The solid is the face-joined cuboctahedron used by NASA ARMADAS
(Gregg et al., Science Robotics 2024): six square faces, four joint
sites at the corners of each shared face. Centers stay on the integer
cubic grid, so the cell has the same six face-neighbors as a cube, and
the same surface locomotion.

This is not a mechanical model of that hardware, and it is not the 2013
vertex-connected octahedron lattice (Cheung & Gershenfeld, Science).
Citations, the joint-spring assumptions, and the list of what changes
are in sim/cuboct.py.

Run it:  python examples/cuboct_lattice.py
"""

import sys
from pathlib import Path

try:  # installed package, or run from the repo root
    import sim  # noqa: F401
except ImportError:  # run as `python examples/cuboct_lattice.py` pre-install
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.cuboct import (
    EXAMPLE_DEPOT,
    EXAMPLE_START,
    CuboctahedralLattice3D,
    example_world,
    face_joint,
)
from sim.geometry3d import CubicLattice3D
from sim.planner import plan_build_order, validate_plan
from sim.swarm import Swarm


def main() -> None:
    world = example_world()
    depot = EXAMPLE_DEPOT
    cuboct = CuboctahedralLattice3D(world)
    cubic = CubicLattice3D(world)
    # Same footing graph before anything is built: the ground course.
    stance = (1, 2, 2)
    assert cuboct.is_footing(stance) and cubic.is_footing(stance)
    assert set(cuboct.neighbors(stance)) == set(cubic.neighbors(stance))

    joint = face_joint((1, 2, 2), (1, 2, 3), cuboct.voxel_shape)
    cube_joint = face_joint((1, 2, 2), (1, 2, 3), cubic.voxel_shape)
    assert joint.center == cube_joint.center
    assert joint.sites != cube_joint.sites
    assert len(joint.sites) == 4

    plan = plan_build_order(world, depot, CuboctahedralLattice3D)
    assert plan is not None, "cuboct blueprint has no build order"
    ok, why = validate_plan(world, plan, depot, CuboctahedralLattice3D)
    assert ok, why

    swarm = Swarm(
        world,
        depot,
        starts=[EXAMPLE_START],
        geometry_factory=CuboctahedralLattice3D,
    )
    while not swarm.done and swarm.tick_count < 8000:
        swarm.tick()
    assert world.complete, (
        f"cuboct build incomplete {world.built_count}/{world.blueprint_count}"
    )
    print(
        f"cuboctahedral lattice: {world.blueprint_count} voxels in "
        f"{swarm.tick_count} ticks, kickbacks={swarm.kickbacks}"
    )
    print(
        "face topology matches the cubic lattice "
        "(6 face-neighbors, same locomotion). "
        "Joint sites are the four corners of each shared square, "
        "not the cube's face corners."
    )
    print(
        "complete. Coordination stack ran UNCHANGED. "
        "This is a shape and joint-location model, not ARMADAS stiffness."
    )


if __name__ == "__main__":
    main()
