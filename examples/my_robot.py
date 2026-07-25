# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Bring-your-own-robot, worked example: hardware capability as config.

Your robot is not our robot. Maybe it places blocks 3 cells away, takes
2-cell inchworm strides, or works as a rigidly coupled pair with a
longer combined arm. In this stack those are MotionModel FIELDS, not
code changes — the same planner, gates, and reservation table run every
configuration below.

Run it:  python examples/my_robot.py
"""

import sys
from pathlib import Path

try:  # installed package, or run from the repo root
    import sim  # noqa: F401
except ImportError:  # run as `python examples/my_robot.py` pre-install
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.geometry3d import CubicLattice3D, MotionModel
from sim.swarm import Swarm
from sim.world3d import World3D


def build(motion: MotionModel, label: str) -> int:
    """One robot builds a small 3D box under the given motion model."""
    world = World3D(4, 6, 6)
    world.set_box_blueprint(width=3, depth=3, height=2, left=2, front=2)
    depot = (1, 0, 0)
    swarm = Swarm(
        world, depot,
        starts=[(1, 5, 0)],
        geometry_factory=lambda w: CubicLattice3D(w, motion),
    )
    while not swarm.done and swarm.tick_count < 5000:
        swarm.tick()
    assert world.complete, f"{label}: build incomplete"
    print(f"{label:<44} {swarm.tick_count:>5} ticks")
    return swarm.tick_count


def main() -> None:
    print("same blueprint, same coordination code, different hardware:\n")
    base = build(MotionModel(), "baseline stepper (reach 1, stride 1)")
    build(
        MotionModel(reach_radius=3),
        "long snake-arm (reach 3): places around corners",
    )
    build(
        MotionModel(stride=2),
        "inchworm strides (stride 2): SOLL-E-class gait",
    )
    pair = MotionModel(reach_radius=1).coupled(MotionModel(reach_radius=1))
    ticks = build(pair, "coupled pair (combined reach 2, one logical robot)")
    print(
        f"\nbaseline {base} ticks -> coupled pair {ticks} ticks, "
        "with zero changes to the coordination stack.\n"
        "Swap in YOUR numbers and find out what your hardware buys."
    )


if __name__ == "__main__":
    main()
