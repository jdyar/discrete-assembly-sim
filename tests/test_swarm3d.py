# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Slice 4b: coordinated repair in 3D + fuzzed coupled-loop tests.

Repair: the differentiator — a defect detected mid-swarm must be
removed and rebuilt THROUGH live reservations while at least one other
robot makes real progress (the swarm is never frozen for a repair).
Same contract as the 2D repair-in-a-crowd fixture, on the 3D box.

Fuzz: the coupled sequencer<->choreographer loop under randomized
blueprints, robot counts, and seeds. Every run asserts the per-tick
invariants (no collisions, no entrapment) and completion. Failures
print the scenario tuple — paste it into a named fixture to pin it.
"""

from __future__ import annotations

import random
import unittest

from sim.geometry import bfs_path
from sim.geometry3d import CubicLattice3D, MotionModel
from sim.swarm import Swarm, SwarmStuck
from sim.world3d import World3D

MAX_TICKS = 4000


def drive(test, swarm, world, depot, budget=MAX_TICKS, label=""):
    while not swarm.done and swarm.tick_count < budget:
        swarm.tick()
        positions = [r.pos for r in swarm.robots]
        test.assertEqual(
            len(positions), len(set(positions)),
            f"{label}: collision at tick {swarm.tick_count}: {positions}",
        )
        geom = CubicLattice3D(world)
        for r in swarm.robots:
            test.assertIsNotNone(
                bfs_path(geom, r.pos, {depot}),
                f"{label}: robot {r.id} entrapped at {r.pos}, "
                f"tick {swarm.tick_count}",
            )
    test.assertTrue(swarm.done, f"{label}: not done after {budget} ticks")
    test.assertTrue(world.complete, f"{label}: blueprint incomplete")


class TestRepairInACrowd3D(unittest.TestCase):
    """Forced defect on the 3D box mid-swarm; repair threads through
    live reservations; liveness asserted for the rest of the crew."""

    def test_repair_completes_with_swarm_liveness(self):
        world = World3D(5, 8, 10)
        world.set_box_blueprint(width=3, depth=3, height=2, left=4, front=3)
        depot = (1, 4, 1)
        first_cell = (1, 3, 4)  # an early, low placement: repair mid-build
        swarm = Swarm(
            world, depot,
            starts=[(1, 2, 1), (1, 6, 1), (1, 4, 8)],
            defect_cells={first_cell},
            geometry_factory=CubicLattice3D,
        )
        drive(self, swarm, world, depot, label="repair3d")

        events = swarm.events  # (tick, robot_id, kind, node)
        removed = [e for e in events if e[2] == "defect_removed"]
        self.assertEqual(len(removed), 1, "exactly one forced defect")
        t_removed, detector, _, cell = removed[0]
        self.assertEqual(cell, first_cell)
        replaced = [
            e for e in events
            if e[2] == "place" and e[3] == first_cell and e[0] > t_removed
        ]
        self.assertTrue(replaced, "defective cell was never rebuilt")
        t_replaced = replaced[0][0]
        others_alive = [
            e for e in events
            if t_removed <= e[0] <= t_replaced
            and e[1] != detector
            and e[2] in ("move", "place", "pick")
        ]
        self.assertTrue(
            others_alive,
            "no other robot progressed during the repair window — swarm froze",
        )


class TestConcurrentDefectResequence(unittest.TestCase):
    """Regression: robot A's repair-resequence while robot B's defect is
    still standing must DEFER, not crash (found by the 3D yield
    experiment at N=2, p=0.08 — plan_build_order refuses dirty worlds,
    and the single-robot 'remove before replanning' contract doesn't
    survive concurrency)."""

    def test_repair_defers_while_another_defect_stands(self):
        world = World3D(5, 8, 10)
        world.set_box_blueprint(width=3, depth=3, height=2, left=4, front=3)
        depot = (1, 4, 1)
        swarm = Swarm(
            world, depot, starts=[(1, 2, 1), (1, 6, 1)],
            geometry_factory=CubicLattice3D,
        )
        # Robot B's defective voxel is standing (inspect pending)...
        standing = (1, 3, 4)
        world.place_voxel(standing, defective=True)
        # ...when robot A's repair triggers a resequence. Must not raise.
        swarm.sequencer.repair((1, 3, 6))
        self.assertTrue(swarm.sequencer.needs_resequence)
        # B's pipeline removes its defect; the next tick consumes the flag.
        world.remove_voxel(standing)
        swarm.tick()
        self.assertFalse(swarm.sequencer.needs_resequence)
        self.assertIn((1, 3, 6), swarm.sequencer.queue)


class TestStrideSwarm3D(unittest.TestCase):
    """Two stride-2 robots share the stage: multi-cell moves thread the
    live reservation table without collisions or entrapment (the
    footprint machinery under real coupled-loop traffic)."""

    def test_two_stride2_robots_build_the_box(self):
        world = World3D(5, 8, 10)
        world.set_box_blueprint(width=3, depth=3, height=2, left=4, front=3)
        depot = (1, 4, 1)
        factory = lambda w: CubicLattice3D(w, MotionModel(stride=2))
        swarm = Swarm(
            world, depot,
            starts=[(1, 2, 1), (1, 6, 1)],
            geometry_factory=factory,
        )
        drive(self, swarm, world, depot, label="stride2-swarm")


class TestCoupledPairSwarm3D(unittest.TestCase):
    """The coupled-robot model at the swarm level: one PAIR (a single
    logical robot with combined reach 2) builds the box — coupling is a
    swarm-setup config, zero coordination changes (modeling decision
    2026-07-19, see sim/geometry3d.py MotionModel docstring)."""

    def test_coupled_pair_builds_the_box(self):
        world = World3D(5, 8, 10)
        world.set_box_blueprint(width=3, depth=3, height=2, left=4, front=3)
        depot = (1, 4, 1)
        pair = MotionModel(reach_radius=1).coupled(MotionModel(reach_radius=1))
        factory = lambda w: CubicLattice3D(w, pair)
        swarm = Swarm(
            world, depot, starts=[(1, 2, 1)], geometry_factory=factory,
        )
        drive(self, swarm, world, depot, label="coupled-pair")


class TestFuzzCoupledLoop3D(unittest.TestCase):
    """Randomized scenarios through the full coupled loop.

    Scenario space: solid/hollow boxes and walls of random dimensions,
    1-3 robots on distinct ground starts, defects on/off, reach radius
    1 or 2. Small worlds keep each run in the seconds range; the seeds
    are FIXED so the suite is deterministic — bump FUZZ_CASES locally
    for a longer campaign (main.py fuzz3d runs an open-ended one).
    """

    FUZZ_CASES = 8

    def test_fuzz(self):
        for case in range(self.FUZZ_CASES):
            rng = random.Random(1000 + case)
            levels = rng.choice((4, 5))
            rows, cols = rng.choice(((7, 9), (8, 8), (6, 10)))
            world = World3D(levels, rows, cols)
            width = rng.randint(2, min(4, cols - 4))
            depth = rng.randint(2, min(4, rows - 4))
            height = rng.randint(1, min(3, levels - 2))
            left = rng.randint(2, cols - width - 2)
            front = rng.randint(2, rows - depth - 2)
            hollow = rng.random() < 0.5
            world.set_box_blueprint(
                width=width, depth=depth, height=height,
                left=left, front=front, hollow=hollow,
            )
            depot = (1, rng.randrange(rows), 0)
            n_robots = rng.randint(1, 3)
            ground = [
                (1, r, c) for r in range(rows) for c in range(cols)
                if not world.blueprint[1, r, c] and (1, r, c) != depot
            ]
            starts = rng.sample(ground, n_robots)
            defect_p = rng.choice((0.0, 0.0, 0.1))
            reach = rng.choice((1, 1, 2))
            stride = rng.choice((1, 1, 2))
            label = (
                f"case={case} box={width}x{depth}x{height}@({front},{left}) "
                f"hollow={hollow} world={levels}x{rows}x{cols} depot={depot} "
                f"starts={starts} p={defect_p} reach={reach} stride={stride}"
            )
            motion = MotionModel(reach_radius=reach, stride=stride)
            geometry_factory = (
                lambda w, _m=motion: CubicLattice3D(w, _m)
            )
            with self.subTest(label):
                swarm = Swarm(
                    world, depot, starts=starts,
                    defect_p=defect_p, rng=random.Random(case),
                    geometry_factory=geometry_factory,
                )
                try:
                    drive(self, swarm, world, depot, label=label)
                except SwarmStuck as exc:  # pragma: no cover
                    self.fail(f"{label}: SwarmStuck: {exc}")


if __name__ == "__main__":
    unittest.main()
