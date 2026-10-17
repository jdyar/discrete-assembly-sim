# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Structural screen, sag estimate, and the optional planner gate.

The three shapes the screen has to get right are a tower (height is
free), a cantilever (horizontal steps count, and past the limit fails),
and a bridge (a span is legal when half of it fits the limit). The sag
tests lock the closed forms a printed bar can be checked against, and
the gate tests lock the default: nothing in the old search changes
unless a StabilityPolicy is passed.
"""

from __future__ import annotations

import unittest

from main import DEPOT, build_world
from sim.choreographer import plan_task
from sim.geometry import SquareLattice2D
from sim.geometry3d import CubicLattice3D
from sim.planner import _approaches_for, plan_build_order, validate_plan
from sim.reservations import ReservationTable
from sim.stability import (
    assess_stability,
    axial_tower_compression,
    estimate_sag,
    is_stable,
    occupied_voxels,
    path_chain_deflection_at,
    shear_cantilever_tip_sag,
)
from sim.swarm import Swarm
from sim.world import World
from sim.world3d import World3D

# 3D frame: level increases upward, the whole of level 0 is the plate.
_FRAME = dict(up_axis=0, up_sign=1, ground_plane=0)
# Finite clamp under a single root, for the closed-form cantilever. The
# stock ground plane would fasten every first-course voxel, not just the root.
_CLAMP = dict(up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),))


def _overhang(cells, **kwargs):
    report = assess_stability(cells, **kwargs)
    return report, dict(report.overhang)


class TestIsStable(unittest.TestCase):
    def test_tower_any_height_is_column_stable(self) -> None:
        # Eight voxels on one planted root. max_cantilever 0 allows this
        # and would refuse a single sideways step.
        tower = [(level, 0, 0) for level in range(1, 9)]
        report, overhang = _overhang(tower, **_FRAME, max_cantilever=0)
        self.assertTrue(report.stable, report.reason)
        self.assertEqual(report.worst_overhang, 0)
        self.assertTrue(all(distance == 0 for distance in overhang.values()))

        with_a_nub = tower + [(8, 0, 1)]
        self.assertFalse(is_stable(with_a_nub, **_FRAME, max_cantilever=0))
        self.assertTrue(is_stable(with_a_nub, **_FRAME, max_cantilever=1))

    def test_cantilever_limit_is_horizontal_steps(self) -> None:
        # Root column in column 0, arm sticking out along +col at level 2.
        # Distances from the column: 1, 2, 3.
        cells = [(1, 0, 0), (2, 0, 0), (2, 0, 1), (2, 0, 2), (2, 0, 3)]
        report, overhang = _overhang(cells, **_FRAME, max_cantilever=3)
        self.assertTrue(report.stable, report.reason)
        self.assertEqual(overhang[(2, 0, 0)], 0)
        self.assertEqual(overhang[(2, 0, 1)], 1)
        self.assertEqual(overhang[(2, 0, 2)], 2)
        self.assertEqual(overhang[(2, 0, 3)], 3)
        self.assertEqual(report.worst_overhang, 3)
        self.assertFalse(is_stable(cells, **_FRAME, max_cantilever=2))
        # Unlimited span still requires a path to the base.
        self.assertTrue(is_stable(cells, **_FRAME, max_cantilever=None))

    def test_bridge_accepts_span_inside_limit_and_rejects_past_it(self) -> None:
        # Piers at col 0 and col 6. Five deck cells between them, so the
        # middle sits 3 horizontal steps from either column.
        piers = [(1, 0, 0), (1, 0, 6)]
        deck = [(2, 0, col) for col in range(7)]
        cells = piers + deck
        report, overhang = _overhang(cells, **_FRAME, max_cantilever=3)
        self.assertTrue(report.stable, report.reason)
        self.assertEqual(overhang[(2, 0, 0)], 0)
        self.assertEqual(overhang[(2, 0, 6)], 0)
        self.assertEqual(overhang[(2, 0, 1)], 1)
        self.assertEqual(overhang[(2, 0, 5)], 1)
        self.assertEqual(overhang[(2, 0, 3)], 3)
        self.assertEqual(report.worst_overhang, 3)
        self.assertFalse(is_stable(cells, **_FRAME, max_cantilever=2))

    def test_floating_gap_and_diagonal_contact_are_not_supported(self) -> None:
        self.assertFalse(is_stable([(2, 0, 0)], **_FRAME, max_cantilever=5))
        # A gap in the column: the upper voxel does not touch the pier.
        self.assertFalse(
            is_stable([(1, 0, 0), (3, 0, 0)], **_FRAME, max_cantilever=5)
        )
        # Corner contact only. (2, 0, 1) shares no face with the pier.
        diagonal = [(1, 0, 0), (2, 0, 1)]
        report, _overhangs = _overhang(diagonal, **_FRAME, max_cantilever=5)
        self.assertFalse(report.stable)
        self.assertIn((2, 0, 1), report.floating)
        self.assertNotIn((1, 0, 0), report.floating)

    def test_hanging_voxel_inherits_overhang(self) -> None:
        # Bolted-lattice assumption: a voxel may hang under a beam.
        # Level-1 under the tip would be sitting on the ground plane, so
        # the hanger is at level 2 under a level-3 beam, with air below.
        cells = [
            (1, 0, 0),
            (2, 0, 0),
            (3, 0, 0),
            (3, 0, 1),
            (3, 0, 2),
            (2, 0, 2),  # hangs under the tip
        ]
        report, overhang = _overhang(cells, **_FRAME, max_cantilever=2)
        self.assertTrue(report.stable, report.reason)
        self.assertEqual(overhang[(3, 0, 2)], 2)
        self.assertEqual(overhang[(2, 0, 2)], 2)
        self.assertNotIn((2, 0, 2), report.floating)

    def test_stacking_on_a_span_does_not_increase_overhang(self) -> None:
        cells = [
            (1, 0, 0),
            (2, 0, 0),
            (2, 0, 1),
            (2, 0, 2),
            (3, 0, 2),  # stacked on the tip
        ]
        _report, overhang = _overhang(cells, **_FRAME, max_cantilever=2)
        self.assertEqual(overhang[(2, 0, 2)], 2)
        self.assertEqual(overhang[(3, 0, 2)], 2)

    def test_finite_base_pad_is_not_an_infinite_floor(self) -> None:
        # No ground plane: only the one cell under the tower is plate.
        # The neighbor at level 1 is a cantilever of 1, not a second pier.
        pad = ((0, 0, 0),)
        tower = [(1, 0, 0)]
        self.assertTrue(
            is_stable(tower, up_axis=0, up_sign=1, ground_plane=None, base=pad)
        )
        neighbor = [(1, 0, 0), (1, 0, 1)]
        report, overhang = _overhang(
            neighbor,
            up_axis=0,
            up_sign=1,
            ground_plane=None,
            base=pad,
            max_cantilever=1,
        )
        self.assertTrue(report.stable, report.reason)
        self.assertEqual(overhang[(1, 0, 1)], 1)
        self.assertFalse(
            is_stable(
                [(1, 5, 5)],
                up_axis=0,
                up_sign=1,
                ground_plane=None,
                base=pad,
                max_cantilever=10,
            )
        )

    def test_empty_structure_is_stable(self) -> None:
        self.assertTrue(is_stable((), **_FRAME))

    def test_defective_voxel_still_counts_as_material(self) -> None:
        world = World(rows=5, cols=5)
        world.place_voxel((3, 2), defective=True)
        policy = SquareLattice2D(world).stability_policy(max_cantilever=0)
        cells = occupied_voxels(world)
        self.assertEqual(cells, ((3, 2),))
        self.assertTrue(is_stable(cells, policy))

    def test_geometry_frames_match_the_two_stock_lattices(self) -> None:
        flat = SquareLattice2D(World(rows=7, cols=10)).stability_policy()
        self.assertEqual(flat.up_axis, 0)
        self.assertEqual(flat.up_sign, -1)
        self.assertEqual(flat.ground_plane, 6)
        self.assertEqual(flat.max_cantilever, 0)

        cubic = CubicLattice3D(World3D(4, 4, 4)).stability_policy(2)
        self.assertEqual((cubic.up_axis, cubic.up_sign, cubic.ground_plane), (0, 1, 0))
        self.assertEqual(cubic.max_cantilever, 2)


class TestSag(unittest.TestCase):
    def test_one_voxel_cantilever_matches_shear_chain(self) -> None:
        # Fixed root plus one free voxel. The clamp is a finite pad under
        # the root: an infinite ground plane would plant the free voxel
        # too, and the arm would not be an arm. Tip sag is w / k_shear.
        # A deliberately different k_axial must not enter.
        cells = [(1, 0, 0), (1, 0, 1)]
        estimate = estimate_sag(
            cells, **_CLAMP, k_axial=50.0, k_shear=4.0, mass=2.0, g=3.0
        )
        expected = shear_cantilever_tip_sag(1, mass=2.0, g=3.0, k_shear=4.0)
        self.assertAlmostEqual(expected, 2.0 * 3.0 / 4.0)
        tip = estimate.displacements[(1, 0, 1)]
        self.assertAlmostEqual(-tip[0], expected)
        self.assertAlmostEqual(tip[1], 0.0)
        self.assertAlmostEqual(tip[2], 0.0)
        self.assertEqual(estimate.displacements[(1, 0, 0)], (0.0, 0.0, 0.0))
        self.assertEqual(estimate.worst_cell, (1, 0, 1))
        self.assertAlmostEqual(estimate.worst_sag, expected)
        self.assertAlmostEqual(estimate.worst_displacement, expected)
        self.assertEqual((estimate.n_fixed, estimate.n_free), (1, 1))

    def test_cantilever_profile_matches_closed_form(self) -> None:
        n = 4
        mass, g, k_shear = 0.4, 10.0, 2.5
        cells = [(1, 0, i) for i in range(n + 1)]
        estimate = estimate_sag(
            cells,
            **_CLAMP,
            k_axial=99.0,
            k_shear=k_shear,
            mass=mass,
            g=g,
        )
        for i in range(1, n + 1):
            expected = path_chain_deflection_at(
                i, n, weight=mass * g, stiffness=k_shear
            )
            got = estimate.displacements[(1, 0, i)]
            self.assertAlmostEqual(-got[0], expected)
            self.assertAlmostEqual(got[1], 0.0)
            self.assertAlmostEqual(got[2], 0.0)
        tip = shear_cantilever_tip_sag(n, mass=mass, g=g, k_shear=k_shear)
        self.assertAlmostEqual(estimate.worst_sag, tip)
        self.assertEqual(estimate.worst_sag_cell, (1, 0, n))
        self.assertEqual(estimate.worst_cell, (1, 0, n))

    def test_tower_compression_uses_axial_stiffness_only(self) -> None:
        n = 5
        mass, g, k_axial = 1.5, 2.0, 3.0
        # Level 1 is fastened to the plate; levels 2..n+1 are the n free voxels.
        cells = [(level, 0, 0) for level in range(1, n + 2)]
        estimate = estimate_sag(
            cells, **_FRAME, k_axial=k_axial, k_shear=100.0, mass=mass, g=g
        )
        expected = axial_tower_compression(n, mass=mass, g=g, k_axial=k_axial)
        top = estimate.displacements[(n + 1, 0, 0)]
        self.assertAlmostEqual(-top[0], expected)
        self.assertAlmostEqual(top[1], 0.0)
        self.assertAlmostEqual(top[2], 0.0)
        self.assertEqual(estimate.worst_sag_cell, (n + 1, 0, 0))
        self.assertAlmostEqual(estimate.worst_sag, expected)
        self.assertEqual(estimate.displacements[(1, 0, 0)], (0.0, 0.0, 0.0))

    def test_bridge_worst_sag_is_midspan(self) -> None:
        cells = [
            (1, 0, 0),
            (1, 0, 4),
            (2, 0, 0),
            (2, 0, 1),
            (2, 0, 2),
            (2, 0, 3),
            (2, 0, 4),
        ]
        estimate = estimate_sag(
            cells, **_FRAME, k_axial=8.0, k_shear=2.0, mass=1.0, g=1.0
        )

        def sag(cell):
            return -estimate.displacements[cell][0]

        self.assertAlmostEqual(sag((2, 0, 0)), sag((2, 0, 4)))
        self.assertAlmostEqual(sag((2, 0, 1)), sag((2, 0, 3)))
        self.assertGreater(sag((2, 0, 2)), sag((2, 0, 1)))
        self.assertGreater(sag((2, 0, 1)), sag((2, 0, 0)))
        self.assertGreater(sag((2, 0, 0)), 0.0)
        self.assertEqual(estimate.worst_sag_cell, (2, 0, 2))
        self.assertAlmostEqual(estimate.worst_displacement, estimate.worst_sag)
        self.assertEqual(estimate.displacements[(1, 0, 0)], (0.0, 0.0, 0.0))
        self.assertEqual(estimate.displacements[(1, 0, 4)], (0.0, 0.0, 0.0))

    def test_unanchored_structure_is_not_given_a_finite_sag(self) -> None:
        with self.assertRaises(ValueError):
            estimate_sag([(3, 0, 0)], **_FRAME)
        with self.assertRaises(ValueError):
            estimate_sag([(1, 0, 0), (3, 5, 5)], **_FRAME)

    def test_layer_fastened_to_the_base_does_not_move(self) -> None:
        cells = [(1, 0, 0), (1, 0, 1), (1, 0, 2)]
        estimate = estimate_sag(cells, **_FRAME, mass=2.0, g=9.8)
        self.assertEqual(estimate.worst_displacement, 0.0)
        self.assertIsNone(estimate.worst_cell)
        self.assertEqual(estimate.worst_sag, 0.0)
        self.assertEqual((estimate.n_fixed, estimate.n_free), (3, 0))

    def test_empty_sag_is_zero(self) -> None:
        estimate = estimate_sag((), **_FRAME)
        self.assertEqual(estimate.worst_displacement, 0.0)
        self.assertEqual(estimate.displacements, {})

    def test_two_dimensional_frame_sags_toward_the_ground_row(self) -> None:
        # Up is decreasing row. Only the cell under the root is plate, so
        # the neighbor is a real cantilever. It must move toward larger
        # row (down) by w / k_shear.
        cells = [(4, 2), (4, 3)]
        estimate = estimate_sag(
            cells,
            up_axis=0,
            up_sign=-1,
            ground_plane=None,
            base=((5, 2),),
            k_shear=2.0,
            k_axial=9.0,
            mass=1.0,
            g=1.0,
        )
        free = estimate.displacements[(4, 3)]
        self.assertAlmostEqual(free[0], 0.5)
        self.assertAlmostEqual(free[1], 0.0)
        self.assertAlmostEqual(estimate.worst_sag, 0.5)
        self.assertEqual(estimate.worst_sag_cell, (4, 3))
        self.assertEqual(estimate.displacements[(4, 2)], (0.0, 0.0))


class TestPlannerGate(unittest.TestCase):
    def test_gate_off_by_default_still_plans_a_floating_cell(self) -> None:
        # Two rows above the ground, nothing under it. Reachable from a
        # ground-level stance, and illegal the moment the gate is on.
        world = World(rows=7, cols=8)
        world.blueprint[4, 3] = True
        depot = (5, 0)
        plan = plan_build_order(world, depot)
        self.assertIsNotNone(plan)
        ok, why = validate_plan(world, plan, depot)
        self.assertTrue(ok, why)

        policy = SquareLattice2D(world).stability_policy(max_cantilever=10)
        self.assertIsNone(plan_build_order(world, depot, stability=policy))
        ok, why = validate_plan(world, plan, depot, stability=policy)
        self.assertFalse(ok)
        self.assertIn("not stable", why)

    def test_overlong_cantilever_is_infeasible_only_when_the_gate_is_on(self) -> None:
        world, depot = _cantilever_world()
        loose = SquareLattice2D(world).stability_policy(max_cantilever=2)
        tight = SquareLattice2D(world).stability_policy(max_cantilever=1)
        self.assertIsNotNone(plan_build_order(world, depot))
        self.assertIsNotNone(plan_build_order(world, depot, stability=loose))
        self.assertIsNone(plan_build_order(world, depot, stability=tight))

    def test_legal_cantilever_order_keeps_every_prefix_stable(self) -> None:
        world, depot = _cantilever_world()
        policy = SquareLattice2D(world).stability_policy(max_cantilever=2)
        plan = plan_build_order(world, depot, stability=policy)
        self.assertIsNotNone(plan)
        self.assertEqual(plan[0].cell, (5, 6))
        scratch = world.copy()
        for task in plan:
            scratch.place_voxel(task.cell)
            self.assertTrue(
                is_stable(occupied_voxels(scratch), policy),
                f"prefix ending at {task.cell} is unstable",
            )
        self.assertEqual([task.cell for task in plan][-1], (4, 4))
        ok, why = validate_plan(world, plan, depot, stability=policy)
        self.assertTrue(ok, why)

    def test_strict_columns_still_plan_the_grounded_wall(self) -> None:
        world = build_world()
        policy = SquareLattice2D(world).stability_policy(max_cantilever=0)
        plan = plan_build_order(world, DEPOT, stability=policy)
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan), 24)
        ok, why = validate_plan(world, plan, DEPOT, stability=policy)
        self.assertTrue(ok, why)

    def test_plan_task_refuses_the_tip_until_the_root_exists(self) -> None:
        world = World(rows=7, cols=10)
        depot = (5, 0)
        root = (5, 6)
        upper = (4, 6)
        tip = (4, 7)
        policy = SquareLattice2D(world).stability_policy(max_cantilever=2)

        refused = _plan_placement(world, depot, tip, policy)
        self.assertIsNone(refused)
        accepted_without_gate = _plan_placement(world, depot, tip, None)
        self.assertIsNotNone(accepted_without_gate)

        world.place_voxel(root)
        world.place_voxel(upper)
        accepted = _plan_placement(world, depot, tip, policy)
        self.assertIsNotNone(accepted)
        self.assertEqual(accepted.target, tip)

    def test_swarm_kickback_reorders_and_finishes_the_cantilever(self) -> None:
        # The robot starts next to the tip, so frontier claiming grabs the
        # tip first. The gate refuses it, kickback reorders, and the root
        # goes down before anything sticks out.
        world, depot = _cantilever_world()
        policy = SquareLattice2D(world).stability_policy(max_cantilever=2)
        swarm = Swarm(
            world,
            depot,
            starts=[(5, 1)],
            inspect_enabled=False,
            stability=policy,
        )
        for _ in range(2000):
            if swarm.done:
                break
            swarm.tick()
        self.assertTrue(
            swarm.done,
            f"not done at tick {swarm.tick_count}, kickbacks={swarm.kickbacks}",
        )
        self.assertTrue(world.complete)
        self.assertGreater(swarm.kickbacks, 0)
        places = [event for event in swarm.events if event[2] == "place"]
        self.assertEqual(places[0][3], (5, 6))
        self.assertEqual([event[3] for event in places], [
            (5, 6),
            (4, 6),
            (4, 5),
            (4, 4),
        ])


def _cantilever_world() -> tuple[World, tuple[int, int]]:
    """A root on the ground and a two-step arm toward the depot.

    Overhang of the tip is 2. Row 6 is ground; row 5 is the first course.
    """
    world = World(rows=7, cols=10)
    for cell in ((5, 6), (4, 6), (4, 5), (4, 4)):
        world.blueprint[cell] = True
    return world, (5, 0)


def _plan_placement(world, depot, target, stability):
    geometry = SquareLattice2D(world)
    return plan_task(
        0,
        depot,
        0,
        target,
        _approaches_for(target, geometry),
        geometry,
        ReservationTable(),
        depot,
        anchors=[],
        inspect_enabled=False,
        remaining_cells=(),
        stability=stability,
    )


if __name__ == "__main__":
    unittest.main()
