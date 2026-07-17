# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Unit tests for the cubic 3D lattice + motion model (Slice 4a).

The trap fixtures (test_traps3d) cover the coordination stack end-to-end;
these pin the geometry rules themselves — the 3D generalizations of the
machine-verified 2D BILL-E rules (NOTES.md, decided 2026-07-05) — and the
motion-model parameterization that Slice 4c builds on.
"""

import unittest

from sim.geometry import bfs_path
from sim.geometry3d import CubicLattice3D, MotionModel
from sim.planner import plan_build_order, validate_plan
from sim.world3d import World3D


class TestFooting(unittest.TestCase):
    def test_ground_level_cells_are_footing(self):
        world = World3D(3, 4, 4)
        geom = CubicLattice3D(world)
        self.assertTrue(geom.is_footing((1, 2, 2)))  # stands on ground

    def test_cell_above_empty_cavity_is_not_footing(self):
        """The tomb-trap insight: with four walls around a cavity, the
        cell ABOVE the cavity has no solid face-neighbor — a robot inside
        cannot climb out even before the roof exists."""
        world = World3D(4, 5, 5)
        for cell in [(1, 1, 2), (1, 3, 2), (1, 2, 1), (1, 2, 3)]:
            world.place_voxel(cell)
        geom = CubicLattice3D(world)
        self.assertTrue(geom.is_footing((1, 2, 2)))   # cavity floor: ground below
        self.assertFalse(geom.is_footing((2, 2, 2)))  # above cavity: nothing solid
        self.assertTrue(geom.is_footing((2, 1, 2)))   # atop a wall

    def test_occupied_cell_is_not_footing(self):
        world = World3D(3, 3, 3)
        world.place_voxel((1, 1, 1))
        geom = CubicLattice3D(world)
        self.assertFalse(geom.is_footing((1, 1, 1)))


class TestMovement(unittest.TestCase):
    def test_orthogonal_moves_on_ground(self):
        world = World3D(3, 3, 3)
        geom = CubicLattice3D(world)
        nbrs = set(geom.neighbors((1, 1, 1)))
        self.assertIn((1, 0, 1), nbrs)
        self.assertIn((1, 2, 1), nbrs)
        self.assertIn((1, 1, 0), nbrs)
        self.assertIn((1, 1, 2), nbrs)

    def test_corner_rounding_over_a_ledge(self):
        """Climb onto a block: (1,r,c) -> (2,r,c+1) rounds the block's top
        edge (one between-cell solid: the block; the other empty)."""
        world = World3D(4, 3, 4)
        world.place_voxel((1, 1, 2))
        geom = CubicLattice3D(world)
        self.assertIn((2, 1, 2), set(geom.neighbors((1, 1, 1))))

    def test_no_move_through_a_pinch(self):
        """Both between-cells solid = a pinch, not a corner (2D rule,
        held per axis-plane in 3D)."""
        world = World3D(3, 4, 4)
        world.place_voxel((1, 1, 2))
        world.place_voxel((1, 2, 1))
        geom = CubicLattice3D(world)
        self.assertNotIn((1, 2, 2), set(geom.neighbors((1, 1, 1))))

    def test_no_diagonal_across_open_gap(self):
        """Zero between-cells solid = nothing to pivot around."""
        world = World3D(4, 4, 4)
        geom = CubicLattice3D(world)
        # (2,1,1) and (2,2,2) are both non-footing anyway at level 2 with
        # no solids; use ground level where footing holds but no corner:
        nbrs = set(geom.neighbors((1, 1, 1)))
        self.assertNotIn((1, 2, 2), nbrs)  # both between-cells empty


class TestReach(unittest.TestCase):
    def test_default_reach_is_26_neighborhood(self):
        geom = CubicLattice3D(World3D(3, 3, 3))
        self.assertEqual(len(geom.reach_cells((1, 1, 1))), 26)

    def test_reach_is_symmetric_at_every_radius(self):
        """The planner's stance inversion requires symmetry (Geometry
        contract) — must hold for every 4c radius setting."""
        world = World3D(9, 9, 9)
        for radius in (1, 2, 4):
            geom = CubicLattice3D(world, MotionModel(reach_radius=radius))
            a, b = (4, 4, 4), (4 + radius, 4, 4 - radius)
            self.assertIn(b, geom.reach_cells(a))
            self.assertIn(a, geom.reach_cells(b))

    def test_extended_reach_widens_the_stance_set(self):
        """Slice 4c grounding: a larger radius yields a strict SUPERSET
        of placement stances, including stances a full gap away from the
        target — the geometric fact that lets extended-reach robots keep
        working from uncongested cells. (Whether reach may pass THROUGH
        solid volume — vs articulate around corners — is an explicit 4c
        modeling decision, flagged in sim/geometry3d.py; radius 1 matches
        the machine-verified 2D convention exactly.)"""
        world = World3D(4, 5, 8)
        target = (1, 2, 4)

        def stances(geom):
            return {
                s for s in geom.reach_cells(target)
                if geom.is_footing(s) and target in geom.reach_cells(s)
            }

        r1 = stances(CubicLattice3D(world, MotionModel(reach_radius=1)))
        r2 = stances(CubicLattice3D(world, MotionModel(reach_radius=2)))
        self.assertTrue(r1 < r2)  # strict superset
        self.assertIn((1, 2, 2), r2)     # places across the gap at (1,2,3)
        self.assertNotIn((1, 2, 2), r1)


class TestPlannerOn3D(unittest.TestCase):
    def test_single_robot_builds_a_box_end_to_end(self):
        """plan_build_order + validate_plan run unchanged on 3D nodes:
        a 3x3x2 solid box, planned and independently re-validated."""
        world = World3D(4, 6, 6)
        world.set_box_blueprint(width=3, depth=3, height=2, left=2, front=2)
        depot = (1, 0, 0)
        plan = plan_build_order(world, depot, CubicLattice3D)
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan), world.blueprint_count)
        ok, why = validate_plan(world, plan, depot, CubicLattice3D)
        self.assertTrue(ok, why)

    def test_depot_stays_connected_throughout(self):
        world = World3D(4, 5, 5)
        world.set_box_blueprint(width=2, depth=2, height=2, left=2, front=2)
        depot = (1, 0, 0)
        plan = plan_build_order(world, depot, CubicLattice3D)
        self.assertIsNotNone(plan)
        scratch = world.copy()
        geom = CubicLattice3D(scratch)
        for task in plan:
            scratch.occupancy[task.cell] = 2  # VOXEL
            self.assertIsNotNone(
                bfs_path(geom, task.approach, {depot}),
                f"stranded after placing {task.cell}",
            )


if __name__ == "__main__":
    unittest.main()
