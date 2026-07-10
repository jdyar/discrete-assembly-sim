# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Slice 1 robot tests: grip rule, surface locomotion, reach, BFS,
one-action-per-tick discipline, and the end-to-end full-wall build.
"""

from __future__ import annotations

import unittest

from main import DEPOT, build_world, run_robot, run_slice1
from sim.robot import (
    IDLE,
    Robot,
    bfs_path,
    is_grip,
    legal_moves,
    reach_cells,
)
from sim.world import World


class TestGrip(unittest.TestCase):
    def test_grip_needs_adjacent_solid(self) -> None:
        w = World(rows=5, cols=4)  # ground at row 4
        self.assertTrue(is_grip(w, (3, 0)))  # on ground
        self.assertFalse(is_grip(w, (2, 0)))  # mid-air
        w.place_voxel((3, 1))
        self.assertTrue(is_grip(w, (2, 1)))  # on top of voxel
        self.assertTrue(is_grip(w, (3, 2)))  # beside voxel (and on ground)
        self.assertFalse(is_grip(w, (3, 1)))  # occupied
        self.assertFalse(is_grip(w, (-1, 0)))  # out of bounds

    def test_grip_on_vertical_face(self) -> None:
        w = World(rows=6, cols=4)
        w.place_voxel((4, 1))
        w.place_voxel((3, 1))  # 2-high tower
        self.assertTrue(is_grip(w, (3, 0)))  # hugging its left face
        self.assertTrue(is_grip(w, (3, 2)))  # hugging its right face
        self.assertTrue(is_grip(w, (2, 1)))  # on top


class TestSurfaceMoves(unittest.TestCase):
    def test_climbs_tall_faces(self) -> None:
        w = World(rows=6, cols=4)
        w.place_voxel((4, 1))
        w.place_voxel((3, 1))  # 2-high tower: step-climber couldn't climb it
        path = bfs_path(w, (4, 0), {(2, 1)})  # ground to tower top
        self.assertIsNotNone(path)

    def test_corner_rounding_not_gap_jumping(self) -> None:
        w = World(rows=6, cols=5)
        w.place_voxel((4, 2))
        moves = set(legal_moves(w, (3, 2)))  # on top of the voxel
        self.assertIn((4, 1), moves)   # round the corner down its left face
        self.assertIn((4, 3), moves)   # ... and its right face
        # From beside the voxel, no diagonal into free air (nothing to pivot on).
        w2 = World(rows=6, cols=5)
        moves2 = set(legal_moves(w2, (4, 2)))  # standing on bare ground
        self.assertEqual(moves2, {(4, 1), (4, 3)})  # orthogonal only

    def test_no_squeeze_through_pinch(self) -> None:
        w = World(rows=6, cols=5)
        w.place_voxel((4, 1))
        w.place_voxel((3, 2))  # diagonal pinch between the two voxels
        self.assertNotIn((3, 1), set(legal_moves(w, (4, 2))))


class TestReach(unittest.TestCase):
    def test_reach_is_all_eight(self) -> None:
        self.assertEqual(len(reach_cells((3, 4))), 8)
        self.assertIn((2, 3), reach_cells((3, 4)))  # diagonal included
        self.assertNotIn((3, 4), reach_cells((3, 4)))  # not itself


class TestFullWall(unittest.TestCase):
    def test_builds_full_wall(self) -> None:
        world = build_world()
        log, robot = run_slice1(world, verbose=False)
        self.assertTrue(world.complete, "wall incomplete")
        self.assertEqual(world.built_count, 24)
        self.assertEqual(robot.state, IDLE)
        self.assertFalse(robot.carrying)
        # Never stranded: the robot can still crawl back to the depot.
        self.assertIsNotNone(bfs_path(world, robot.pos, {DEPOT}))

    def test_one_action_per_tick(self) -> None:
        world = build_world()
        log, _robot = run_slice1(world, verbose=False)
        frames = log.frames
        for a, b in zip(frames, frames[1:]):
            (r0, c0), (r1, c1) = a["robot"]["pos"], b["robot"]["pos"]
            self.assertLessEqual(abs(c1 - c0), 1)
            self.assertLessEqual(abs(r1 - r0), 1)
            if (r0, c0) != (r1, c1):  # a move tick never also places
                self.assertEqual(a["placed"], b["placed"])
            self.assertLessEqual(b["placed"] - a["placed"], 1)

    def test_robot_always_on_legal_grip(self) -> None:
        world = build_world()
        from sim.planner import plan_build_order

        plan = plan_build_order(world, DEPOT)
        self.assertIsNotNone(plan)
        robot = Robot(pos=DEPOT, depot=DEPOT, tasks=plan)
        for _ in range(2000):
            robot.tick(world)
            self.assertTrue(is_grip(world, robot.pos), f"illegal grip {robot.pos}")
            if robot.state == IDLE and not robot.tasks:
                break
        self.assertTrue(world.complete)

    def test_bare_cell_tasks_still_work(self) -> None:
        """Robots accept bare target cells (any stance) — used by tests/demos."""
        world = build_world()
        bottom = world.rows - 2
        tasks = [(bottom, c) for c in range(2, 8)]  # bottom course only
        log, robot = run_robot(world, tasks)
        self.assertEqual(world.built_count, 6)
        self.assertEqual(robot.state, IDLE)


if __name__ == "__main__":
    unittest.main()
