# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Slice 2 tests: defect model, inspect/remove states, replan loop, yield."""

from __future__ import annotations

import unittest

from main import DEPOT, build_world, run_slice2
from sim.planner import plan_build_order
from sim.robot import IDLE, INSPECT, PLACE, REMOVE, Robot, is_grip
from sim.world import DEFECT, VOXEL, World


class ScriptedRng:
    """random.Random stand-in returning scripted values for .random()."""

    def __init__(self, values):
        self.values = list(values)

    def random(self):
        return self.values.pop(0) if self.values else 1.0


class TestWorldDefects(unittest.TestCase):
    def test_defective_placement_and_removal(self) -> None:
        w = World(rows=4, cols=3)
        w.blueprint[2, 1] = True
        w.place_voxel((2, 1), defective=True)
        self.assertTrue(w.is_defective((2, 1)))
        self.assertEqual(w.built_count, 0)  # defects don't count
        self.assertEqual(w.defect_count, 1)
        self.assertFalse(w.complete)
        w.remove_voxel((2, 1))
        self.assertEqual(w.defect_count, 0)
        w.place_voxel((2, 1))
        self.assertEqual(w.built_count, 1)
        self.assertTrue(w.complete)

    def test_defect_is_crawlable(self) -> None:
        w = World(rows=5, cols=3)
        w.place_voxel((3, 1), defective=True)
        self.assertTrue(is_grip(w, (2, 1)))  # can stand on a defect
        self.assertFalse(is_grip(w, (3, 1)))  # but not inside it


class TestInspectRemoveStates(unittest.TestCase):
    def test_defect_flow_place_inspect_remove_halt(self) -> None:
        w = World(rows=4, cols=4)
        w.blueprint[2, 2] = True
        # First roll 0.0 < p forces a defect on the first placement.
        robot = Robot(pos=(2, 1), depot=(2, 0), defect_p=0.5,
                      rng=ScriptedRng([0.0, 0.9]))
        robot.load_plan([(2, 2)])
        # Drive to PLACE: TO_DEPOT -> ... -> PICK -> TO_SITE -> PLACE
        for _ in range(50):
            if robot.state == PLACE:
                break
            robot.tick(w)
        robot.tick(w)  # PLACE: defective voxel goes in
        self.assertTrue(w.is_defective((2, 2)))
        self.assertEqual(robot.state, INSPECT)
        robot.tick(w)  # INSPECT: detects
        self.assertEqual(robot.state, REMOVE)
        self.assertEqual(robot.defects_found, 1)
        robot.tick(w)  # REMOVE: discards + halts
        self.assertEqual(w.defect_count, 0)
        self.assertTrue(robot.needs_replan)
        self.assertEqual(robot.state, IDLE)
        # Halted: does not self-resume without a fresh plan.
        robot.tick(w)
        self.assertEqual(robot.state, IDLE)

    def test_good_placement_consumes_task(self) -> None:
        w = World(rows=4, cols=4)
        w.blueprint[2, 2] = True
        robot = Robot(pos=(2, 1), depot=(2, 0), defect_p=0.5,
                      rng=ScriptedRng([0.9]))  # roll above p: good part
        robot.load_plan([(2, 2)])
        for _ in range(50):
            robot.tick(w)
            if robot.state == IDLE and not robot.tasks:
                break
        self.assertTrue(w.complete)
        self.assertFalse(robot.needs_replan)
        self.assertEqual(robot.defects_found, 0)
        self.assertEqual(robot.inspections, 1)


class TestPlannerDefectGuard(unittest.TestCase):
    def test_planner_rejects_dirty_world(self) -> None:
        w = build_world()
        w.place_voxel((5, 2), defective=True)
        with self.assertRaises(ValueError):
            plan_build_order(w, DEPOT)


class TestSlice2Runs(unittest.TestCase):
    def test_corrected_run_reaches_full_yield(self) -> None:
        world = build_world()
        log, robot = run_slice2(world, defect_p=0.15, seed=3, verbose=False)
        self.assertTrue(world.complete)
        self.assertEqual(world.defect_count, 0)
        self.assertGreater(robot.defects_found, 0)  # seed must exercise repair
        self.assertEqual(log.meta["replans"], robot.defects_found)

    def test_baseline_leaves_defects(self) -> None:
        world = build_world()
        run_slice2(world, defect_p=1.0, correction=False, seed=1, verbose=False)
        # Every placement defective, none detected: zero yield, full of scrap.
        self.assertEqual(world.built_count, 0)
        self.assertEqual(world.defect_count, world.blueprint_count)

    def test_demo_seed_shows_repairs_and_logs_defects(self) -> None:
        world = build_world()
        log, robot = run_slice2(world, defect_p=0.12, seed=7, verbose=False)
        self.assertTrue(world.complete)
        self.assertGreater(robot.defects_found, 0, "demo seed shows no repair")
        self.assertEqual(log.meta["legend"]["DEFECT"], DEFECT)
        # At least one logged frame actually shows a defective voxel.
        self.assertTrue(
            any(DEFECT in row for f in log.frames for row in f["occupancy"]),
            "no frame captured the defect",
        )
        # And the wall ends made of good voxels only.
        final = log.frames[-1]["occupancy"]
        self.assertFalse(any(DEFECT in row for row in final))
        self.assertEqual(sum(row.count(VOXEL) for row in final), 24)


if __name__ == "__main__":
    unittest.main()
