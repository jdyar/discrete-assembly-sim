# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Planner tests: build-order search, plan validation, replan interface."""

from __future__ import annotations

import unittest

from main import DEPOT, build_world
from sim.planner import Task, plan_build_order, validate_plan
from sim.world import World


class TestPlanBuildOrder(unittest.TestCase):
    def test_full_wall_plan_found_and_valid(self) -> None:
        world = build_world()
        plan = plan_build_order(world, DEPOT)
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan), 24)
        ok, why = validate_plan(world, plan, DEPOT)
        self.assertTrue(ok, why)
        # Search must not have mutated the world.
        self.assertEqual(world.built_count, 0)

    def test_plan_covers_blueprint_exactly_once(self) -> None:
        world = build_world()
        plan = plan_build_order(world, DEPOT)
        cells = [t.cell for t in plan]
        self.assertEqual(len(cells), len(set(cells)))
        self.assertTrue(all(world.blueprint[c] for c in cells))

    def test_replan_from_partial_world(self) -> None:
        """Plan/execute/replan contract: planning is callable mid-build."""
        world = build_world()
        plan = plan_build_order(world, DEPOT)
        # Execute the first 10 placements directly, then replan the rest.
        for task in plan[:10]:
            world.place_voxel(task.cell)
        replan = plan_build_order(world, DEPOT)
        self.assertIsNotNone(replan)
        self.assertEqual(len(replan), 14)
        ok, why = validate_plan(world, replan, DEPOT)
        self.assertTrue(ok, why)

    def test_empty_blueprint_plans_empty(self) -> None:
        world = World(rows=4, cols=4)
        self.assertEqual(plan_build_order(world, (2, 0)), [])

    def test_unreachable_blueprint_is_infeasible(self) -> None:
        # Blueprint floating far above the ground: no grip cell can reach
        # the first placement, so search exhausts and proves infeasibility.
        world = World(rows=8, cols=6)
        world.blueprint[1, 3] = True
        self.assertIsNone(plan_build_order(world, (6, 0)))


class TestValidatePlan(unittest.TestCase):
    def test_rejects_bad_approach(self) -> None:
        world = build_world()
        bad = [Task(cell=(5, 2), approach=(1, 1))]  # mid-air stance
        ok, why = validate_plan(world, bad, DEPOT)
        self.assertFalse(ok)
        self.assertIn("grip", why)

    def test_rejects_out_of_reach(self) -> None:
        world = build_world()
        bad = [Task(cell=(5, 7), approach=(5, 1))]  # stance too far away
        ok, why = validate_plan(world, bad, DEPOT)
        self.assertFalse(ok)
        self.assertIn("reach", why)

    def test_rejects_incomplete_plan(self) -> None:
        world = build_world()
        plan = plan_build_order(world, DEPOT)
        ok, why = validate_plan(world, plan[:-1], DEPOT)
        self.assertFalse(ok)
        self.assertIn("complete", why)


if __name__ == "__main__":
    unittest.main()
