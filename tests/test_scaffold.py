# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Temporary scaffold: insert the minimum, build, then take it down.

The on-lattice inchworm grips faces of voxels that are already there,
so it cannot walk the open ground to the far pier. A bridge whose
finished deck is inside the cantilever and sag caps still has no plain
order: reaching that pier cantilevers past both. Support cells under
the deck make an order, and taking them out at the end stays inside
the same caps. A crane that walks the floor needs none. A finished
blueprint that already fails the caps is not rescued.
"""

from __future__ import annotations

import unittest

from experiments.on_lattice import GAIT
from experiments.scaffold_campaign import (
    disjoint_top_stances,
    inchworm_factory,
    plant_entry_pad,
)
from experiments.stability_cost import build_bridge, build_wall
from sim.geometry import bfs_path
from sim.geometry3d import CubicLattice3D
from sim.inchworm import OnLatticeGeometry, entry_stances
from sim.planner import plan_build_order
from sim.render import render_ascii3d
from sim.scaffold import (
    CRANE,
    INCHWORM,
    caps_for_finished,
    check_assembly,
    finished_cells,
    geometry_factory,
    near_yard,
    plan_with_scaffold,
)
from sim.stability import SagCap, is_stable, occupied_voxels, sag_within
from sim.swarm import Swarm
from sim.world import SCAFFOLD

_MAX_NODES = 40_000
_CACHE: dict = {}


def _solve(kind: str, span: int | None = None):
    """Plan once per (gait, shape). Later tests reuse the same plan."""
    key = (kind, span)
    if key in _CACHE:
        return _CACHE[key]
    pad: tuple = ()
    start_pool: list = []
    if kind == "wall":
        world, depot = build_wall()
        factory = geometry_factory(CRANE)
    elif kind == "inchworm":
        world, depot = build_bridge(span)
        pad = plant_entry_pad(world)
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
        factory = inchworm_factory(policy)
        stances = disjoint_top_stances(factory(world), pad)
        depot = stances[0]
        start_pool = stances[1:]
    else:
        world, depot = build_bridge(span)
        factory = geometry_factory(CRANE)
    if kind != "inchworm":
        policy, sag = caps_for_finished(world, CubicLattice3D(world).stability_policy(0))
    plain = plan_build_order(
        world,
        depot,
        factory,
        max_nodes=_MAX_NODES,
        stability=policy,
        sag=sag,
    )
    plan = plan_with_scaffold(
        world,
        depot,
        factory,
        policy,
        sag,
        max_nodes=_MAX_NODES,
    )
    solved = {
        "world": world,
        "depot": depot,
        "factory": factory,
        "policy": policy,
        "sag": sag,
        "plain": plain,
        "plan": plan,
        "pad": pad,
        "start_pool": start_pool,
    }
    _CACHE[key] = solved
    return solved


class TestScaffoldFeasibility(unittest.TestCase):
    def test_inchworm_span5_was_infeasible(self) -> None:
        case = _solve("inchworm", 5)
        self.assertIsNone(case["plain"])
        plan = case["plan"]
        self.assertIsNotNone(plan)
        self.assertGreater(plan.scaffold_count, 0)
        ok, reason = check_assembly(
            case["world"], plan, case["depot"], case["factory"], case["policy"], case["sag"]
        )
        self.assertTrue(ok, reason)
        finished = finished_cells(case["world"])
        self.assertTrue(is_stable(finished, case["policy"]))
        self.assertTrue(sag_within(finished, case["policy"], case["sag"]))
        self.assertGreater(len(case["pad"]), 0)

    def test_inchworm_span6_was_infeasible(self) -> None:
        case = _solve("inchworm", 6)
        self.assertIsNone(case["plain"])
        plan = case["plan"]
        self.assertIsNotNone(plan)
        self.assertGreater(plan.scaffold_count, 0)
        ok, reason = check_assembly(
            case["world"], plan, case["depot"], case["factory"], case["policy"], case["sag"]
        )
        self.assertTrue(ok, reason)


class TestRemovalGates(unittest.TestCase):
    def test_every_removal_prefix_stays_inside_the_caps(self) -> None:
        for span in (5, 6):
            case = _solve("inchworm", span)
            plan = case["plan"]
            world = case["world"].copy()
            factory = case["factory"]
            policy = case["policy"]
            sag = case["sag"]
            depot = case["depot"]
            scaffold = set(plan.scaffold)
            geometry = factory(world)
            for task in plan.place:
                world.place_voxel(task.cell, scaffold=task.cell in scaffold)
            self.assertTrue(world.complete)
            for task in plan.remove:
                self.assertEqual(world.occupancy[task.cell], SCAFFOLD)
                self.assertIn(task.cell, geometry.reach_cells(task.approach))
                self.assertTrue(geometry.is_footing(task.approach))
                self.assertIsNotNone(bfs_path(geometry, depot, {task.approach}))
                world.remove_voxel(task.cell)
                cells = occupied_voxels(world)
                self.assertTrue(
                    is_stable(cells, policy),
                    f"span {span} removal of {task.cell} broke the cantilever gate",
                )
                self.assertTrue(
                    sag_within(cells, policy, sag),
                    f"span {span} removal of {task.cell} broke the sag cap",
                )
                self.assertTrue(geometry.is_footing(task.approach))
                self.assertIsNotNone(
                    bfs_path(geometry, task.approach, {depot}),
                    f"span {span} removal of {task.cell} stranded the stance",
                )
            self.assertTrue(world.complete)
            self.assertFalse(any(world.occupancy[cell] == SCAFFOLD for cell in scaffold))


class TestNoUnnecessaryScaffold(unittest.TestCase):
    def test_crane_bridge_needs_none(self) -> None:
        case = _solve("crane", 5)
        self.assertIsNotNone(case["plain"])
        plan = case["plan"]
        self.assertIsNotNone(plan)
        self.assertEqual(plan.scaffold_count, 0)
        self.assertEqual(
            [task.cell for task in plan.place],
            [task.cell for task in case["plain"]],
        )
        self.assertEqual(plan.remove, ())

    def test_wall_needs_none(self) -> None:
        case = _solve("wall")
        self.assertIsNotNone(case["plain"])
        plan = case["plan"]
        self.assertIsNotNone(plan)
        self.assertEqual(plan.scaffold_count, 0)
        self.assertEqual(
            [task.cell for task in plan.place],
            [task.cell for task in case["plain"]],
        )


class TestScaffoldDoesNotRescueAnIllegalFinish(unittest.TestCase):
    def test_tighter_cantilever_returns_none(self) -> None:
        case = _solve("inchworm", 5)
        tighter = case["policy"]._replace(max_cantilever=case["policy"].max_cantilever - 1)
        self.assertIsNone(
            plan_with_scaffold(
                case["world"],
                case["depot"],
                case["factory"],
                tighter,
                case["sag"],
                max_nodes=1000,
            )
        )

    def test_tighter_sag_cap_returns_none(self) -> None:
        case = _solve("inchworm", 5)
        tighter = SagCap(
            max_sag=case["sag"].max_sag - 1.0,
            k=1.0,
            mass=1.0,
            g=1.0,
        )
        self.assertIsNone(
            plan_with_scaffold(
                case["world"],
                case["depot"],
                case["factory"],
                case["policy"],
                tighter,
                max_nodes=1000,
            )
        )


class TestWalkOnScaffold(unittest.TestCase):
    def test_neighbor_becomes_a_footing(self) -> None:
        world, depot = build_bridge(5)
        yard = near_yard(world, depot)
        factory = geometry_factory(INCHWORM, yard)
        geometry = factory(world)
        support = (1, 2, 4)
        beside = (1, 1, 4)
        above = (2, 2, 4)
        self.assertNotIn(beside, yard)
        self.assertFalse(geometry.is_footing(beside))
        self.assertFalse(geometry.is_footing(above))
        world.place_voxel(support, scaffold=True)
        self.assertEqual(world.occupancy[support], SCAFFOLD)
        self.assertTrue(geometry.is_footing(beside))
        self.assertTrue(geometry.is_footing(above))
        self.assertIn("+", render_ascii3d(world))

    def test_inchworm_grips_scaffold(self) -> None:
        world, _depot = build_bridge(5)
        left, right = (1, 2, 4), (1, 2, 5)
        world.place_voxel(left, scaffold=True)
        world.place_voxel(right, scaffold=True)
        geometry = OnLatticeGeometry(world, GAIT)
        gripped = [face for face in geometry._resting_faces() if face[0] in (left, right)]
        self.assertTrue(gripped)
        self.assertTrue(entry_stances(geometry, (left, right)))


class TestSwarmRemovesScaffold(unittest.TestCase):
    def test_one_inchworm_finishes_with_none_left(self) -> None:
        case = _solve("inchworm", 5)
        plan = case["plan"]
        world = case["world"].copy()
        start = case["start_pool"][0]
        swarm = Swarm(
            world,
            case["depot"],
            [start],
            defect_p=0.0,
            inspect_enabled=True,
            geometry_factory=case["factory"],
            stability=case["policy"],
            sag=case["sag"],
            operations=plan.operations(),
            scaffold_cells=plan.scaffold,
        )
        while not swarm.done and swarm.tick_count < 4_000:
            swarm.tick()
        self.assertTrue(swarm.done, f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}")
        self.assertTrue(world.complete)
        self.assertFalse(any(world.occupancy[cell] == SCAFFOLD for cell in plan.scaffold))


def occupied_voxels_blueprint(world) -> list:
    return [tuple(int(v) for v in raw) for raw in zip(*world.blueprint.nonzero())]


if __name__ == "__main__":
    unittest.main()
