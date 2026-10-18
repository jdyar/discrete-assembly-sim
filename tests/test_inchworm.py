# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""On-lattice inchworm gait, robot point loads, and planner integration.

The crane geometry is the default. These tests opt in. The closed form
they lock is: a point load P at the tip of a shear chain of n free
voxels adds P*n/k_shear at the tip, on top of self-weight.
"""

from __future__ import annotations

import unittest

from sim.choreographer import plan_task
from sim.geometry3d import CubicLattice3D
from sim.inchworm import (
    VIEW_FOOT_HALF_THICK,
    VIEW_FOOT_OUTSET,
    VIEW_VOXEL_HALF,
    InchwormConfig,
    OnLatticeGeometry,
    center_chebyshev,
    entry_stances,
    make_stance,
)
from sim.planner import plan_build_order, validate_plan
from sim.reservations import ReservationTable
from sim.stability import (
    estimate_sag,
    path_chain_deflection_at,
    point_load_deflection_at,
    robot_stance_ok,
    shear_cantilever_tip_sag,
    split_foot_loads,
)
from sim.swarm import Swarm
from sim.texgraph import TENode, TimeExpandedGraph
from sim.world3d import World3D


def _up(cell):
    return (cell, (1, 0, 0))


def _seeded_line(cols, *, blueprint_from: int | None = None) -> tuple[World3D, OnLatticeGeometry]:
    """Level-1 voxels along row 1, columns ``cols``. Optional blueprint tail."""
    world = World3D(levels=4, rows=3, cols=8)
    for col in cols:
        world.place_voxel((1, 1, col))
    if blueprint_from is not None:
        for col in range(blueprint_from, max(cols) + 1):
            if (1, 1, col) not in [(1, 1, c) for c in cols]:
                world.blueprint[1, 1, col] = True
    geom = OnLatticeGeometry(world, InchwormConfig())
    return world, geom


class TestGaitValidity(unittest.TestCase):
    def test_both_feet_must_be_on_exposed_voxel_faces(self) -> None:
        world, geom = _seeded_line([1, 2])
        tops = entry_stances(geom, [(1, 1, 1), (1, 1, 2)])
        self.assertTrue(tops)
        for stance in tops:
            self.assertTrue(geom.is_footing(stance))
            for cell in geom.foot_cells(stance):
                self.assertIn(world.occupancy[cell], (2,))  # VOXEL
        # An empty cell is not a face to grip, and a buried bottom face
        # (against the ground plane) is not a surface.
        bottom = ((1, 1, 1), (-1, 0, 0))
        self.assertFalse(geom._exposed(*bottom))
        ghost = make_stance(_up((1, 1, 1)), _up((1, 1, 5)))
        self.assertFalse(geom.is_footing(ghost))

    def test_step_moves_exactly_one_foot_and_respects_stride_and_span(self) -> None:
        world, geom = _seeded_line([0, 1, 2, 3])
        start = make_stance(_up((1, 1, 0)), _up((1, 1, 1)))
        self.assertTrue(geom.is_footing(start))
        moves = list(geom.neighbors(start))
        self.assertTrue(moves)
        for nxt in moves:
            step = geom.classify_step(start, nxt)
            self.assertIsNotNone(step)
            self.assertEqual(step.attach_states[0], "both")
            self.assertEqual(step.attach_states[-1], "both")
            self.assertIn("pivot_only", step.attach_states)
            self.assertLessEqual(len(step.path) - 1, geom.config.stride)
            faces = nxt[1], nxt[2]
            self.assertLessEqual(center_chebyshev(*faces), geom.config.span + 1e-9)
            # The pivot face is one of the feet before and after.
            self.assertIn(step.pivot, (start[1], start[2]))
            self.assertIn(step.pivot, faces)
        # Opening the body: front foot steps from col 1 to col 2, pivot stays on 0.
        opened = make_stance(_up((1, 1, 0)), _up((1, 1, 2)))
        self.assertIn(opened, moves)
        # span 1 cannot hold feet two pitches apart.
        tight = OnLatticeGeometry(world, InchwormConfig(span=1.0))
        self.assertNotIn(opened, list(tight.neighbors(start)))
        self.assertFalse(tight.is_footing(opened))
        # stride 1 cannot jump the swing foot two edges in one tick.
        far = make_stance(_up((1, 1, 0)), _up((1, 1, 3)))
        self.assertIsNone(geom.classify_step(start, far))

    def test_flat_convex_and_concave_are_separate_switches(self) -> None:
        world = World3D(levels=4, rows=3, cols=4)
        world.place_voxel((1, 1, 1))
        world.place_voxel((2, 1, 2))  # up and +col: concave corner with the first top
        sides = InchwormConfig(allow_sides=True, allow_flat=True, allow_convex=True, allow_concave=True)
        geom = OnLatticeGeometry(world, sides)
        top = ((1, 1, 1), (1, 0, 0))
        side = ((1, 1, 1), (0, 0, 1))
        kinds = {face: kind for face, kind in geom.surface_neighbors(top)}
        self.assertEqual(kinds[side], "convex")
        # Concave: top of A and the -col face of B = A + up - (-col).
        concave_face = ((2, 1, 2), (0, 0, -1))
        self.assertEqual(kinds[concave_face], "concave")

        no_convex = OnLatticeGeometry(
            world, InchwormConfig(allow_sides=True, allow_convex=False, allow_concave=True)
        )
        kinds = {face: kind for face, kind in no_convex.surface_neighbors(top)}
        self.assertNotIn(side, kinds)
        self.assertIn(concave_face, kinds)

        no_concave = OnLatticeGeometry(
            world, InchwormConfig(allow_sides=True, allow_convex=True, allow_concave=False)
        )
        kinds = {face: kind for face, kind in no_concave.surface_neighbors(top)}
        self.assertIn(side, kinds)
        self.assertNotIn(concave_face, kinds)

        # Default config does not rest on side faces or undersides.
        default = OnLatticeGeometry(world, InchwormConfig())
        self.assertTrue(default._resting((1, 0, 0)))
        self.assertFalse(default._resting((0, 1, 0)))
        self.assertFalse(default._resting((-1, 0, 0)))
        # Inverted faces are a switch, not a default.
        under = OnLatticeGeometry(
            world, InchwormConfig(allow_inverted=True, allow_top=False, allow_sides=False)
        )
        self.assertTrue(under._resting((-1, 0, 0)))

    def test_shared_foot_is_a_reservation_conflict(self) -> None:
        world, geom = _seeded_line([0, 1, 2])
        a = make_stance(_up((1, 1, 0)), _up((1, 1, 1)))
        b = make_stance(_up((1, 1, 1)), _up((1, 1, 2)))
        self.assertTrue(set(geom.reservation_keys(a)) & set(geom.reservation_keys(b)))
        table = ReservationTable()
        graph = TimeExpandedGraph(geom, table, owner="r1", static_obstacles=frozenset({a}))
        # r2's pose shares a foot with the parked robot, so it is not a successor.
        succ = {state.node for state in graph.successors(TENode(b, 0))}
        self.assertNotIn(a, succ)
        table.reserve_path("r2", [a], t0=1)
        for key in geom.reservation_keys(a):
            if key != a:
                table.reserve_hold("r2", key, 1, 1)
        blocked = TimeExpandedGraph(geom, table, owner="r1")
        succ = {state.node for state in blocked.successors(TENode(b, 0))}
        self.assertNotIn(a, succ)

    def test_placement_is_against_a_face_and_does_not_bury_a_foot(self) -> None:
        _world, geom = _seeded_line([1, 2])
        stance = make_stance(_up((1, 1, 1)), _up((1, 1, 2)))
        cells = set(geom.reach_cells(stance))
        self.assertIn((1, 1, 3), cells)
        # The cell directly outside a top foot is where the gripper is.
        self.assertNotIn((2, 1, 1), cells)
        self.assertNotIn((2, 1, 2), cells)
        # A cell that only touches at a corner is not "against a face".
        self.assertNotIn((1, 0, 0), cells)
        # Symmetry: the stance is an approach for the cell it can place.
        self.assertIn(stance, geom.reach_cells((1, 1, 3)))

    def test_viewer_foot_offset_is_outside_the_voxel(self) -> None:
        # The replay viewer places the foot center one outset along the
        # outward normal. The solid is a 0.96 cube. The foot's inner
        # face must clear that solid, and the center must stay within
        # a fraction of a pitch of the face (not floating above it).
        inner = VIEW_FOOT_OUTSET - VIEW_FOOT_HALF_THICK
        self.assertGreater(inner, VIEW_VOXEL_HALF)
        self.assertLess(VIEW_FOOT_OUTSET, VIEW_VOXEL_HALF + 0.2)


class TestPointLoads(unittest.TestCase):
    def _chain(self, n_free: int):
        # Root at (1, 0, 0) planted on a finite base, arm along +col.
        cells = [(1, 0, i) for i in range(n_free + 1)]
        policy = dict(up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),))
        return cells, policy

    def test_tip_point_load_adds_p_times_n_over_k(self) -> None:
        n = 4
        k = 2.5
        p = 3.0
        mass, g = 1.5, 2.0
        cells, frame = self._chain(n)
        bare = estimate_sag(cells, k_shear=k, k_axial=k, mass=mass, g=g, **frame)
        loaded = estimate_sag(
            cells, k_shear=k, k_axial=k, mass=mass, g=g, point_loads=[((1, 0, n), p)], **frame
        )
        tip = (1, 0, n)
        self.assertAlmostEqual(
            loaded.displacements[tip][0] * -1 - bare.displacements[tip][0] * -1,
            point_load_deflection_at(n, n, load=p, stiffness=k),
        )
        self.assertAlmostEqual(
            point_load_deflection_at(n, n, load=p, stiffness=k), p * n / k
        )
        # The whole extra profile is P*i/k, and self-weight still matches
        # the shear chain. Up is +level, so sag is the negated level component.
        for i in range(1, n + 1):
            cell = (1, 0, i)
            extra = -loaded.displacements[cell][0] - (-bare.displacements[cell][0])
            self.assertAlmostEqual(extra, point_load_deflection_at(i, n, load=p, stiffness=k))
            self.assertAlmostEqual(
                -bare.displacements[cell][0],
                path_chain_deflection_at(i, n, weight=mass * g, stiffness=k),
            )
        self.assertAlmostEqual(
            -bare.displacements[tip][0],
            shear_cantilever_tip_sag(n, mass=mass, g=g, k_shear=k),
        )

    def test_load_split_across_feet_and_a_planted_foot_does_not_sag(self) -> None:
        loads = split_foot_loads([(1, 0, 4), (1, 0, 3)], robot_mass=10.0, carried_mass=1.0, g=1.0)
        self.assertEqual(len(loads), 2)
        self.assertAlmostEqual(sum(force for _cell, force in loads), 11.0)
        cells, frame = self._chain(4)
        # Full load on the planted root: the support reacts it.
        bare = estimate_sag(cells, k=1.0, **frame)
        rooted = estimate_sag(cells, k=1.0, point_loads=[((1, 0, 0), 11.0)], **frame)
        self.assertAlmostEqual(rooted.worst_sag, bare.worst_sag)
        # A foot past the cantilever limit is refused even before sag.
        ok, why = robot_stance_ok(
            cells,
            [(1, 0, 4)],
            max_cantilever=2,
            up_axis=frame["up_axis"],
            up_sign=frame["up_sign"],
            ground_plane=frame["ground_plane"],
            base=frame["base"],
        )
        self.assertFalse(ok)
        self.assertIn("overhang", why)


class TestPlannerIntegration(unittest.TestCase):
    def test_flat_line_builds_from_a_seed_and_the_crane_default_is_untouched(self) -> None:
        world = World3D(levels=3, rows=3, cols=8)
        world.place_voxel((1, 1, 1))
        world.place_voxel((1, 1, 2))
        world.blueprint[1, 1, 3] = True
        world.blueprint[1, 1, 4] = True
        config = InchwormConfig()
        factory = lambda w, config=config: OnLatticeGeometry(w, config)  # noqa: E731
        geom = factory(world)
        depot = make_stance(_up((1, 1, 1)), _up((1, 1, 2)))
        self.assertTrue(geom.is_footing(depot))
        plan = plan_build_order(world, depot, factory, stability=geom.stability_policy(0))
        self.assertIsNotNone(plan)
        self.assertEqual([task.cell for task in plan], [(1, 1, 3), (1, 1, 4)])
        ok, why = validate_plan(world, plan, depot, factory, stability=geom.stability_policy(0))
        self.assertTrue(ok, why)
        swarm = Swarm(
            world, depot, starts=[depot], geometry_factory=factory,
            stability=geom.stability_policy(0), inspect_enabled=False,
        )
        for _tick in range(400):
            if swarm.settled:
                break
            swarm.tick()
        self.assertTrue(world.complete, f"ticks={swarm.tick_count} kickbacks={swarm.kickbacks}")

        # Same blueprint, crane geometry, no policy: still plans. The
        # inchworm depot is not a crane cell; use a ground footing.
        crane_world = World3D(levels=3, rows=3, cols=8)
        crane_world.blueprint[1, 1, 3] = True
        crane_world.blueprint[1, 1, 4] = True
        crane = plan_build_order(crane_world, (1, 1, 0), CubicLattice3D)
        self.assertIsNotNone(crane)
        self.assertTrue(all(len(task.approach) == 3 and isinstance(task.approach[0], int) for task in crane))

    def test_sag_cap_rejects_a_heavy_robot_on_the_arm(self) -> None:
        # Root column plus a depot voxel beside it. The arm is level 2,
        # so it is a shear chain. Sides are on: the robot has to step
        # off the root's top before it can place above that top.
        world = World3D(levels=4, rows=3, cols=8)
        for col in (1, 2):
            world.place_voxel((1, 1, col))
        for col in (2, 3, 4):
            world.blueprint[2, 1, col] = True
        config = InchwormConfig(allow_sides=True, allow_convex=True, allow_concave=True)
        # Loose cantilever, sag cap below the loaded tip. Voxel mass 1,
        # robot mass 10, carried mass 1. n=3 free. Self-weight tip is
        # 6; a tip load of 11 adds 11*3 = 33. The cap of 10 is under that.
        frame_policy = OnLatticeGeometry(world, config).stability_policy(3)
        loose = frame_policy._replace(max_cantilever=3, max_sag=None, robot_mass=10.0, carried_mass=1.0)
        capped = loose._replace(max_sag=10.0)
        factory_loose = lambda w, config=config, policy=loose: OnLatticeGeometry(w, config, policy)  # noqa: E731
        factory_capped = lambda w, config=config, policy=capped: OnLatticeGeometry(w, config, policy)  # noqa: E731
        geom = OnLatticeGeometry(world, config, loose)
        choices = entry_stances(geom, [(1, 1, 1), (1, 1, 2)])
        self.assertTrue(choices)
        depot = choices[0]
        plan = plan_build_order(world, depot, factory_loose, stability=loose)
        self.assertIsNotNone(plan, "uncapped search should build the arm")
        refused = plan_build_order(world, depot, factory_capped, stability=capped)
        self.assertIsNone(refused)
        # The cap is enforced at the stance, not only as a missing plan:
        # standing on the tip with the load is over 10.
        cells = [(1, 1, 1), (1, 1, 2), (2, 1, 2), (2, 1, 3), (2, 1, 4)]
        ok, why = robot_stance_ok(
            cells, [(2, 1, 4), (2, 1, 3)], capped, carrying=True, k=1.0, mass=1.0, g=1.0
        )
        self.assertFalse(ok, why)
        self.assertIn("sag", why)

    def test_choreographer_gate_sees_the_same_foot_load(self) -> None:
        world = World3D(levels=3, rows=3, cols=6)
        world.place_voxel((1, 1, 1))
        world.place_voxel((1, 1, 2))
        world.blueprint[1, 1, 3] = True
        config = InchwormConfig()
        policy = OnLatticeGeometry(world, config).stability_policy(0)
        geom = OnLatticeGeometry(world, config, policy)
        depot = make_stance(_up((1, 1, 1)), _up((1, 1, 2)))
        table = ReservationTable()
        task = plan_task(
            "r0", depot, 0, (1, 1, 3),
            [s for s in geom.reach_cells((1, 1, 3)) if geom.is_footing(s)],
            geom, table, depot, anchors=[], inspect_enabled=False,
            stability=policy,
        )
        self.assertIsNotNone(task)
        self.assertEqual(task.target, (1, 1, 3))


if __name__ == "__main__":
    unittest.main()
