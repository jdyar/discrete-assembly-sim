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

import random
import unittest

from main import DEPOT, build_world
from sim.choreographer import plan_task
from sim.geometry import SquareLattice2D
from sim.geometry3d import CubicLattice3D
from sim.planner import (
    SearchBudgetExceeded,
    _approaches_for,
    plan_build_order,
    stability_precheck,
    validate_plan,
)
from sim.reservations import ReservationTable
from sim.stability import (
    StabilityPolicy,
    assess_stability,
    axial_tower_compression,
    estimate_sag,
    is_stable,
    necessary_stability_precheck,
    occupied_voxels,
    overhang_lower_bounds,
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


def _face_neighbors(cell):
    coords = list(cell)
    for axis in range(len(coords)):
        for delta in (-1, 1):
            coords[axis] += delta
            yield tuple(coords)
            coords[axis] -= delta


def _greedy_stable_order(cells, policy):
    """Prefix-stable order, or None if the finished set itself fails.

    A return of None on a set that passes :func:`is_stable` would mean
    the overhang lower bound is not a sufficient stability condition.
    The precheck relies on that sufficiency only as an explanation; the
    soundness test is the one-directional check against the planner.
    """
    if not is_stable(cells, policy):
        return None
    placed: set = set()
    remaining = set(cells)
    order = []
    while remaining:
        nxt = None
        for cell in sorted(remaining):
            if is_stable(placed | {cell}, policy):
                nxt = cell
                break
        if nxt is None:
            return None
        remaining.remove(nxt)
        placed.add(nxt)
        order.append(nxt)
    return order


class TestNecessaryPrecheck(unittest.TestCase):
    """The finished-structure screen is a proof, not a search heuristic.

    Overhang in the full set is a lower bound on overhang in every
    subset. Rejecting on that bound cannot throw away a plan whose
    finished prefix passed ``is_stable``.
    """

    def test_lower_bound_is_monotone_under_deletion(self) -> None:
        rng = random.Random(0)
        policy = StabilityPolicy(0, 1, ground_plane=0, max_cantilever=1)
        for _trial in range(40):
            cells = []
            n = 1 + rng.randrange(8)
            while len(cells) < n:
                cell = (1 + rng.randrange(3), rng.randrange(4), rng.randrange(4))
                if cell not in cells:
                    cells.append(cell)
            bounds = overhang_lower_bounds(cells, policy)
            self.assertEqual(set(bounds), set(cells))
            for _subset in range(6):
                chosen = [c for c in cells if rng.randrange(2)]
                if not chosen:
                    continue
                sub_bounds = overhang_lower_bounds(chosen, policy)
                for cell in chosen:
                    full = bounds[cell]
                    part = sub_bounds[cell]
                    if full is None:
                        self.assertIsNone(part)
                    elif part is None:
                        continue
                    else:
                        self.assertGreaterEqual(part, full)

    def test_floating_voxel_has_no_finite_lower_bound(self) -> None:
        # Nothing under it and nothing beside it. Limit does not matter:
        # there is no column to be a finite number of steps from.
        cells = [(3, 1, 1)]
        policy = StabilityPolicy(0, 1, ground_plane=0, max_cantilever=10)
        bounds = overhang_lower_bounds(cells, policy)
        self.assertIsNone(bounds[(3, 1, 1)])
        screen = necessary_stability_precheck(cells, policy)
        self.assertTrue(screen.rejected)
        self.assertEqual(screen.cell, (3, 1, 1))
        self.assertIsNone(screen.lower_bound)
        self.assertIn("no finite overhang lower bound", screen.reason)

    def test_span_lower_bound_rejects_only_past_the_limit(self) -> None:
        # Column on the plate, then two steps out at the next level. The
        # first course of an infinite ground plane would be planted, so
        # the arm has to sit above that course. Tip overhang is 2.
        # Limit 1 is a proof; limit 2 is not a rejection.
        cells = [(1, 0, 0), (2, 0, 0), (2, 0, 1), (2, 0, 2)]
        tight = StabilityPolicy(0, 1, ground_plane=0, max_cantilever=1)
        loose = StabilityPolicy(0, 1, ground_plane=0, max_cantilever=2)
        screen = necessary_stability_precheck(cells, tight)
        self.assertTrue(screen.rejected)
        self.assertEqual(screen.cell, (2, 0, 2))
        self.assertEqual(screen.lower_bound, 2)
        self.assertIn("overhang lower bound 2", screen.reason)
        self.assertIn("max_cantilever 1", screen.reason)
        allowed = necessary_stability_precheck(cells, loose)
        self.assertFalse(allowed.rejected)
        self.assertEqual(allowed.lower_bound, 2)
        self.assertIsNotNone(_greedy_stable_order(cells, loose))
        self.assertIsNone(_greedy_stable_order(cells, tight))

    def test_hollow_roof_center_bound_is_one(self) -> None:
        # 3×3 footprint, hollow center on level 1, full roof on level 2.
        # Same combinatorics as the campaign box, on a smaller index set.
        cells = [
            (level, row, col)
            for level in (1, 2)
            for row in range(3)
            for col in range(3)
            if not (level == 1 and row == 1 and col == 1)
        ]
        self.assertEqual(len(cells), 17)
        policy0 = StabilityPolicy(0, 1, ground_plane=0, max_cantilever=0)
        bounds = overhang_lower_bounds(cells, policy0)
        self.assertEqual(bounds[(2, 1, 1)], 1)
        self.assertTrue(all(v == 0 for c, v in bounds.items() if c != (2, 1, 1)))
        screen = necessary_stability_precheck(cells, policy0)
        self.assertTrue(screen.rejected)
        self.assertEqual(screen.cell, (2, 1, 1))
        self.assertEqual(screen.lower_bound, 1)
        for limit in (1, 2):
            policy = StabilityPolicy(0, 1, ground_plane=0, max_cantilever=limit)
            passed = necessary_stability_precheck(cells, policy)
            self.assertFalse(passed.rejected, passed.reason)
            self.assertEqual(passed.lower_bound, 1)
            self.assertIsNotNone(_greedy_stable_order(cells, policy))

    def test_gate_off_does_not_reject_a_floating_blueprint(self) -> None:
        world = World(rows=7, cols=8)
        world.blueprint[4, 3] = True
        depot = (5, 0)
        self.assertFalse(stability_precheck(world, None).rejected)
        self.assertEqual(stability_precheck(world, None).reason, "stability gate is off")
        plan = plan_build_order(world, depot)
        self.assertIsNotNone(plan)
        policy = SquareLattice2D(world).stability_policy(max_cantilever=10)
        screen = stability_precheck(world, policy)
        self.assertTrue(screen.rejected)
        self.assertIsNone(screen.lower_bound)
        self.assertIsNone(plan_build_order(world, depot, stability=policy, max_nodes=1))

    def test_empty_remainder_is_still_an_empty_plan(self) -> None:
        # Nothing left to place. The replan contract stays [], even if a
        # direct precheck of the occupancy would complain.
        world = World(rows=7, cols=8)
        world.place_voxel((4, 3))
        policy = SquareLattice2D(world).stability_policy(max_cantilever=0)
        self.assertTrue(stability_precheck(world, policy).rejected)
        self.assertEqual(plan_build_order(world, (5, 0), stability=policy), [])

    def test_2d_cantilever_screen_matches_the_search(self) -> None:
        world, depot = _cantilever_world()
        for limit, feasible in ((0, False), (1, False), (2, True)):
            policy = SquareLattice2D(world).stability_policy(max_cantilever=limit)
            screen = stability_precheck(world, policy)
            self.assertEqual(screen.rejected, not feasible, screen.reason)
            screened = plan_build_order(world, depot, stability=policy, max_nodes=1000)
            raw = plan_build_order(
                world, depot, stability=policy, max_nodes=1000, precheck=False
            )
            if feasible:
                self.assertIsNotNone(screened)
                self.assertIsNotNone(raw)
                self.assertFalse(screen.rejected)
            else:
                self.assertIsNone(screened)
                self.assertIsNone(raw)
                self.assertTrue(screen.rejected)
            self.assertEqual(world.built_count, 0)

    def test_small_fuzz_never_rejects_a_blueprint_the_search_can_build(self) -> None:
        saw_plan = False
        saw_rejection = False
        for dim, cases in _fuzz_cases():
            for seed, cells, limit, world, depot, factory, policy in cases:
                with self.subTest(dim=dim, seed=seed, n=len(cells), limit=limit):
                    screen = stability_precheck(world, policy)
                    final = [
                        tuple(int(v) for v in nd)
                        for nd in zip(*world.blueprint.nonzero())
                    ]
                    self.assertEqual(screen.rejected, not is_stable(final, policy))
                    greedy = _greedy_stable_order(final, policy)
                    if screen.rejected:
                        self.assertIsNone(greedy)
                    else:
                        self.assertIsNotNone(greedy)
                    try:
                        raw = plan_build_order(
                            world,
                            depot,
                            factory,
                            max_nodes=8_000,
                            stability=policy,
                            precheck=False,
                        )
                    except SearchBudgetExceeded as exc:
                        self.fail(f"unscreened search did not finish: {exc}")
                    screened = plan_build_order(
                        world,
                        depot,
                        factory,
                        max_nodes=8_000,
                        stability=policy,
                        precheck=True,
                    )
                    if not screen.rejected:
                        self.assertEqual(
                            None if screened is None else [task.cell for task in screened],
                            None if raw is None else [task.cell for task in raw],
                        )
                    if raw is not None:
                        saw_plan = True
                        self.assertIsNotNone(screened)
                        self.assertFalse(screen.rejected, screen.reason)
                        ok, why = validate_plan(
                            world, screened, depot, factory, stability=policy
                        )
                        self.assertTrue(ok, why)
                    if screen.rejected:
                        saw_rejection = True
                        self.assertIsNone(raw)
                        self.assertIsNone(screened)
                        self.assertTrue(
                            screen.lower_bound is None or screen.lower_bound > limit
                        )
                    # A screened rejection must not depend on the budget.
                    if screen.rejected:
                        self.assertIsNone(
                            plan_build_order(
                                world,
                                depot,
                                factory,
                                max_nodes=1,
                                stability=policy,
                            )
                        )
        self.assertTrue(saw_plan)
        self.assertTrue(saw_rejection)


def _in_box(cell, bounds) -> bool:
    return all(lo <= value < hi for value, (lo, hi) in zip(cell, bounds))


def _grow(rng, seed_cell, n, bounds):
    cells = [seed_cell]
    guard = 0
    while len(cells) < n and guard < 1000:
        guard += 1
        host = cells[rng.randrange(len(cells))]
        options = [
            nb
            for nb in _face_neighbors(host)
            if _in_box(nb, bounds) and nb not in cells
        ]
        if not options:
            continue
        cells.append(options[rng.randrange(len(options))])
    return cells


def _fuzz_cases():
    """Small connected blueprints in 2D and 3D, several cantilever limits.

    Sizes stay at or below 6 voxels so the unscreened search exhausts
    (the dead-end memo visits each subset at most once). ``randrange``
    draws the cells; ``random.sample`` is avoided because it is not
    stable across the Python versions this repo tests.
    """
    out = []
    rng = random.Random(20261018)
    cases3 = []
    for seed in range(8):
        for n in (4, 5, 6):
            cells = _grow(rng, (1, 1, 2), n, ((1, 4), (1, 4), (2, 6)))
            for limit in (0, 1, 2):
                world = World3D(5, 5, 7)
                for cell in cells:
                    world.blueprint[cell] = True
                policy = CubicLattice3D(world).stability_policy(max_cantilever=limit)
                cases3.append((seed, cells, limit, world, (1, 0, 0), CubicLattice3D, policy))
    out.append((3, cases3))
    cases2 = []
    for seed in range(6):
        for n in (4, 5, 6):
            cells = _grow(rng, (6, 2), n, ((3, 7), (2, 7)))
            for limit in (0, 1, 2):
                world = World(rows=8, cols=8)
                for cell in cells:
                    world.blueprint[cell] = True
                policy = SquareLattice2D(world).stability_policy(max_cantilever=limit)
                cases2.append((seed, cells, limit, world, (6, 0), SquareLattice2D, policy))
    out.append((2, cases2))
    return out


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
