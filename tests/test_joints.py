# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Face-joint strength, the utilization gate, and the 3×3×3 derivation.

Forces at ``k = mass = g = 1`` are in units of ``mass·g``. The ARMADAS
face compression is in newtons and is not one of those forces.
"""

from __future__ import annotations

import unittest

from sim.geometry3d import CubicLattice3D, MotionModel
from sim.metrics import RunLog, StructureLog
from sim.planner import plan_build_order, validate_plan
from sim.scaffold import CRANE, geometry_factory
from sim.stability import (
    JointCap,
    JointLoad,
    JointStrength,
    StabilityPolicy,
    armadas_face_compression,
    armadas_joint_strength,
    assess_joints,
    estimate_sag,
    joint_loads,
    joint_strains,
    joint_utilizations,
    joints_within,
    path_chain_joint_force,
    robot_stance_ok,
    split_foot_loads,
)
from sim.world3d import World3D

_CLAMP = dict(up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),))
_FRAME = dict(up_axis=0, up_sign=1, ground_plane=0)


def _bar(n_free: int, *, horizontal: bool) -> list[tuple[int, int, int]]:
    cells = [(1, 0, 0)]
    if horizontal:
        cells += [(1, 0, i) for i in range(1, n_free + 1)]
    else:
        cells += [(1 + i, 0, 0) for i in range(1, n_free + 1)]
    return cells


def _loads(cells, **kwargs):
    estimate = estimate_sag(cells, **kwargs)
    return joint_loads(
        estimate,
        k=kwargs.get("k", 1.0),
        k_axial=kwargs.get("k_axial"),
        k_shear=kwargs.get("k_shear"),
    )


class TestChainForces(unittest.TestCase):
    def test_shear_chain_matches_the_outboard_weight(self) -> None:
        mass, g, k = 2.0, 3.0, 4.0
        weight = mass * g
        for n in (1, 3, 6):
            cells = _bar(n, horizontal=True)
            loads = _loads(cells, **_CLAMP, k_shear=k, k_axial=50.0, mass=mass, g=g)
            by_outboard = {}
            for load in loads:
                # Horizontal joints run along +col. The outboard voxel's
                # column index is the chain index.
                if load.axis != 2:
                    continue
                index = max(load.a[2], load.b[2])
                by_outboard[index] = load
            self.assertEqual(set(by_outboard), set(range(1, n + 1)))
            for index, load in by_outboard.items():
                self.assertAlmostEqual(load.axial, 0.0, places=9)
                self.assertAlmostEqual(
                    load.shear, path_chain_joint_force(index, n, weight=weight)
                )
                self.assertIsNone(load.moment)

    def test_tower_compression_is_the_same_magnitude_and_negative(self) -> None:
        mass, g, k = 1.5, 2.0, 3.0
        weight = mass * g
        n = 4
        cells = _bar(n, horizontal=False)
        loads = _loads(cells, **_FRAME, k_axial=k, k_shear=80.0, mass=mass, g=g)
        vertical = [load for load in loads if load.axis == 0]
        self.assertEqual(len(vertical), n)
        for load in vertical:
            index = max(load.a[0], load.b[0]) - 1
            self.assertAlmostEqual(load.shear, 0.0, places=9)
            self.assertAlmostEqual(
                load.axial, -path_chain_joint_force(index, n, weight=weight)
            )

    def test_force_does_not_depend_on_k(self) -> None:
        cells = _bar(3, horizontal=True)
        soft = _loads(cells, **_CLAMP, k=1.0, mass=1.0, g=1.0)
        stiff = _loads(cells, **_CLAMP, k=250.0, mass=1.0, g=1.0)
        for left, right in zip(soft, stiff):
            self.assertEqual((left.a, left.b), (right.a, right.b))
            self.assertAlmostEqual(left.shear, right.shear, places=8)
            self.assertAlmostEqual(left.axial, right.axial, places=8)

    def test_tip_point_load_adds_p_to_every_spring(self) -> None:
        n = 3
        cells = _bar(n, horizontal=True)
        weight = 1.0
        added = 2.5
        loads = _loads(
            cells,
            **_CLAMP,
            k_shear=7.0,
            mass=weight,
            g=1.0,
            point_loads=[((1, 0, n), added)],
        )
        for load in loads:
            if load.axis != 2:
                continue
            index = max(load.a[2], load.b[2])
            self.assertAlmostEqual(
                load.shear,
                path_chain_joint_force(index, n, weight=weight) + added,
            )

    def test_strain_times_stiffness_is_the_force_at_unit_pitch(self) -> None:
        cells = _bar(2, horizontal=True)
        estimate = estimate_sag(cells, **_CLAMP, k_shear=4.0, k_axial=4.0, mass=2.0, g=3.0)
        strains = joint_strains(estimate, pitch=1.0)
        loads = joint_loads(estimate, k_shear=4.0, k_axial=4.0)
        self.assertEqual(len(strains), len(loads))
        for strain, load in zip(strains, loads):
            self.assertEqual((strain.a, strain.b), (load.a, load.b))
            self.assertAlmostEqual(load.axial, 4.0 * strain.axial)
            self.assertAlmostEqual(load.shear, 4.0 * strain.shear)


class TestUtilization(unittest.TestCase):
    def test_compression_and_tension_use_different_capacities(self) -> None:
        strength = JointStrength(axial_compression=10.0, axial_tension=4.0, shear=100.0)
        loads = (
            JointLoad((0,), (1,), 0, axial=-5.0, shear=0.0),
            JointLoad((1,), (2,), 0, axial=2.0, shear=0.0),
        )
        uses = joint_utilizations(loads, strength)
        self.assertAlmostEqual(uses[0].axial_utilization, 0.5)
        self.assertAlmostEqual(uses[1].axial_utilization, 0.5)
        self.assertAlmostEqual(uses[0].utilization, 0.5)

    def test_unset_component_is_not_checked(self) -> None:
        cells = _bar(4, horizontal=True)
        # Root shear is 4. A compression capacity of 0.1 does not see it.
        cap = JointCap(JointStrength(axial_compression=0.1), safety_factor=1.0)
        self.assertTrue(joints_within(cells, cap=cap, **_CLAMP))
        shear_cap = JointCap(JointStrength(shear=3.9), safety_factor=1.0)
        self.assertFalse(joints_within(cells, cap=shear_cap, **_CLAMP))
        report = assess_joints(cells, cap=JointCap(JointStrength(shear=4.0)), **_CLAMP)
        self.assertTrue(report.ok)
        self.assertAlmostEqual(report.worst_utilization, 1.0)

    def test_safety_factor_is_a_reduction(self) -> None:
        cells = _bar(2, horizontal=False)
        # Root compression is 2.
        full = JointCap(JointStrength(axial_compression=2.0), safety_factor=1.0)
        half = JointCap(JointStrength(axial_compression=2.0), safety_factor=0.5)
        self.assertTrue(joints_within(cells, cap=full, **_FRAME))
        self.assertFalse(joints_within(cells, cap=half, **_FRAME))
        with self.assertRaises(ValueError):
            joints_within(
                cells,
                cap=JointCap(JointStrength(axial_compression=2.0), safety_factor=1.1),
                **_FRAME,
            )
        with self.assertRaises(ValueError):
            JointCap(JointStrength(axial_compression=2.0), safety_factor=0.0)
            joints_within(
                cells,
                cap=JointCap(JointStrength(axial_compression=2.0), safety_factor=0.0),
                **_FRAME,
            )

    def test_moment_is_optional_and_absent_from_the_solve(self) -> None:
        loaded = JointLoad((0,), (1,), 0, axial=0.0, shear=0.0, moment=2.0)
        uses = joint_utilizations((loaded,), JointStrength(shear=10.0, moment=4.0))
        self.assertAlmostEqual(uses[0].moment_utilization, 0.5)
        self.assertAlmostEqual(uses[0].utilization, 0.5)
        cells = _bar(1, horizontal=True)
        loads = _loads(cells, **_CLAMP)
        self.assertTrue(all(load.moment is None for load in loads))
        # A moment capacity does not fire on a load that has no moment.
        cap = JointCap(JointStrength(shear=100.0, moment=0.01))
        self.assertTrue(joints_within(cells, cap=cap, **_CLAMP))

    def test_empty_passes_and_a_floating_voxel_fails(self) -> None:
        cap = JointCap(JointStrength(shear=1.0))
        self.assertTrue(joints_within((), cap=cap, **_CLAMP))
        self.assertFalse(joints_within([(1, 0, 1)], cap=cap, **_CLAMP))

    def test_strength_with_no_component_is_rejected(self) -> None:
        with self.assertRaises(ValueError):
            joints_within([(1, 0, 0)], cap=JointCap(JointStrength()), **_CLAMP)


class TestArmadasDerivation(unittest.TestCase):
    def test_published_numbers_and_the_even_share(self) -> None:
        got = armadas_face_compression()
        self.assertEqual(got.published_strength_status, "confirmed")
        self.assertEqual(got.published_strength_kpa, 11.38)
        self.assertEqual(got.published_strength_sd_kpa, 0.51)
        self.assertEqual(got.stiffness_mpa, 1.1129)
        self.assertEqual(got.stiffness_status, "confirmed")
        self.assertEqual(got.stiffness_sd_status, "unconfirmed")
        self.assertEqual(got.failure_load_lower_bound_n, 9000.0)
        self.assertIn("lower bound", got.failure_load_status)
        self.assertEqual(got.manuscript_strength_kpa, 11.17)
        self.assertNotEqual(got.manuscript_strength_kpa, got.published_strength_kpa)
        area = (3 * 0.3048) ** 2
        self.assertAlmostEqual(got.gross_area_m2, area)
        load = 11.38e3 * area
        self.assertAlmostEqual(got.implied_mean_load_n, load)
        self.assertGreater(load, 9000.0)
        self.assertAlmostEqual(got.implied_mean_load_n, 9515.1293568, places=6)
        self.assertEqual(got.joints_per_interface, 9)
        self.assertAlmostEqual(got.face_compression_n, load / 9)
        self.assertAlmostEqual(got.face_compression_n, 1057.2365952, places=6)
        self.assertAlmostEqual(got.face_compression_sd_n, 0.51e3 * area / 9)
        self.assertIsNone(got.shear_n)
        self.assertIsNone(got.moment_nm)
        self.assertEqual(got.fastener_tension_n, 797.3)
        self.assertNotAlmostEqual(got.face_compression_n, got.fastener_tension_n, delta=1.0)
        self.assertIn("not used", got.fastener_tension_status)
        strength = armadas_joint_strength()
        self.assertAlmostEqual(strength.axial_compression, got.face_compression_n)
        self.assertIsNone(strength.shear)
        self.assertIsNone(strength.axial_tension)
        self.assertIsNone(strength.moment)
        # A k = 1 cantilever force is a few mass·g, not this newton figure.
        self.assertGreater(strength.axial_compression, 100.0)


class TestPlannerGate(unittest.TestCase):
    def _tower_world(self) -> tuple[World3D, tuple, object]:
        world = World3D(6, 4, 6)
        depot = (1, 1, 1)
        for level in range(1, 4):
            world.blueprint[level, 1, 3] = True
        factory = geometry_factory(MotionModel(reach_radius=4, stride=1, climb=0))
        policy = CubicLattice3D(world).stability_policy(0)
        return world, depot, factory, policy

    def test_compression_cap_rejects_only_the_overloaded_tower(self) -> None:
        world, depot, factory, policy = self._tower_world()
        # Two free voxels. Root compression is 2 mass·g.
        loose = JointCap(JointStrength(axial_compression=2.0))
        tight = JointCap(JointStrength(axial_compression=1.5))
        plain = plan_build_order(world, depot, factory, stability=policy)
        allowed = plan_build_order(world, depot, factory, stability=policy, joints=loose)
        refused = plan_build_order(world, depot, factory, stability=policy, joints=tight)
        self.assertIsNotNone(plain)
        self.assertIsNotNone(allowed)
        self.assertEqual([task.cell for task in allowed], [task.cell for task in plain])
        self.assertIsNone(refused)
        ok, reason = validate_plan(
            world, allowed, depot, factory, stability=policy, joints=loose
        )
        self.assertTrue(ok, reason)
        bad, why = validate_plan(
            world, allowed, depot, factory, stability=policy, joints=tight
        )
        self.assertFalse(bad)
        self.assertIn("capacity", why)

    def test_shear_cap_does_not_reject_a_pure_tower(self) -> None:
        world, depot, factory, policy = self._tower_world()
        cap = JointCap(JointStrength(shear=0.1))
        self.assertIsNotNone(
            plan_build_order(world, depot, factory, stability=policy, joints=cap)
        )

    def test_crane_span4_joint_cap_does_not_add_scaffold(self) -> None:
        """A cap at the finished demand stays at 0 posts. Half rejects the deck.

        Scaffold is removed at the end, so a cap the finished bridge
        already fails cannot be rescued by more posts. The count goes
        from 0 to infeasible. It does not step up. k = mass = g = 1,
        so the capacities are mass·g, not the ARMADAS newton figure.
        """
        from experiments.stability_cost import build_bridge
        from sim.scaffold import caps_for_finished, plan_with_scaffold

        world, depot = build_bridge(4)
        factory = geometry_factory(CRANE)
        policy, sag = caps_for_finished(
            world, CubicLattice3D(world).stability_policy(0)
        )
        cells = tuple(
            (int(level), int(row), int(col))
            for level, row, col in zip(*world.blueprint.nonzero())
        )
        loads = joint_loads(estimate_sag(cells, policy, k=1.0, mass=1.0, g=1.0), k=1.0)
        compression = max((-load.axial for load in loads if load.axial < 0), default=0.0)
        shear = max((load.shear for load in loads), default=0.0)
        self.assertGreater(compression, 0.0)
        self.assertGreater(shear, 0.0)
        at_finished = JointCap(
            JointStrength(axial_compression=compression, shear=shear)
        )
        half = JointCap(
            JointStrength(axial_compression=compression * 0.5, shear=shear * 0.5)
        )
        sag_only = plan_with_scaffold(
            world, depot, factory, policy, sag, max_nodes=40_000
        )
        capped = plan_with_scaffold(
            world, depot, factory, policy, sag, max_nodes=40_000, joints=at_finished
        )
        refused = plan_with_scaffold(
            world, depot, factory, policy, sag, max_nodes=40_000, joints=half
        )
        self.assertIsNotNone(sag_only)
        self.assertEqual(sag_only.scaffold_count, 0)
        self.assertIsNotNone(capped)
        self.assertEqual(capped.scaffold_count, 0)
        self.assertIsNone(refused)

    def test_joint_cap_requires_a_frame(self) -> None:
        world, depot, factory, _policy = self._tower_world()
        with self.assertRaises(ValueError):
            plan_build_order(
                world,
                depot,
                factory,
                joints=JointCap(JointStrength(axial_compression=1.0)),
            )

    def test_foot_load_increases_the_joint_the_robot_stands_on(self) -> None:
        cells = _bar(3, horizontal=True)
        feet = [(1, 0, 3)]
        loads = split_foot_loads(feet, robot_mass=2.0, g=1.0)
        bare = assess_joints(cells, cap=JointCap(JointStrength(shear=100.0)), **_CLAMP)
        loaded = assess_joints(
            cells,
            cap=JointCap(JointStrength(shear=100.0)),
            point_loads=loads,
            **_CLAMP,
        )
        self.assertGreater(loaded.worst_utilization, bare.worst_utilization)
        policy = StabilityPolicy(
            up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),),
            max_cantilever=None, robot_mass=2.0,
        )
        # Root shear with the robot is 3 + 2 = 5.
        ok, _why = robot_stance_ok(
            cells, feet, policy, joints=JointCap(JointStrength(shear=5.0))
        )
        self.assertTrue(ok, _why)
        no, why = robot_stance_ok(
            cells, feet, policy, joints=JointCap(JointStrength(shear=4.9))
        )
        self.assertFalse(no)
        self.assertIn("capacity", why)


class TestStructureLogUtilization(unittest.TestCase):
    def test_utilization_is_omitted_until_a_strength_is_set(self) -> None:
        world = World3D(2, 1, 3)
        policy = StabilityPolicy(
            up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),), max_cantilever=None
        )
        plain = RunLog(structure=StructureLog(policy, k_shear=4.0, k_axial=4.0))
        plain.log_tick(0, world)
        world.place_voxel((1, 0, 0))
        plain.log_tick(1, world)
        world.place_voxel((1, 0, 1))
        plain.log_tick(2, world)
        joint = plain.frames[2]["structure"]["joints"][0]
        self.assertIn("axial_force", joint)
        self.assertNotIn("utilization", joint)
        self.assertNotIn("worst_utilization", plain.frames[2]["structure"])
        self.assertEqual(plain.meta["structure"]["force_units"], "mass*g")
        self.assertNotIn("joint_strength", plain.meta["structure"])

    def test_sample_utilization_matches_the_solve(self) -> None:
        world = World3D(2, 1, 3)
        policy = StabilityPolicy(
            up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),), max_cantilever=None
        )
        # Two free voxels once both are placed? Root is planted, one free.
        # Place root and one free voxel. Root shear equals one mass·g at k=1.
        strength = JointStrength(shear=2.0)
        log = RunLog(
            structure=StructureLog(policy, strength=strength, safety_factor=1.0)
        )
        log.log_tick(0, world)
        world.place_voxel((1, 0, 0))
        log.log_tick(1, world)
        world.place_voxel((1, 0, 1))
        log.log_tick(2, world)
        sample = log.frames[2]["structure"]
        self.assertIn("worst_utilization", sample)
        self.assertAlmostEqual(sample["worst_utilization"], 0.5)
        self.assertEqual(sample["safety_factor"], 1.0)
        joint = sample["joints"][0]
        self.assertAlmostEqual(joint["shear_force"], 1.0)
        self.assertAlmostEqual(joint["utilization"], 0.5)
        self.assertIn("demand/capacity", log.meta["structure"]["note"])


class TestCraneConstant(unittest.TestCase):
    def test_campaign_crane_is_the_ground_walker(self) -> None:
        # Imported so a rename of the scaffold crane fails here too.
        self.assertEqual(CRANE.reach_radius, 4)
        self.assertFalse(CRANE.lattice_only)


if __name__ == "__main__":
    unittest.main()
