# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Placement error, the ski/claw snap, and a miss as a defect.

Closed forms cover the uniform box against perfect mates, and the
translation-only random walk. The IEEE strut-length RMS numbers are not
targets. k is not an input.
"""

from __future__ import annotations

import random
import tempfile
import unittest
from pathlib import Path

import numpy as np

from experiments.precision_campaign import load_rows, run_smoke
from sim.armadas import (
    CLAW_OUT_OF_PLANE_CM,
    GRIPPER_OUT_OF_PLANE_CM,
    PETAL_IN_PLANE_CM,
    PLACE_TOLERANCE_CM,
    PLACE_TOLERANCE_DEG,
    SKI_IN_PLANE_CM,
)
from sim.geometry3d import CubicLattice3D
from sim.precision import (
    PITCH_M,
    PrecisionModel,
    armadas_precision,
    assemble,
    bridge_chain,
    cantilever_chain,
    expected_uniform_fail_rate,
    rms_line,
    rms_plane,
    rms_position,
    tip_length_error,
    translation_tip_rms,
    uniform_pass_probability,
)
from sim.robot import IDLE, Robot
from sim.swarm import Swarm
from sim.world import World
from sim.world3d import World3D


def _pool(runs) -> float:
    attempts = sum(item.n_attempts for item in runs)
    failures = sum(item.n_failures for item in runs)
    return failures / attempts if attempts else 0.0


class TestPublishedDefaults(unittest.TestCase):
    def test_defaults_are_the_manuscript_box_and_envelopes(self) -> None:
        model = armadas_precision()
        self.assertEqual(model.position_half_cm, PLACE_TOLERANCE_CM)
        self.assertEqual(model.position_half_cm, 1.78)
        self.assertEqual(model.angle_half_deg, PLACE_TOLERANCE_DEG)
        self.assertEqual(model.angle_half_deg, 3.0)
        self.assertEqual(model.ski_cm, SKI_IN_PLANE_CM)
        self.assertEqual(model.claw_cm, CLAW_OUT_OF_PLANE_CM)
        self.assertEqual(GRIPPER_OUT_OF_PLANE_CM, CLAW_OUT_OF_PLANE_CM)
        self.assertEqual(model.pitch_m, PITCH_M)
        self.assertEqual(model.pitch_m, 0.3048)
        self.assertGreater(PETAL_IN_PLANE_CM, model.position_half_cm)
        self.assertEqual(model.in_plane_m, 0.0178)
        self.assertAlmostEqual(model.out_of_plane_m, 0.0134)
        self.assertTrue(model.snap)
        self.assertTrue(model.check)
        self.assertEqual(model.distribution, "uniform")
        self.assertEqual(model.residual_in_plane_mm, 0.0)
        self.assertFalse(hasattr(model, "k"))

    def test_unknown_distribution_raises(self) -> None:
        with self.assertRaises(ValueError):
            PrecisionModel(distribution="histogram")


class TestClosedForms(unittest.TestCase):
    def test_single_mate_pass_probability(self) -> None:
        model = armadas_precision()
        # Ski equals the box, so only the claw truncates one axis.
        expected = CLAW_OUT_OF_PLANE_CM / PLACE_TOLERANCE_CM
        self.assertAlmostEqual(uniform_pass_probability(1, model), expected)
        self.assertAlmostEqual(
            uniform_pass_probability(2, model), expected**2
        )
        self.assertAlmostEqual(
            uniform_pass_probability(3, model), expected**3
        )

    def test_cantilever_and_bridge_rates(self) -> None:
        model = armadas_precision()
        single = 1.0 - uniform_pass_probability(1, model)
        double = 1.0 - uniform_pass_probability(2, model)
        for n in (1, 3, 12):
            datums, order = cantilever_chain(n)
            self.assertAlmostEqual(
                expected_uniform_fail_rate(order, datums, model), single
            )
        datums, order = bridge_chain(5)
        # Five single-mate deck cells and one closing cap with two normals.
        self.assertAlmostEqual(
            expected_uniform_fail_rate(order, datums, model),
            (5 * single + double) / 6,
        )

    def test_corner_cell_uses_three_axes(self) -> None:
        model = armadas_precision()
        # Three datums, each sharing a different face with the new cell.
        datums = ((0, 1, 1), (1, 0, 1), (1, 1, 0))
        order = ((1, 1, 1),)
        rate = expected_uniform_fail_rate(order, datums, model)
        self.assertAlmostEqual(rate, 1.0 - uniform_pass_probability(3, model))

    def test_scale_two_accepts_the_whole_box(self) -> None:
        model = armadas_precision(envelope_scale=2.0)
        self.assertEqual(uniform_pass_probability(3, model), 1.0)
        datums, order = cantilever_chain(4)
        self.assertEqual(expected_uniform_fail_rate(order, datums, model), 0.0)

    def test_half_scale_binds_ski_and_claw(self) -> None:
        model = armadas_precision(envelope_scale=0.5)
        half = model.position_half_m
        probability = (
            min(1.0, model.out_of_plane_m / half)
            * min(1.0, model.in_plane_m / half) ** 2
        )
        self.assertAlmostEqual(uniform_pass_probability(1, model), probability)
        self.assertLess(model.in_plane_m, half)

    def test_translation_only_tip_rms(self) -> None:
        half = 0.0178
        self.assertAlmostEqual(translation_tip_rms(0, half), 0.0)
        self.assertAlmostEqual(translation_tip_rms(9, half), half * 3.0)
        claw = 0.0134
        ski = 0.0178
        truncated = translation_tip_rms(4, half, claw_m=claw, ski_m=ski)
        self.assertLess(truncated, translation_tip_rms(4, half))


class TestMonteCarlo(unittest.TestCase):
    def test_snap_sits_on_the_ideal_centers(self) -> None:
        datums, order = cantilever_chain(12)
        for seed in range(6):
            result = assemble(
                armadas_precision(),
                order,
                datums,
                random.Random(seed),
                max_attempts=30,
            )
            self.assertTrue(result.complete)
            self.assertLess(rms_position(result, PITCH_M), 1e-9)

    def test_uniform_miss_rate_matches_the_closed_form(self) -> None:
        model_kwargs = {}
        datums, order = cantilever_chain(1)
        runs = [
            assemble(
                armadas_precision(**model_kwargs),
                order,
                datums,
                random.Random(1000 + seed),
                max_attempts=1,
            )
            for seed in range(3000)
        ]
        # One attempt, no retry, so the pooled rate is the draw's miss rate.
        expected = expected_uniform_fail_rate(order, datums, armadas_precision())
        self.assertAlmostEqual(_pool(runs), expected, delta=0.02)

    def test_bridge_closing_rate_matches_the_closed_form(self) -> None:
        datums, order = bridge_chain(4)
        model = armadas_precision()
        runs = [
            assemble(
                armadas_precision(),
                order,
                datums,
                random.Random(2000 + seed),
                max_attempts=40,
            )
            for seed in range(400)
        ]
        self.assertTrue(all(item.complete for item in runs))
        self.assertAlmostEqual(
            _pool(runs), expected_uniform_fail_rate(order, datums, model), delta=0.03
        )

    def test_open_loop_translation_matches_the_random_walk(self) -> None:
        n = 6
        datums, order = cantilever_chain(n)
        runs = []
        for seed in range(1500):
            result = assemble(
                PrecisionModel(snap=False, check=False, angle_half_deg=0.0),
                order,
                datums,
                random.Random(seed),
                max_attempts=1,
            )
            runs.append(result)
        tips = [tip_length_error(item, datums[0], order[-1], PITCH_M)[0] for item in runs]
        measured = float(np.sqrt(np.mean(np.square(tips))))
        expected = translation_tip_rms(n, 0.0178)
        self.assertAlmostEqual(measured / expected, 1.0, delta=0.06)

    def test_gate_without_snap_truncates_the_step(self) -> None:
        n = 5
        datums, order = cantilever_chain(n)
        model = PrecisionModel(snap=False, check=True, angle_half_deg=0.0)
        runs = [
            assemble(PrecisionModel(snap=False, check=True, angle_half_deg=0.0), order, datums, random.Random(seed), max_attempts=40)
            for seed in range(800)
        ]
        self.assertTrue(all(item.complete for item in runs))
        tips = [tip_length_error(item, datums[0], order[-1], PITCH_M)[0] for item in runs]
        measured = float(np.sqrt(np.mean(np.square(tips))))
        expected = translation_tip_rms(
            n, model.position_half_m, claw_m=model.out_of_plane_m, ski_m=model.in_plane_m
        )
        self.assertAlmostEqual(measured / expected, 1.0, delta=0.08)
        self.assertLess(measured, translation_tip_rms(n, model.position_half_m))

    def test_angle_walk_grows_and_snap_does_not(self) -> None:
        short = _median_tip(4)
        long = _median_tip(16)
        self.assertGreater(long, short * 1.5)
        for n in (4, 16):
            datums, order = cantilever_chain(n)
            result = assemble(
                armadas_precision(), order, datums, random.Random(3), max_attempts=40
            )
            self.assertLess(rms_position(result, PITCH_M), 1e-9)

    def test_open_loop_bridge_walks_off_the_far_pier(self) -> None:
        datums, order = bridge_chain(16)
        runs = [
            assemble(
                PrecisionModel(snap=False, check=True),
                order,
                datums,
                random.Random(seed),
                max_attempts=6,
            )
            for seed in range(8)
        ]
        self.assertEqual(sum(item.complete for item in runs), 0)
        snapped = [
            assemble(
                armadas_precision(),
                order,
                datums,
                random.Random(seed),
                max_attempts=40,
            )
            for seed in range(4)
        ]
        self.assertTrue(all(item.complete for item in snapped))

    def test_same_seed_repeats_and_a_new_seed_moves(self) -> None:
        def tip(seed: int) -> np.ndarray:
            datums, order = cantilever_chain(4)
            result = assemble(
                PrecisionModel(snap=False, check=False, angle_half_deg=0.0),
                order,
                datums,
                random.Random(seed),
                max_attempts=1,
            )
            return result.poses[(0, 0, 4)].t.copy()

        self.assertTrue(np.allclose(tip(7), tip(7)))
        self.assertFalse(np.allclose(tip(7), tip(8)))

    def test_zero_distribution_has_no_error_and_no_miss(self) -> None:
        datums, order = cantilever_chain(8)
        for snap, check in ((True, True), (False, False)):
            result = assemble(
                PrecisionModel(distribution="zero", snap=snap, check=check),
                order,
                datums,
                random.Random(0),
                max_attempts=1,
            )
            self.assertTrue(result.complete)
            self.assertEqual(result.n_failures, 0)
            self.assertAlmostEqual(rms_position(result, PITCH_M), 0.0, places=12)

    def test_wide_gaussian_misses_more_than_the_box(self) -> None:
        datums, order = cantilever_chain(1)
        runs = [
            assemble(
                PrecisionModel(distribution="gaussian", sigma_position_cm=10.0),
                order,
                datums,
                random.Random(seed),
                max_attempts=1,
            )
            for seed in range(400)
        ]
        self.assertGreater(_pool(runs), 0.8)
        self.assertLess(
            expected_uniform_fail_rate(order, datums, armadas_precision()), 0.3
        )

    def test_scripted_draw_and_residual_bound(self) -> None:
        datums, order = cantilever_chain(1)
        miss = PrecisionModel(scripted=[(0.0, 0.0, 0.02, 0.0)])
        result = assemble(miss, order, datums, random.Random(0), max_attempts=1)
        self.assertFalse(result.complete)
        self.assertEqual(result.n_failures, 1)
        angle = PrecisionModel(scripted=[(0.0, 0.0, 0.0, 5.0)])
        result = assemble(angle, order, datums, random.Random(0), max_attempts=1)
        self.assertFalse(result.complete)
        seated = PrecisionModel(
            scripted=[(0.0, 0.0, 0.0, 0.0)],
            residual_in_plane_mm=0.1,
            residual_out_of_plane_mm=0.1,
        )
        result = assemble(seated, order, datums, random.Random(1), max_attempts=1)
        self.assertTrue(result.complete)
        error = rms_position(result, PITCH_M)
        self.assertGreater(error, 0.0)
        self.assertLess(error, 2e-4)


def _median_tip(n: int) -> float:
    datums, order = cantilever_chain(n)
    tips = []
    for seed in range(12):
        result = assemble(
            PrecisionModel(snap=False, check=False),
            order,
            datums,
            random.Random(seed),
            max_attempts=1,
        )
        tips.append(tip_length_error(result, datums[0], order[-1], PITCH_M)[0])
    return float(np.median(tips))


class TestSurfaceFit(unittest.TestCase):
    def test_plane_and_line(self) -> None:
        flat = [(0, 0, 0), (1, 0, 0), (0, 1, 0), (1, 1, 0)]
        self.assertAlmostEqual(rms_plane(flat), 0.0, places=12)
        self.assertAlmostEqual(rms_line([(0, 0, 0), (0, 0, 1), (0, 0, 2)]), 0.0, places=12)
        self.assertGreater(rms_plane([(0, 0, 0), (1, 0, 0), (0, 1, 0), (0, 0, 1)]), 0.0)
        self.assertGreater(rms_line([(0, 0, 0), (0, 0, 1), (0, 1, 0.5)]), 0.0)


class TestDefectPath(unittest.TestCase):
    def test_robot_miss_is_removed_and_replaced(self) -> None:
        world = World(rows=4, cols=4)
        world.blueprint[2, 2] = True
        world.blueprint[2, 3] = True
        model = PrecisionModel(scripted=[(0.0, 0.0, 0.02, 0.0), (0.0, 0.0, 0.0, 0.0)])
        robot = Robot(pos=(2, 1), depot=(2, 0), precision=model)
        robot.load_plan([(2, 2), (2, 3)])
        removed = False
        for _ in range(400):
            robot.tick(world)
            if robot.needs_replan and world.defect_count == 0 and robot.capture_failures == 1:
                removed = True
                break
        self.assertTrue(removed, "capture miss did not travel inspect → remove")
        self.assertEqual(robot.state, IDLE)
        self.assertGreater(robot.defects_found, 0)
        robot.load_plan([(2, 3)])
        for _ in range(400):
            robot.tick(world)
            if world.complete and robot.state == IDLE and not robot.tasks:
                break
        self.assertTrue(world.complete)
        self.assertEqual(world.defect_count, 0)
        self.assertEqual(robot.capture_failures, 1)
        self.assertEqual(model.scripted, [])
        self.assertLess(rms_position_of(model, (2, 3)), 1e-9)

    def test_blind_robot_leaves_the_miss_standing(self) -> None:
        world = World(rows=4, cols=4)
        world.blueprint[2, 2] = True
        world.blueprint[2, 3] = True
        model = PrecisionModel(scripted=[(0.0, 0.0, 0.02, 0.0)])
        robot = Robot(
            pos=(2, 1), depot=(2, 0), precision=model, inspect_enabled=False
        )
        robot.load_plan([(2, 2), (2, 3)])
        for _ in range(400):
            robot.tick(world)
            if robot.state == IDLE and not robot.tasks:
                break
        self.assertEqual(robot.capture_failures, 1)
        self.assertEqual(world.defect_count, 1)
        self.assertEqual(world.built_count, 1)
        self.assertTrue(world.is_defective((2, 3)))

    def test_swarm_repairs_a_miss(self) -> None:
        world = World3D(4, 6, 8)
        world.blueprint[1, 2, 3] = True
        world.blueprint[1, 2, 4] = True
        model = PrecisionModel(scripted=[(0.0, 0.0, 0.02, 0.0), (0.0, 0.0, 0.0, 0.0)])
        depot = (1, 2, 1)
        swarm = Swarm(
            world,
            depot,
            starts=[(1, 3, 1)],
            geometry_factory=CubicLattice3D,
            precision=model,
        )
        for _ in range(2000):
            if swarm.done:
                break
            swarm.tick()
        self.assertTrue(swarm.done)
        self.assertTrue(world.complete)
        self.assertEqual(world.defect_count, 0)
        self.assertEqual(swarm.capture_failures, 1)
        self.assertEqual(swarm.defects_found, 1)
        removed = [event for event in swarm.events if event[2] == "defect_removed"]
        self.assertEqual(len(removed), 1)


def rms_position_of(model: PrecisionModel, cell) -> float:
    from sim.precision import AssemblyResult

    result = AssemblyResult(
        poses=dict(model.poses),
        order=(cell,),
        datums=(),
        n_attempts=0,
        n_failures=0,
        n_placed=1,
        complete=True,
    )
    return rms_position(result, model.pitch_m)


class TestCampaignSmoke(unittest.TestCase):
    def test_smoke_writes_csv(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = run_smoke(Path(tmp))
            rows = load_rows(Path(tmp) / "precision_campaign.csv")
        self.assertGreater(len(rows), 0)
        corrected = [
            row
            for row in rows
            if row["mode"] == "corrected" and row["structure"] == "cantilever"
        ]
        open_loop = [
            row
            for row in rows
            if row["mode"] == "uncorrected" and row["structure"] == "cantilever"
        ]
        self.assertTrue(corrected)
        self.assertTrue(all(row["rms_position_m"] < 1e-9 for row in corrected))
        self.assertTrue(all(row["stiffness"] == "not_used" for row in rows))
        self.assertGreater(
            max(row["rms_position_m"] for row in open_loop),
            max(row["rms_position_m"] for row in corrected),
        )
        self.assertIn("k is not used", result["summary"])


class TestCommittedCampaign(unittest.TestCase):
    def test_committed_rows_match_the_closed_form(self) -> None:
        path = Path(__file__).resolve().parents[1] / "docs" / "precision_campaign.csv"
        rows = load_rows(path)
        self.assertGreater(len(rows), 200)
        corrected = [
            row
            for row in rows
            if row["structure"] == "cantilever"
            and row["mode"] == "corrected"
            and row["envelope_scale"] == 1.0
        ]
        self.assertTrue(corrected)
        attempts = sum(row["n_attempts"] for row in corrected)
        failures = sum(row["n_failures"] for row in corrected)
        datums, order = cantilever_chain(3)
        expected = expected_uniform_fail_rate(order, datums, armadas_precision())
        self.assertAlmostEqual(failures / attempts, expected, delta=0.02)
        self.assertTrue(all(row["rms_position_m"] < 1e-8 for row in corrected))
        self.assertTrue(all(row["stiffness"] == "not_used" for row in rows))
        open_tips = {}
        for length in (3, 6, 12, 24):
            group = [
                row["tip_error_m"]
                for row in rows
                if row["structure"] == "cantilever"
                and row["size"] == length
                and row["mode"] == "uncorrected"
                and row["envelope_scale"] == 1.0
            ]
            self.assertEqual(len(group), 40)
            open_tips[length] = float(np.median(group))
        self.assertLess(open_tips[3], open_tips[6])
        self.assertLess(open_tips[6], open_tips[12])
        self.assertLess(open_tips[12], open_tips[24])
        shelter = [
            row
            for row in rows
            if row["structure"] == "shelter" and row["mode"] == "corrected" and row["envelope_scale"] == 1.0
        ]
        self.assertEqual(len(shelter), 16)
        self.assertTrue(all(row["n_cells"] == 256 for row in shelter))
        self.assertTrue(all(row["complete"] == 1 for row in shelter))
        self.assertTrue(all(row["rms_position_m"] < 1e-8 for row in shelter))
        self.assertTrue(all(row["surface_fit"] == "plane" for row in shelter))


if __name__ == "__main__":
    unittest.main()
