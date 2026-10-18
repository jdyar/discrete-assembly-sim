# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Opt-in structural samples in the run log.

The default log does not grow a structure key. When a StructureLog is
attached, each sampled occupancy change matches estimate_sag and the
joint strain proxy, including the closed-form shear chain.
"""

from __future__ import annotations

import json
import time
import unittest

from main import (
    BRIDGE_CANTILEVER,
    BRIDGE_DEPOT,
    build_bridge_world,
    build_world,
    run,
)
from sim.geometry import SquareLattice2D
from sim.geometry3d import CubicLattice3D
from sim.metrics import RUN_LOG_VERSION_3D, StructureLog, RunLog
from sim.planner import plan_build_order, validate_plan
from sim.stability import (
    StabilityPolicy,
    estimate_sag,
    is_stable,
    joint_strains,
    occupied_voxels,
    shear_cantilever_tip_sag,
)
from sim.world import World
from sim.world3d import World3D

# Finite clamp under a single root. The stock ground plane would plant
# every first-course voxel, and the arm would not be a shear chain.
_CLAMP = StabilityPolicy(
    up_axis=0, up_sign=1, ground_plane=None, base=((0, 0, 0),), max_cantilever=5
)


def _place_and_log(world, log, tick, cell):
    world.place_voxel(cell)
    log.log_tick(tick, world)


class TestStructureOptIn(unittest.TestCase):
    def test_default_log_has_no_structure_anywhere(self) -> None:
        data = run(build_world(), verbose=False).to_dict()
        self.assertNotIn("structure", data["meta"])
        for frame in data["ticks"]:
            self.assertNotIn("structure", frame)
            self.assertEqual(
                set(frame),
                {"tick", "occupancy", "placed", "robot", "robots"},
            )
        self.assertNotIn('"structure"', json.dumps(data))

    def test_bad_sampler_args_fail_at_construction(self) -> None:
        policy = CubicLattice3D(World3D(2, 2, 2)).stability_policy(1)
        with self.assertRaises(ValueError):
            StructureLog(policy, every=0)
        with self.assertRaises(ValueError):
            StructureLog(policy, pitch=0)
        with self.assertRaises(ValueError):
            StructureLog(policy, k=0)

    def test_meta_units_are_model_unless_si(self) -> None:
        policy = _CLAMP
        model = RunLog(structure=StructureLog(policy)).meta["structure"]
        self.assertEqual(model["displacement_units"], "mg/k")
        self.assertEqual(model["strain_units"], "1")
        self.assertFalse(model["si"])
        self.assertIn("not been fit", model["note"])
        self.assertIn("mass*g/k", model["note"])
        self.assertEqual(model["k_axial"], 1.0)
        self.assertEqual(model["k_shear"], 1.0)

        si = RunLog(
            structure=StructureLog(
                policy, k_axial=1.0e6, k_shear=2.0e5, mass=0.05, g=9.81,
                pitch=0.1, si=True,
            )
        ).meta["structure"]
        self.assertEqual(si["displacement_units"], "m")
        self.assertTrue(si["si"])
        self.assertEqual(si["pitch"], 0.1)
        self.assertIn("meters", si["note"])
        self.assertNotIn("mass*g/k", si["note"])


class TestStructureSamples(unittest.TestCase):
    def _cantilever_world(self) -> World3D:
        return World3D(2, 1, 3)

    def test_sample_matches_estimate_sag_and_closed_form(self) -> None:
        # Root plus two free voxels, one row, finite clamp under the root.
        # Tip sag is the shear-chain formula; joint shear is the weight
        # outboard of that joint divided by k_shear, over pitch.
        world = self._cantilever_world()
        spec = StructureLog(
            _CLAMP, k_axial=50.0, k_shear=4.0, mass=2.0, g=3.0, pitch=2.0
        )
        log = RunLog(structure=spec)
        log.log_tick(0, world)
        self.assertNotIn("structure", log.frames[0])

        _place_and_log(world, log, 1, (1, 0, 0))
        _place_and_log(world, log, 2, (1, 0, 1))
        _place_and_log(world, log, 3, (1, 0, 2))
        # Same occupancy again: not a new sample.
        log.log_tick(4, world)
        self.assertNotIn("structure", log.frames[4])

        frame = log.frames[3]["structure"]
        self.assertNotIn("error", frame)
        cells = occupied_voxels(world)
        estimate = estimate_sag(
            cells, _CLAMP, k_axial=50.0, k_shear=4.0, mass=2.0, g=3.0
        )
        strains = joint_strains(estimate, pitch=2.0)

        self.assertEqual(frame["n_fixed"], estimate.n_fixed)
        self.assertEqual(frame["n_free"], estimate.n_free)
        self.assertEqual(frame["worst_cell"], list(estimate.worst_cell))
        self.assertEqual(frame["worst_sag_cell"], [1, 0, 2])
        self.assertAlmostEqual(frame["worst_sag"], estimate.worst_sag)
        self.assertAlmostEqual(frame["worst_displacement"], estimate.worst_displacement)
        tip = shear_cantilever_tip_sag(2, mass=2.0, g=3.0, k_shear=4.0)
        self.assertAlmostEqual(frame["worst_sag"], tip)
        self.assertAlmostEqual(tip, 4.5)

        by_cell = {tuple(item["cell"]): item["u"] for item in frame["displacements"]}
        self.assertEqual(set(by_cell), set(estimate.displacements))
        for cell, vec in estimate.displacements.items():
            self.assertEqual(len(by_cell[cell]), 3)
            for got, want in zip(by_cell[cell], vec):
                self.assertAlmostEqual(got, want)
        # Horizontal motion is identically zero in this model.
        for vec in by_cell.values():
            self.assertAlmostEqual(vec[1], 0.0)
            self.assertAlmostEqual(vec[2], 0.0)

        self.assertEqual(len(frame["joints"]), len(strains))
        for got, want in zip(frame["joints"], strains):
            self.assertEqual(got["a"], list(want.a))
            self.assertEqual(got["b"], list(want.b))
            self.assertEqual(got["axis"], want.axis)
            self.assertAlmostEqual(got["axial"], want.axial)
            self.assertAlmostEqual(got["shear"], want.shear)
        # Inboard joint carries both free voxels; outboard carries the tip.
        # shear = (weight_outboard / k_shear) / pitch.
        weight = 2.0 * 3.0
        joints = {(tuple(j["a"]), tuple(j["b"])): j for j in frame["joints"]}
        inner = joints[((1, 0, 0), (1, 0, 1))]
        outer = joints[((1, 0, 1), (1, 0, 2))]
        self.assertEqual(inner["axis"], 2)
        self.assertAlmostEqual(inner["axial"], 0.0)
        self.assertAlmostEqual(inner["shear"], (2 * weight / 4.0) / 2.0)
        self.assertAlmostEqual(outer["shear"], (weight / 4.0) / 2.0)
        self.assertAlmostEqual(outer["axial"], 0.0)

        data = log.to_dict()
        self.assertEqual(data["version"], RUN_LOG_VERSION_3D)
        self.assertEqual(
            set(frame),
            {
                "worst_displacement",
                "worst_cell",
                "worst_sag",
                "worst_sag_cell",
                "n_fixed",
                "n_free",
                "displacements",
                "joints",
            },
        )
        # JSON round-trip keeps the values the viewer will see.
        blob = json.loads(json.dumps(data))
        again = blob["ticks"][3]["structure"]["worst_sag"]
        self.assertAlmostEqual(again, tip)

    def test_every_skips_intermediate_placements(self) -> None:
        world = self._cantilever_world()
        log = RunLog(structure=StructureLog(_CLAMP, every=2))
        log.log_tick(0, world)
        _place_and_log(world, log, 1, (1, 0, 0))
        _place_and_log(world, log, 2, (1, 0, 1))
        _place_and_log(world, log, 3, (1, 0, 2))
        self.assertNotIn("structure", log.frames[1])
        self.assertIn("structure", log.frames[2])
        self.assertNotIn("structure", log.frames[3])
        # The sample is the two-voxel state, not the final three.
        sampled_cells = [tuple(item["cell"]) for item in log.frames[2]["structure"]["displacements"]]
        self.assertEqual(sampled_cells, [(1, 0, 0), (1, 0, 1)])
        estimate = estimate_sag([(1, 0, 0), (1, 0, 1)], _CLAMP)
        self.assertAlmostEqual(
            log.frames[2]["structure"]["worst_sag"], estimate.worst_sag
        )

    def test_removal_is_an_occupancy_change(self) -> None:
        world = self._cantilever_world()
        log = RunLog(structure=StructureLog(_CLAMP, k_shear=4.0, k_axial=4.0))
        log.log_tick(0, world)
        _place_and_log(world, log, 1, (1, 0, 0))
        _place_and_log(world, log, 2, (1, 0, 1))
        world.remove_voxel((1, 0, 1))
        log.log_tick(3, world)
        sample = log.frames[3]["structure"]
        estimate = estimate_sag([(1, 0, 0)], _CLAMP, k_axial=4.0, k_shear=4.0)
        self.assertEqual([tuple(item["cell"]) for item in sample["displacements"]], [(1, 0, 0)])
        self.assertEqual(sample["joints"], [])
        # A lone planted voxel does not move, so there is no sag cell to mark.
        self.assertAlmostEqual(sample["worst_sag"], estimate.worst_sag)
        self.assertEqual(sample["worst_sag_cell"], None)
        self.assertIsNone(estimate.worst_sag_cell)

    def test_singular_partial_build_is_recorded_not_raised(self) -> None:
        world = World3D(4, 1, 1)
        policy = CubicLattice3D(world).stability_policy(0)
        log = RunLog(structure=StructureLog(policy))
        log.log_tick(0, world)
        # Level 2 with nothing under it: not fastened to the base.
        _place_and_log(world, log, 1, (2, 0, 0))
        sample = log.frames[1]["structure"]
        self.assertIn("error", sample)
        self.assertNotIn("displacements", sample)
        self.assertIn("base", sample["error"])

    def test_two_dimensional_sample_uses_the_flat_frame(self) -> None:
        world = World(rows=6, cols=4)
        policy = SquareLattice2D(world).stability_policy(max_cantilever=2)
        log = RunLog(structure=StructureLog(policy, k_axial=8.0, k_shear=2.0))
        log.log_tick(0, world)
        # Ground row is 5. (4, 1) is planted; (3, 1) stacks on it;
        # (3, 2) is one horizontal step out.
        _place_and_log(world, log, 1, (4, 1))
        _place_and_log(world, log, 2, (3, 1))
        _place_and_log(world, log, 3, (3, 2))
        sample = log.frames[3]["structure"]
        estimate = estimate_sag(
            occupied_voxels(world), policy, k_axial=8.0, k_shear=2.0
        )
        self.assertEqual(sample["worst_sag_cell"], list(estimate.worst_sag_cell))
        self.assertEqual(len(sample["displacements"][0]["u"]), 2)
        for item in sample["displacements"]:
            cell = tuple(item["cell"])
            for got, want in zip(item["u"], estimate.displacements[cell]):
                self.assertAlmostEqual(got, want)
        self.assertTrue(any(joint["axis"] == 1 for joint in sample["joints"]))

    def test_hundred_voxel_tower_samples_every_placement(self) -> None:
        world = World3D(102, 1, 1)
        policy = CubicLattice3D(world).stability_policy(0)
        log = RunLog(structure=StructureLog(policy, every=1))
        log.log_tick(0, world)
        started = time.perf_counter()
        for level in range(1, 101):
            world.place_voxel((level, 0, 0))
            log.log_tick(level, world)
        elapsed = time.perf_counter() - started
        samples = [f for f in log.frames if "structure" in f]
        self.assertEqual(len(samples), 100)
        self.assertLess(elapsed, 8.0, f"100-voxel sampling took {elapsed:.2f}s")
        last = samples[-1]["structure"]
        self.assertEqual(last["n_free"], 99)
        self.assertEqual(last["n_fixed"], 1)
        self.assertNotIn("error", last)


class TestBridgeDemo(unittest.TestCase):
    def test_finished_bridge_sags_at_midspan(self) -> None:
        world = build_bridge_world()
        cells = [
            tuple(int(v) for v in node) for node in zip(*world.blueprint.nonzero())
        ]
        self.assertEqual(len(cells), 18)
        policy = CubicLattice3D(world).stability_policy(BRIDGE_CANTILEVER)
        self.assertTrue(is_stable(cells, policy))
        self.assertFalse(is_stable(cells, policy, max_cantilever=2))
        estimate = estimate_sag(cells, policy)
        level, _row, col = estimate.worst_sag_cell
        self.assertEqual(level, 2)
        self.assertEqual(col, 6)
        # The deck is symmetric about column 6, so the two quarter-span
        # cells on one row drop the same amount.
        self.assertAlmostEqual(
            estimate.displacements[(2, 2, 4)][0],
            estimate.displacements[(2, 2, 8)][0],
        )
        strains = {
            (joint.a, joint.b): joint for joint in joint_strains(estimate, pitch=1.0)
        }
        pier = strains[((1, 2, 3), (2, 2, 3))]
        self.assertEqual(pier.axis, 0)
        self.assertLess(pier.axial, 0.0)  # compression
        self.assertAlmostEqual(pier.shear, 0.0)
        deck = strains[((2, 2, 5), (2, 2, 6))]
        self.assertEqual(deck.axis, 2)
        self.assertGreater(deck.shear, 0.0)
        self.assertAlmostEqual(deck.axial, 0.0)

    def test_bridge_plan_passes_the_stability_gate(self) -> None:
        world = build_bridge_world()
        policy = CubicLattice3D(world).stability_policy(BRIDGE_CANTILEVER)
        plan = plan_build_order(
            world, BRIDGE_DEPOT, CubicLattice3D, stability=policy
        )
        self.assertIsNotNone(plan)
        self.assertEqual(len(plan), world.blueprint_count)
        ok, why = validate_plan(
            world, plan, BRIDGE_DEPOT, CubicLattice3D, stability=policy
        )
        self.assertTrue(ok, why)


if __name__ == "__main__":
    unittest.main()
