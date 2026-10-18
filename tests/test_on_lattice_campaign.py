# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Smoke coverage for the crane versus on-lattice campaign.

The published charts are `python main.py onlattice`. What this module
locks is the script: the three shapes mean what the writeup says, the
sag cap rejects the one-sided bridge the inchworm has to walk, the
crane on that same bridge still has an order, and the smoke run writes
a CSV where crane peaks match voxels-only and the inchworm's do not.
"""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from experiments import on_lattice as campaign
from experiments.on_lattice import (
    CSV_FIELDS,
    FRAME,
    GAIT,
    SEED,
    _factory,
    _policy,
    build_bridge,
    build_cantilever,
    build_tower,
    buried_tops,
    choose_entries,
    finished_overhang,
    run_smoke,
    sag_cap,
)
from sim.geometry3d import CubicLattice3D
from sim.inchworm import OnLatticeGeometry
from sim.planner import plan_build_order
from sim.stability import estimate_sag, occupied_voxels


class TestShapes(unittest.TestCase):
    def test_counts_and_overhangs(self) -> None:
        arm = build_cantilever()
        self.assertEqual(arm.blueprint_count, 3)
        self.assertEqual(finished_overhang(arm), 2)
        span = build_bridge()
        self.assertEqual(span.blueprint_count, 4)
        self.assertEqual(finished_overhang(span), 1)
        tower = build_tower()
        self.assertEqual(tower.blueprint_count, 3)
        self.assertEqual(finished_overhang(tower), 0)

    def test_seed_is_shared_and_entries_avoid_buried_tops(self) -> None:
        for builder in (build_cantilever, build_bridge, build_tower):
            world = builder()
            for cell in SEED:
                self.assertEqual(int(world.occupancy[cell]), 2)
            entries = choose_entries(world, 3)
            self.assertEqual(len(entries), 3)
            buried = buried_tops(world)
            for stance in entries:
                for cell, normal in (stance[1], stance[2]):
                    self.assertFalse(cell in buried and normal == (1, 0, 0))
            self.assertTrue(CubicLattice3D(world).is_footing((1, 1, 0)))

    def test_bridge_sag_cap_is_above_the_voxels_alone(self) -> None:
        world = build_bridge()
        done = world.copy()
        for col in range(3, 7):
            done.place_voxel((2, 1, col))
        alone = estimate_sag(occupied_voxels(done), FRAME, k=1, mass=1, g=1).worst_sag
        cap = sag_cap(world, robot_mass=1.0, carried_mass=1.0)
        self.assertGreater(cap, alone)

    def test_sag_gate_rejects_the_inchworm_bridge_and_not_the_crane(self) -> None:
        world = build_bridge()
        policy = _policy(world, "sag", robot_mass=1.0, carried_mass=1.0)
        depot = choose_entries(world)[0]
        inch = plan_build_order(
            world.copy(), depot, _factory(policy), max_nodes=4000, stability=policy
        )
        self.assertIsNone(inch)
        crane_policy = _policy(world, "sag", robot_mass=0.0, carried_mass=0.0)
        crane = plan_build_order(
            world.copy(),
            (1, 1, 0),
            CubicLattice3D,
            max_nodes=4000,
            stability=crane_policy,
        )
        self.assertIsNotNone(crane)
        self.assertEqual(len(crane), 4)

    def test_viewer_duplicates_the_foot_offsets(self) -> None:
        html = Path("replay_viewer.html").read_text(encoding="utf-8")
        self.assertIn("const VIEW_FOOT_OUTSET = 0.56;", html)
        self.assertIn("const VIEW_BODY_LIFT = 0.20;", html)
        self.assertIn("const VIEW_VOXEL_HALF = 0.48;", html)
        self.assertIn("robot.feet", html)


class TestSmokeRun(unittest.TestCase):
    def test_smoke_writes_csv_and_charts(self) -> None:
        with tempfile.TemporaryDirectory() as tmp:
            result = run_smoke(tmp)
            with result["csv"].open(encoding="utf-8") as handle:
                rows = list(csv.DictReader(handle))
            self.assertEqual(list(rows[0].keys()), list(CSV_FIELDS))
            self.assertEqual(len(rows), 2)
            crane = next(row for row in rows if row["mode"] == "crane")
            inch = next(row for row in rows if row["mode"] == "on-lattice")
            self.assertEqual(crane["status"], "feasible")
            self.assertEqual(crane["completed"], "1")
            self.assertEqual(inch["status"], "feasible")
            self.assertEqual(inch["completed"], "1")
            self.assertEqual(float(crane["peak_sag_robots"]), float(crane["peak_sag_voxels"]))
            self.assertGreater(float(inch["peak_sag_robots"]), float(inch["peak_sag_voxels"]))
            self.assertEqual(crane["place_reordered"], "no")
            self.assertTrue(result["summary"].is_file())
            for chart in result["charts"]:
                self.assertGreater(chart.stat().st_size, 1000)


class TestGaitExport(unittest.TestCase):
    def test_campaign_uses_the_configured_gait(self) -> None:
        self.assertIs(campaign.gait_config(), GAIT)
        self.assertTrue(GAIT.allow_sides)
        self.assertFalse(GAIT.allow_inverted)
        self.assertEqual(GAIT.stride, 1)
        world = build_cantilever()
        geom = OnLatticeGeometry(world, GAIT)
        self.assertTrue(geom.stances())
