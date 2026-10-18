# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Smoke coverage for the structural-gate cost campaign.

The published charts are a multi-minute run (`python main.py stability`).
What this module locks is the script: the named shapes mean what the
writeup says they mean, the smoke run finishes both statuses, and the
CSV plus the chart writers accept that smoke table.
"""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from experiments.stability_cost import (
    CONDITIONS,
    CSV_FIELDS,
    FUZZ_SEEDS,
    GATE_OFF,
    ROBOT_COUNTS,
    START_SEEDS,
    build_bridge,
    build_cantilever,
    build_fuzz,
    build_hollow_box,
    build_wall,
    finished_stable,
    iter_fuzz_jobs,
    iter_named_jobs,
    load_rows,
    run_smoke,
)


class TestShapeScreens(unittest.TestCase):
    """Finished-structure screen, not a build order.

    These are the spans the campaign is about. A failure here means a
    blueprint drifted, not that the planner got slower.
    """

    def test_wall_is_columns(self):
        world, _depot = build_wall()
        self.assertEqual(world.blueprint_count, 12)
        self.assertTrue(finished_stable(world, 0))

    def test_hollow_roof_needs_a_span(self):
        world, _depot = build_hollow_box()
        self.assertEqual(world.blueprint_count, 17)
        self.assertFalse(finished_stable(world, 0))
        self.assertTrue(finished_stable(world, 1))

    def test_bridge_spans(self):
        short, _depot = build_bridge(5)
        self.assertEqual(short.blueprint_count, 8)
        self.assertFalse(finished_stable(short, 1))
        self.assertTrue(finished_stable(short, 2))
        long, _depot = build_bridge(6)
        self.assertEqual(long.blueprint_count, 9)
        self.assertFalse(finished_stable(long, 2))
        self.assertTrue(finished_stable(long, 3))

    def test_cantilevers(self):
        arm3, _depot = build_cantilever(3)
        self.assertFalse(finished_stable(arm3, 2))
        self.assertTrue(finished_stable(arm3, 3))
        arm4, _depot = build_cantilever(4)
        self.assertFalse(finished_stable(arm4, 3))
        self.assertTrue(finished_stable(arm4, None))


class TestCampaignGrid(unittest.TestCase):
    def test_named_jobs_cover_the_requested_factors(self):
        jobs = list(iter_named_jobs())
        names = {job["blueprint"] for job in jobs}
        self.assertEqual(
            names,
            {
                "wall",
                "hollow_box",
                "bridge",
                "long_bridge",
                "cantilever",
                "cantilever_long",
            },
        )
        conditions = {job["condition"] for job in jobs}
        self.assertEqual(conditions, {condition for condition, _limit in CONDITIONS})
        self.assertIn(GATE_OFF, conditions)
        self.assertEqual({job["n_robots"] for job in jobs}, set(ROBOT_COUNTS))
        self.assertEqual({job["start_seed"] for job in jobs}, set(START_SEEDS))
        # 6 shapes x 6 gate settings x 2 robot counts x 3 seeds.
        self.assertEqual(len(jobs), 6 * 6 * 2 * 3)

    def test_fuzz_is_deterministic_and_small(self):
        for seed in FUZZ_SEEDS:
            first = build_fuzz(seed)
            second = build_fuzz(seed)
            self.assertEqual(first[2], second[2])
            self.assertEqual(first[3], second[3])
            self.assertEqual(int(first[0].blueprint_count), int(second[0].blueprint_count))
            self.assertLessEqual(int(first[0].blueprint_count), 12)
            self.assertGreaterEqual(int(first[0].blueprint_count), 4)
        jobs = list(iter_fuzz_jobs())
        self.assertEqual(len(jobs), len(FUZZ_SEEDS) * 6 * 2 * len((0, 1)))


class TestSmokeRun(unittest.TestCase):
    """One wall, one cantilever, gate off / tight / unlimited.

    The wall is feasible with the gate at 0. The length-3 cantilever is
    proved infeasible at 0 and feasible with no span limit. Peak sag on
    the wall is the column compression; the numbers are model units.
    """

    def test_smoke_writes_csv_and_charts(self):
        with tempfile.TemporaryDirectory() as tmp:
            result = run_smoke(tmp)
            rows = result["rows"]
            self.assertEqual(len(rows), 4)
            by_key = {(row["blueprint"], row["condition"]): row for row in rows}

            wall_off = by_key[("wall", "off")]
            wall_on = by_key[("wall", "0")]
            self.assertEqual(wall_off["status"], "feasible")
            self.assertEqual(wall_on["status"], "feasible")
            self.assertTrue(wall_off["completed"])
            self.assertTrue(wall_on["completed"])
            self.assertAlmostEqual(wall_off["peak_sag"], wall_on["peak_sag"])
            self.assertAlmostEqual(wall_off["peak_sag"], 3.0)
            self.assertEqual(wall_off["sag_errors"], 0)
            self.assertGreater(wall_off["makespan"], 0)

            rejected = by_key[("cantilever", "0")]
            allowed = by_key[("cantilever", "none")]
            self.assertEqual(rejected["status"], "infeasible")
            self.assertEqual(rejected["finished_stable"], "no")
            self.assertEqual(rejected["makespan"], "")
            self.assertEqual(rejected["peak_sag"], "")
            self.assertGreater(rejected["plan_s"], 0)
            self.assertEqual(allowed["status"], "feasible")
            self.assertTrue(allowed["completed"])
            self.assertGreater(allowed["peak_sag"], 0)
            self.assertEqual(allowed["sag_errors"], 0)
            # The arm is the finished shape: the gate does not lower its sag
            # below the final value, because there is no second support.
            self.assertAlmostEqual(allowed["peak_sag"], allowed["final_sag"])

            csv_path = Path(tmp) / "stability_cost.csv"
            with csv_path.open(newline="", encoding="utf-8") as handle:
                reader = csv.DictReader(handle)
                self.assertEqual(tuple(reader.fieldnames), CSV_FIELDS)
                stored = list(reader)
            self.assertEqual(len(stored), 4)
            self.assertEqual(stored[0]["k"], "1.000000")
            self.assertEqual(stored[0]["python"].count("."), 2)
            reloaded = load_rows(csv_path)
            self.assertEqual(reloaded[0]["condition"], "off")
            self.assertTrue(reloaded[0]["completed"])
            self.assertAlmostEqual(reloaded[0]["peak_sag"], 3.0)
            self.assertEqual(reloaded[2]["status"], "infeasible")
            self.assertEqual(reloaded[2]["completed"], "")

            for chart in result["charts"]:
                self.assertTrue(Path(chart).is_file())
                self.assertGreater(Path(chart).stat().st_size, 1000)


if __name__ == "__main__":
    unittest.main()
