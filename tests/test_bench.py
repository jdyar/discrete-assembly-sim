# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Public benchmark runner: columns, unfitted k, and the smoke tasks.

The smoke set is the fast half of the suite. Spans 5–8, the inchworm
scaffold search, and the 256-cell shelter run from ``python main.py bench``
and are not repeated here. k = 1 is not a fit.
"""

from __future__ import annotations

import csv
import tempfile
import unittest
from pathlib import Path

from sim.bench import COLUMNS, SMOKE, markdown_table, replay_completion, run_ids, write_table
from sim.format import FORMAT_VERSION, validate
from sim.scenarios import SUITE, catalog

_DOCUMENTS = None


def documents() -> dict:
    global _DOCUMENTS
    if _DOCUMENTS is None:
        _DOCUMENTS = catalog()
    return _DOCUMENTS


class SmokeRun(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.rows = {row["id"]: row for row in run_ids(list(SMOKE))}

    def test_smoke_covers_the_named_tasks(self):
        for ident in SMOKE:
            self.assertIn(ident, self.rows)

    def test_every_row_has_the_table_columns(self):
        for ident, row in self.rows.items():
            for column in COLUMNS:
                self.assertIn(column, row, f"{ident} missing {column}")

    def test_k_is_one_and_unfitted(self):
        for ident, row in self.rows.items():
            self.assertEqual(row["k"], 1.0, ident)
            self.assertFalse(row["fitted"], ident)
            self.assertIn("not a fit", row["note"], ident)
            self.assertIn("configuration", row["note"], ident)

    def test_completion_is_a_fraction(self):
        for ident, row in self.rows.items():
            self.assertGreaterEqual(row["completion"], 0.0, ident)
            self.assertLessEqual(row["completion"], 1.0, ident)

    def test_bridge_span_4_finishes_with_no_scaffold(self):
        row = self.rows["bridge-span-4"]
        self.assertEqual(row["status"], "complete")
        self.assertEqual(row["completion"], 1.0)
        self.assertEqual(row["scaffold_count"], 0)
        self.assertGreater(row["makespan"], 0)
        self.assertEqual(row["makespan_unit"], "tick")
        self.assertAlmostEqual(row["peak_sag"], 4.5)
        self.assertEqual(row["peak_sag_units"], "mass*g/k")
        self.assertAlmostEqual(row["joint_utilization"], 1.0)
        self.assertEqual(row["faults"], 0)
        self.assertEqual(row["recoveries"], 0)

    def test_cantilever_arm_peak_sag_is_the_three_voxel_chain(self):
        # n(n+1)/2 at n = 3, k = mass = g = 1, is 6. Robot weight is off.
        row = self.rows["cantilever-arm"]
        self.assertEqual(row["status"], "complete")
        self.assertAlmostEqual(row["peak_sag"], 6.0)
        self.assertAlmostEqual(row["joint_utilization"], 1.0)

    def test_tower_peak_sag_is_the_axial_chain(self):
        row = self.rows["tower"]
        self.assertEqual(row["status"], "complete")
        self.assertAlmostEqual(row["peak_sag"], 6.0)
        self.assertEqual(row["scaffold_count"], 0)

    def test_hollow_roof_is_a_stability_refusal(self):
        row = self.rows["hollow-roof-limit-0"]
        self.assertEqual(row["status"], "infeasible")
        self.assertEqual(row["completion"], 0.0)
        self.assertEqual(row["makespan"], 0)
        self.assertGreaterEqual(row["gate_refusals"], 1)
        self.assertEqual(row["gate_accepts"], 0)
        refuses = [
            event
            for event in row["event_log"]["events"]
            if event["type"] == "gate" and event["verdict"] == "refuse"
        ]
        self.assertEqual(refuses[0]["gate"], "stability")
        self.assertTrue(refuses[0]["reason"])

    def test_fault_tower_replaces_the_scripted_fastener(self):
        row = self.rows["fault-tower"]
        self.assertEqual(row["status"], "complete")
        self.assertEqual(row["completion"], 1.0)
        self.assertEqual(row["faults"], 1)
        self.assertEqual(row["recoveries"], 1)
        self.assertEqual(row["unrecovered"], 0)
        self.assertEqual(row["cascades"], 0)
        self.assertGreater(row["recovery_time"], 0)
        fault = next(event for event in row["event_log"]["events"] if event["type"] == "fault")
        self.assertEqual(fault["fault"], "fastener")
        recovery = next(
            event for event in row["event_log"]["events"] if event["type"] == "recovery"
        )
        self.assertEqual(recovery["action"], "re-place")

    def test_fault_wall_recovers_every_bad_bond(self):
        row = self.rows["fault-wall"]
        self.assertEqual(row["status"], "complete")
        self.assertEqual(row["completion"], 1.0)
        self.assertGreater(row["faults"], 0)
        self.assertEqual(row["recoveries"], row["faults"])
        self.assertEqual(row["unrecovered"], 0)
        self.assertIsNone(row["peak_sag"])

    def test_wall_and_cuboct_complete(self):
        self.assertEqual(self.rows["wall-2d"]["status"], "complete")
        self.assertEqual(self.rows["wall-2d"]["completion"], 1.0)
        self.assertEqual(self.rows["wall-2d"]["faults"], 0)
        self.assertIsNone(self.rows["wall-2d"]["peak_sag"])
        self.assertEqual(self.rows["cuboct-block"]["status"], "complete")
        self.assertEqual(self.rows["cuboct-block"]["scaffold_count"], 0)
        self.assertGreater(self.rows["cuboct-block"]["peak_sag"], 0)

    def test_crane_span_4_scaffold_search_inserts_nothing(self):
        row = self.rows["scaffold-span-4-crane"]
        self.assertEqual(row["status"], "complete")
        self.assertEqual(row["scaffold_count"], 0)
        self.assertEqual(row["completion"], 1.0)
        self.assertAlmostEqual(row["peak_sag"], 4.5)

    def test_replay_matches_completion(self):
        for ident in ("bridge-span-4", "fault-tower", "fault-wall", "cantilever-arm"):
            row = self.rows[ident]
            scenario = documents()[ident]
            self.assertAlmostEqual(
                replay_completion(scenario, row["event_log"]),
                row["completion"],
                places=6,
                msg=ident,
            )

    def test_event_logs_validate(self):
        for ident, row in self.rows.items():
            log = row["event_log"]
            validate(log)
            self.assertEqual(log["format_version"], FORMAT_VERSION)
            self.assertEqual(log["scenario_id"], ident)
            self.assertEqual(log["makespan"], float(row["makespan"]))

    def test_bridge_log_has_placements_and_gate_accepts(self):
        events = self.rows["bridge-span-4"]["event_log"]["events"]
        placements = [event for event in events if event["type"] == "placement"]
        accepts = [
            event
            for event in events
            if event["type"] == "gate" and event["verdict"] == "accept"
        ]
        self.assertGreater(len(placements), 0)
        self.assertGreater(len(accepts), 0)
        self.assertTrue(any(event["gate"] == "stability" for event in accepts))
        self.assertTrue(any(event["gate"] == "sag" for event in accepts))

    def test_markdown_and_csv_carry_the_columns(self):
        rows = [self.rows[ident] for ident in SMOKE]
        text = markdown_table(rows)
        self.assertIn("peak sag", text)
        self.assertIn("joint utilization", text)
        self.assertIn("not a fit", text)
        self.assertIn("bridge-span-4", text)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "results.csv"
            write_table(rows, path)
            with path.open(encoding="utf-8") as handle:
                header = next(csv.reader(handle))
            self.assertEqual(header, list(COLUMNS))


class SuiteShape(unittest.TestCase):
    def test_required_families_are_in_the_suite(self):
        families = {documents()[ident]["family"] for ident in SUITE}
        for name in ("bridge", "cantilever", "tower", "shelter", "scaffold", "fault"):
            self.assertIn(name, families)

    def test_spans_and_scaffold_cases_are_named(self):
        self.assertTrue(all(f"bridge-span-{span}" in SUITE for span in range(4, 9)))
        self.assertIn("scaffold-span-4-crane", SUITE)
        self.assertIn("scaffold-span-4-inchworm", SUITE)
        self.assertIn("scaffold-span-8-crane", SUITE)
        self.assertIn("armadas-shelter", SUITE)
        self.assertIn("cantilever-3", SUITE)
        self.assertIn("cantilever-4", SUITE)
        self.assertIn("cantilever-arm", SUITE)
        self.assertIn("tower", SUITE)

    def test_smoke_is_a_subset_of_the_suite(self):
        self.assertTrue(set(SMOKE) <= set(SUITE))


if __name__ == "__main__":
    unittest.main()
