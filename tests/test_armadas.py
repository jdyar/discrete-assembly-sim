# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""ARMADAS shelter benchmark: published figures, gates, roles, feed queue.

The 256-voxel shed is a stand-in. Fig. 5 has no coordinate table. The
numbers that are in Gregg et al. 2024 are checked as constants. The
numbers that are not, including every action duration except the Park
et al. 2023 step, are marked config and are not required to reproduce
100.4 h.
"""

from __future__ import annotations

import csv
import unittest
from pathlib import Path

from sim.armadas import (
    CLAW_ENFORCED_MM_BODY,
    CLAW_ENFORCED_MM_CAPTION,
    CONFIG_SOURCE,
    EDGE_MM,
    FACES_BOLTED,
    FASTEN,
    FEED_LOAD_S,
    N_VOXELS,
    PLACE_TOLERANCE_CM,
    PLACE_TOLERANCE_DEG,
    REPORTED_BUILD_DAYS,
    REPORTED_BUILD_HOURS,
    REPORTED_DAYS_AS_HOURS,
    SEED_COUNT,
    STEP_S,
    STEP_SOURCE,
    ActionTimes,
    build_gated_order,
    face_neighbors,
    feed_station_cells,
    inset_boxes_disjoint,
    passes_gate,
    shelter_cells,
    shelter_model,
    shelter_seed,
    simulate,
)
from sim.cuboct import VIEWER_INSET
from sim.stability import SAG_ATOL

ROOT = Path(__file__).resolve().parents[1]
CSV_PATH = ROOT / "docs" / "armadas_sweep.csv"


class PaperFigures(unittest.TestCase):
    def test_confirmed_counts_and_clocks(self):
        self.assertEqual(N_VOXELS, 256)
        self.assertEqual(EDGE_MM, 304.8)
        self.assertEqual(SEED_COUNT, 5)
        self.assertEqual(REPORTED_BUILD_HOURS, 100.4)
        self.assertEqual(REPORTED_BUILD_DAYS, 4.2)
        # The manuscript prints both. They are not the same clock.
        self.assertNotEqual(REPORTED_BUILD_HOURS, REPORTED_DAYS_AS_HOURS)
        self.assertAlmostEqual(REPORTED_DAYS_AS_HOURS, 100.8)

    def test_positioning_envelopes_and_the_caption_disagreement(self):
        self.assertEqual(PLACE_TOLERANCE_CM, 1.78)
        self.assertEqual(PLACE_TOLERANCE_DEG, 3.0)
        self.assertEqual(CLAW_ENFORCED_MM_BODY, 0.1)
        self.assertEqual(CLAW_ENFORCED_MM_CAPTION, 0.2)
        self.assertNotEqual(CLAW_ENFORCED_MM_BODY, CLAW_ENFORCED_MM_CAPTION)

    def test_action_times_are_labeled(self):
        times = ActionTimes()
        sources = times.sources()
        self.assertEqual(times.step_s, STEP_S)
        self.assertEqual(sources["step_s"], STEP_SOURCE)
        self.assertIn("park-2023", STEP_SOURCE)
        for name in ("feed_load_s", "handoff_s", "place_s", "fasten_joint_s"):
            self.assertEqual(sources[name], CONFIG_SOURCE)
            self.assertEqual(getattr(times, name), FEED_LOAD_S)
        # The config quantum was not derived from the lab clock.
        implied = REPORTED_BUILD_HOURS * 3600.0 / FACES_BOLTED
        self.assertNotAlmostEqual(times.fasten_joint_s, implied, delta=1.0)


class ShelterGeometry(unittest.TestCase):
    def test_256_unique_cells_and_a_five_voxel_seed_line(self):
        cells = shelter_cells()
        self.assertEqual(len(cells), 256)
        self.assertEqual(len(set(cells)), 256)
        seed = shelter_seed()
        self.assertEqual(len(seed), 5)
        self.assertTrue(set(seed) <= set(cells))
        levels = {cell[0] for cell in seed}
        rows = {cell[1] for cell in seed}
        cols = [cell[2] for cell in seed]
        self.assertEqual(levels, {1})
        self.assertEqual(rows, {1})
        self.assertEqual(cols, list(range(5)))

    def test_the_eave_is_an_overhang_and_the_inset_boxes_do_not_overlap(self):
        model = shelter_model()
        self.assertGreater(model.worst_overhang, 0)
        self.assertEqual(len(model.order), 256 - 5)
        self.assertTrue(inset_boxes_disjoint(model.cells, VIEWER_INSET))
        self.assertTrue(inset_boxes_disjoint(model.cells, 1.0))
        # Face-adjacent centers are one pitch apart. The 0.96 boxes
        # are strictly separated on that axis.
        self.assertGreaterEqual(1.0 - VIEWER_INSET, 0.03)

    def test_one_fasten_task_per_joint_and_not_the_lab_face_count(self):
        model = shelter_model()
        seen: set[tuple] = set()
        for cell, partners in zip(model.order, model.joints):
            self.assertGreaterEqual(len(partners), 1)
            for partner in partners:
                self.assertIn(partner, face_neighbors(cell))
                key = tuple(sorted((cell, partner)))
                self.assertNotIn(key, seen)
                seen.add(key)
        self.assertEqual(sum(len(partners) for partners in model.joints), len(seen))
        self.assertNotEqual(len(seen), FACES_BOLTED)

    def test_every_prefix_stays_inside_the_finished_caps(self):
        model = shelter_model()
        built = set(model.seed)
        peak = 0.0
        for cell in model.order:
            built.add(cell)
            ok, sag = passes_gate(built, model.policy, model.finished_sag)
            self.assertTrue(ok)
            peak = max(peak, sag)
        self.assertLessEqual(peak, model.finished_sag + SAG_ATOL)
        self.assertAlmostEqual(peak, model.peak_sag, places=6)
        self.assertGreater(model.finished_sag, 0.0)

    def test_one_pickup_is_in_front_of_the_seed_and_extras_are_explicit(self):
        stations = feed_station_cells(4)
        self.assertEqual(len(stations), 4)
        seed = shelter_seed()
        # Station 0 is the empty cell directly in front of the first seed voxel.
        self.assertEqual(stations[0], (seed[0][0], seed[0][1] - 1, seed[0][2]))
        self.assertEqual(len(set(stations)), 4)


class Schedule(unittest.TestCase):
    def _parallel_floor(self):
        seed = ((1, 1, 1),)
        cells = [seed[0], (1, 0, 1), (1, 1, 0), (1, 1, 2), (1, 2, 1)]
        return build_gated_order(cells, seed)

    def test_roles_and_one_task_per_joint(self):
        seed = ((1, 1, 0), (1, 1, 1))
        # (1, 2, 1) shares faces with the second seed voxel and with (1, 2, 0).
        cells = [seed[0], seed[1], (1, 2, 0), (1, 2, 1)]
        model = build_gated_order(cells, seed)
        two = [len(partners) for partners in model.joints]
        self.assertIn(2, two)
        times = ActionTimes(
            step_s=1.0, feed_load_s=1.0, handoff_s=1.0, place_s=1.0, fasten_joint_s=10.0
        )
        result = simulate(
            model, n_feeds=1, robots_per_role=1, times=times, stations=((1, 0, 0),)
        )
        roles_for = {
            "load": "cargo",
            "to_feed": "cargo",
            "carry": "cargo",
            "hold": "cargo",
            "handoff": None,
            "to_place": "crane",
            "place": "crane",
            "fasten": FASTEN,
            "joined": FASTEN,
        }
        places = [event for event in result.events if event[2] == "place"]
        joined = [event for event in result.events if event[2] == "joined"]
        loads = [event for event in result.events if event[2] == "load"]
        self.assertEqual(len(places), len(model.order))
        self.assertEqual(len(loads), len(model.order))
        self.assertEqual(len(joined), sum(two))
        for _t, role, action, _index in result.events:
            expected = roles_for[action]
            if expected is not None:
                self.assertEqual(role, expected)
        # The crane never loads and the fastener never places.
        self.assertFalse(any(role == "crane" and action == "load" for _, role, action, _ in result.events))
        self.assertFalse(any(role == FASTEN and action == "place" for _, role, action, _ in result.events))
        # Two joints on one voxel are two fasten completions, not one.
        by_voxel: dict[int, int] = {}
        for _t, _role, action, index in result.events:
            if action == "joined":
                by_voxel[index] = by_voxel.get(index, 0) + 1
        self.assertIn(2, by_voxel.values())

    def test_feed_station_binds_and_then_moves(self):
        model = self._parallel_floor()
        times = ActionTimes(
            step_s=1.0, feed_load_s=10.0, handoff_s=1.0, place_s=1.0, fasten_joint_s=1.0
        )
        stations = lambda n: tuple((0, 0, i) for i in range(n))  # noqa: E731
        one = simulate(model, n_feeds=1, robots_per_role=4, times=times, stations=stations(1))
        four = simulate(model, n_feeds=4, robots_per_role=4, times=times, stations=stations(4))
        single_robot = simulate(
            model, n_feeds=4, robots_per_role=1, times=times, stations=stations(4)
        )
        self.assertIn("feed", one.bottleneck)
        self.assertGreater(one.feed_util, four.feed_util)
        self.assertLess(four.makespan_s, one.makespan_s)
        # Four stations do not help a single cargo robot past a short walk.
        self.assertGreater(single_robot.cargo_util, single_robot.feed_util)
        self.assertNotIn("feed", four.bottleneck)
        for result in (one, four, single_robot):
            self.assertLessEqual(result.cargo_util, 1.0 + 1e-9)
            self.assertLessEqual(result.crane_util, 1.0 + 1e-9)
            self.assertLessEqual(result.fasten_util, 1.0 + 1e-9)
            self.assertLessEqual(result.feed_util, 1.0 + 1e-9)
            self.assertEqual(result.n_placed, len(model.order))
            self.assertEqual(result.n_fasten_tasks, sum(len(j) for j in model.joints))

    def test_baseline_is_deterministic_and_not_the_lab_clock(self):
        model = shelter_model()
        times = ActionTimes()
        result = simulate(model, n_feeds=1, robots_per_role=1, times=times)
        again = simulate(model, n_feeds=1, robots_per_role=1, times=times)
        self.assertEqual(result.makespan_s, again.makespan_s)
        self.assertEqual(result.n_placed, 251)
        self.assertEqual(result.n_fasten_tasks, sum(len(j) for j in model.joints))
        self.assertLessEqual(result.peak_sag, model.finished_sag + SAG_ATOL)
        # Honest comparison: close would be a coincidence, equality is a fit.
        self.assertNotAlmostEqual(result.makespan_h, REPORTED_BUILD_HOURS, delta=5.0)
        self.assertGreater(result.cargo_util, 0.8)
        self.assertEqual(result.bottleneck, "cargo")


class CommittedSweep(unittest.TestCase):
    def test_csv_covers_the_grid_and_matches_the_baseline(self):
        self.assertTrue(CSV_PATH.is_file(), "docs/armadas_sweep.csv is missing")
        with CSV_PATH.open(newline="") as handle:
            rows = list(csv.DictReader(handle))
        self.assertEqual(len(rows), 24)
        keys = {(int(row["n_feeds"]), int(row["robots_per_role"])) for row in rows}
        self.assertEqual(keys, {(f, n) for f in range(1, 5) for n in range(1, 7)})
        for row in rows:
            self.assertEqual(int(row["n_voxels"]), 256)
            self.assertEqual(int(row["n_placed"]), 251)
            self.assertEqual(row["step_source"], STEP_SOURCE)
            self.assertEqual(row["other_action_source"], CONFIG_SOURCE)
            self.assertEqual(row["geometry"], "stand-in-shed-not-fig5")
            self.assertEqual(float(row["k"]), 1.0)
            for name in ("cargo_util", "crane_util", "fasten_util", "feed_util"):
                self.assertLessEqual(float(row[name]), 1.0 + 1e-9)
            self.assertGreater(float(row["makespan_h"]), 0.0)
        baseline = next(
            row for row in rows if row["n_feeds"] == "1" and row["robots_per_role"] == "1"
        )
        model = shelter_model()
        fresh = simulate(model, n_feeds=1, robots_per_role=1)
        self.assertAlmostEqual(float(baseline["makespan_s"]), fresh.makespan_s, places=3)
        # More feeds at six robots does not increase the makespan.
        six = [
            float(row["makespan_h"])
            for row in sorted(rows, key=lambda item: int(item["n_feeds"]))
            if row["robots_per_role"] == "6"
        ]
        self.assertEqual(len(six), 4)
        self.assertGreaterEqual(six[0], six[-1])


if __name__ == "__main__":
    unittest.main()
