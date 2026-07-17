# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Slice 0 tests: the world fills, renders, and finishes.

Deliberately does not import sim.robot or sim.planner — those arrive in later slices
stubs until Slice 1.
"""

from __future__ import annotations

import unittest

from main import build_world, run
from sim.render import render_ascii
from sim.world import EMPTY, GROUND, VOXEL, World


class TestWorld(unittest.TestCase):
    def test_ground_row_and_air(self) -> None:
        w = World(rows=3, cols=4)
        self.assertTrue((w.occupancy[2, :] == GROUND).all())
        self.assertTrue((w.occupancy[:2, :] == EMPTY).all())

    def test_wall_blueprint_sits_on_ground(self) -> None:
        w = build_world()
        self.assertEqual(w.blueprint_count, 24)  # 6 x 4
        # Bottom wall row is directly above the ground row.
        self.assertTrue(w.blueprint[w.rows - 2, 2:8].all())
        # Nothing blueprinted on or below the ground row.
        self.assertFalse(w.blueprint[w.rows - 1, :].any())

    def test_place_voxel_rejects_occupied(self) -> None:
        w = World(rows=3, cols=3)
        w.place_voxel((0, 0))
        self.assertEqual(w.occupancy[0, 0], VOXEL)
        with self.assertRaises(ValueError):
            w.place_voxel((0, 0))
        with self.assertRaises(ValueError):
            w.place_voxel((2, 0))  # ground

    def test_blueprint_cells_bottom_up(self) -> None:
        w = build_world()
        cells = w.blueprint_cells()
        rows = [r for r, _ in cells]
        # Never place a cell above an unvisited lower row: rows non-increasing.
        self.assertEqual(rows, sorted(rows, reverse=True))


class TestRender(unittest.TestCase):
    def test_legend(self) -> None:
        w = World(rows=3, cols=2)
        w.blueprint[1, 0] = True
        w.place_voxel((0, 0))
        lines = render_ascii(w).splitlines()
        self.assertEqual(lines, ["#.", "o.", "=="])


class TestUglyLoop(unittest.TestCase):
    def test_one_cell_per_tick_to_completion(self) -> None:
        w = build_world()
        log = run(w, verbose=False)
        self.assertTrue(w.complete)
        # Tick 0 initial record, then exactly one voxel placed per tick.
        self.assertEqual(log.ticks, w.blueprint_count)
        placed = [rec.placed for rec in log.records]
        self.assertEqual(placed, list(range(0, w.blueprint_count + 1)))
        self.assertEqual(log.records[-1].progress, 1.0)


if __name__ == "__main__":
    unittest.main()
