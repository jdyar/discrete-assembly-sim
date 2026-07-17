# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Reservation table: lease/deed semantics, conflicts, release, GC."""

import unittest

from sim.reservations import ReservationConflict, ReservationTable


class TestLeases(unittest.TestCase):
    def test_path_lease_occupies_slots(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a", "b", "c"], t0=5)
        self.assertFalse(rt.is_free("a", 5))
        self.assertFalse(rt.is_free("b", 6))
        self.assertFalse(rt.is_free("c", 7))
        # Same node at other ticks stays free — leases are per-tick.
        self.assertTrue(rt.is_free("b", 5))
        self.assertTrue(rt.is_free("b", 7))

    def test_owner_may_reuse_own_slot(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a"], t0=0)
        self.assertTrue(rt.is_free("a", 0, owner="r1"))
        self.assertFalse(rt.is_free("a", 0, owner="r2"))

    def test_node_conflict_rejected_atomically(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a", "b"], t0=0)
        # r2's path is clear at tick 0 but collides at b@1: nothing sticks.
        with self.assertRaises(ReservationConflict):
            rt.reserve_path("r2", ["x", "b"], t0=0)
        self.assertTrue(rt.is_free("x", 0))  # atomicity: first slot rolled back

    def test_swap_conflict_rejected(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a", "b"], t0=0)  # a->b during step 0
        # Head-on swap b->a at the same step: node slots differ but the
        # undirected edge is taken.
        with self.assertRaises(ReservationConflict):
            rt.reserve_path("r2", ["b", "a"], t0=0)

    def test_wait_step_is_not_an_edge(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a", "a", "b"], t0=0)  # waits at a, then moves
        # r2 crossing the a-b edge at step 0 is fine (r1 only waited then).
        self.assertTrue(rt.edge_free("a", "b", 0, owner="r2"))
        self.assertFalse(rt.edge_free("a", "b", 1, owner="r2"))

    def test_hold_pins_interval(self):
        rt = ReservationTable()
        rt.reserve_hold("r1", "a", t0=3, t1=5)
        for t in (3, 4, 5):
            self.assertFalse(rt.is_free("a", t))
        self.assertTrue(rt.is_free("a", 6))


class TestDeeds(unittest.TestCase):
    def test_deed_solid_from_commit_forever(self):
        rt = ReservationTable()
        rt.reserve_deed("r1", "cell", t_commit=10)
        self.assertTrue(rt.is_free("cell", 9))  # walk through before commit
        for t in (10, 11, 1000):
            self.assertFalse(rt.is_free("cell", t, owner="r1"))  # even the placer

    def test_deed_conflicts_with_other_owners_future_lease(self):
        rt = ReservationTable()
        rt.reserve_path("r2", ["cell"], t0=12)
        with self.assertRaises(ReservationConflict):
            rt.reserve_deed("r1", "cell", t_commit=10)

    def test_deed_tolerates_own_lease_and_earlier_traffic(self):
        rt = ReservationTable()
        rt.reserve_path("r2", ["cell"], t0=3)  # passes through before commit
        rt.reserve_path("r1", ["cell"], t0=9)  # the placer approaching
        rt.reserve_deed("r1", "cell", t_commit=10)
        self.assertEqual(rt.deed_holder("cell"), ("r1", 10))

    def test_double_deed_rejected(self):
        rt = ReservationTable()
        rt.reserve_deed("r1", "cell", t_commit=1)
        with self.assertRaises(ReservationConflict):
            rt.reserve_deed("r2", "cell", t_commit=5)

    def test_permanent_nodes_at(self):
        rt = ReservationTable()
        rt.reserve_deed("r1", "early", t_commit=2)
        rt.reserve_deed("r1", "late", t_commit=8)
        self.assertEqual(rt.permanent_nodes_at(1), set())
        self.assertEqual(rt.permanent_nodes_at(2), {"early"})
        self.assertEqual(rt.permanent_nodes_at(8), {"early", "late"})

    def test_clear_deed(self):
        rt = ReservationTable()
        rt.reserve_deed("r1", "cell", t_commit=1)
        rt.clear_deed("cell")
        self.assertTrue(rt.is_free("cell", 5))
        with self.assertRaises(KeyError):
            rt.clear_deed("cell")


class TestReleaseAndGC(unittest.TestCase):
    def test_release_owner_drops_future_keeps_past_and_deeds(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a", "b", "c"], t0=0)
        rt.reserve_deed("r1", "placed", t_commit=2)
        rt.release_owner("r1", from_t=1)
        self.assertFalse(rt.is_free("a", 0))  # past lease stands
        self.assertTrue(rt.is_free("b", 1))  # future leases dropped
        self.assertTrue(rt.is_free("c", 2))
        self.assertFalse(rt.edge_free("a", "b", 0, owner="r2"))  # past edge stands
        # edge (b,c) at step 1 was future — released with the leases.
        self.assertTrue(rt.edge_free("b", "c", 1, owner="r2"))
        self.assertEqual(rt.deed_holder("placed"), ("r1", 2))  # deeds survive

    def test_release_does_not_touch_other_owners(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a"], t0=5)
        rt.reserve_path("r2", ["b"], t0=5)
        rt.release_owner("r1", from_t=0)
        self.assertTrue(rt.is_free("a", 5))
        self.assertFalse(rt.is_free("b", 5))

    def test_advance_collects_expired_leases_never_deeds(self):
        rt = ReservationTable()
        rt.reserve_path("r1", ["a", "b"], t0=0)
        rt.reserve_deed("r1", "cell", t_commit=0)
        rt.advance_to(2)
        self.assertTrue(rt.is_free("a", 0))  # collected
        self.assertFalse(rt.is_free("cell", 1000))  # deed permanent

    def test_empty_path_rejected(self):
        rt = ReservationTable()
        with self.assertRaises(ValueError):
            rt.reserve_path("r1", [], t0=0)


if __name__ == "__main__":
    unittest.main()
