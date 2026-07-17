# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Time-expanded graph: wait/move successors, reservation gating, and
lattice-agnosticism (exercised on a deliberately non-square fake lattice)."""

import unittest

from sim.geometry import Geometry, SquareLattice2D
from sim.reservations import ReservationTable
from sim.texgraph import TENode, TimeExpandedGraph
from sim.world import World


class TriangleLattice(Geometry):
    """Fake non-square lattice: three nodes named by strings, fully
    connected, everything footing. If the graph code indexes into nodes
    or assumes (row, col) tuples, these string nodes break it."""

    NODES = ("alpha", "beta", "gamma")

    def neighbors(self, node):
        return (n for n in self.NODES if n != node)

    def is_footing(self, node):
        return node in self.NODES

    def reach_cells(self, node):
        return [n for n in self.NODES if n != node]


class TestSuccessorsOnFakeLattice(unittest.TestCase):
    def setUp(self):
        self.rt = ReservationTable()
        self.graph = TimeExpandedGraph(TriangleLattice(), self.rt, owner="r1")

    def test_wait_and_moves_unreserved(self):
        succ = set(self.graph.successors(TENode("alpha", 0)))
        self.assertEqual(
            succ,
            {TENode("alpha", 1), TENode("beta", 1), TENode("gamma", 1)},
        )

    def test_leased_destination_pruned(self):
        self.rt.reserve_path("r2", ["beta"], t0=1)
        succ = set(self.graph.successors(TENode("alpha", 0)))
        self.assertNotIn(TENode("beta", 1), succ)
        self.assertIn(TENode("gamma", 1), succ)

    def test_own_lease_not_pruned(self):
        self.rt.reserve_path("r1", ["beta"], t0=1)
        succ = set(self.graph.successors(TENode("alpha", 0)))
        self.assertIn(TENode("beta", 1), succ)

    def test_swap_edge_pruned(self):
        # r2 traverses beta->alpha during step 0; r1 alpha->beta collides.
        self.rt.reserve_path("r2", ["beta", "alpha"], t0=0)
        succ = set(self.graph.successors(TENode("alpha", 0)))
        self.assertNotIn(TENode("beta", 1), succ)
        # Waiting and the other move are unaffected... except alpha@1 is
        # r2's destination slot, so only gamma remains.
        self.assertEqual(succ, {TENode("gamma", 1)})

    def test_deeded_node_pruned_from_commit(self):
        self.rt.reserve_deed("r2", "beta", t_commit=1)
        succ = set(self.graph.successors(TENode("alpha", 0)))
        self.assertNotIn(TENode("beta", 1), succ)  # solid at arrival tick
        # But traffic strictly before commit is legal:
        self.rt.clear_deed("beta")
        self.rt.reserve_deed("r2", "beta", t_commit=5)
        succ = set(self.graph.successors(TENode("alpha", 0)))
        self.assertIn(TENode("beta", 1), succ)


class TestSuccessorsOnSquareLattice(unittest.TestCase):
    """The real 2D lattice through the same graph code path."""

    def setUp(self):
        # 4 rows x 4 cols: ground row at r=3, one voxel at (2,1).
        self.world = World(4, 4)
        self.world.place_voxel((2, 1))
        self.rt = ReservationTable()
        self.graph = TimeExpandedGraph(
            SquareLattice2D(self.world), self.rt, owner="r1"
        )

    def test_wait_requires_footing(self):
        # (2,0) grips the voxel at (2,1): wait is a successor.
        succ = set(self.graph.successors(TENode((2, 0), 0)))
        self.assertIn(TENode((2, 0), 1), succ)
        # A mid-air node yields no wait (and neighbors() already yields
        # only footing, so any successors are real moves).
        airborne = set(self.graph.successors(TENode((0, 3), 0)))
        self.assertNotIn(TENode((0, 3), 1), airborne)

    def test_moves_match_geometry(self):
        geom = SquareLattice2D(self.world)
        start = (2, 0)  # footing: grips the voxel at (2,1) and ground below
        succ = set(self.graph.successors(TENode(start, 0)))
        expected = {TENode(n, 1) for n in geom.neighbors(start)}
        expected.add(TENode(start, 1))  # plus wait
        self.assertEqual(succ, expected)

    def test_reservation_gates_apply_on_real_lattice(self):
        start = (2, 0)
        geom = SquareLattice2D(self.world)
        nbrs = list(geom.neighbors(start))
        self.assertTrue(nbrs)
        blocked = nbrs[0]
        self.rt.reserve_path("r2", [blocked], t0=1)
        succ = set(self.graph.successors(TENode(start, 0)))
        self.assertNotIn(TENode(blocked, 1), succ)


if __name__ == "__main__":
    unittest.main()
