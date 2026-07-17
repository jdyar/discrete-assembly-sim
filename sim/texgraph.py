# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Time-expanded graph over (node, tick) — the choreographer's search space.

Clean-room from the ARMADAS system paper's planning description (Gregg et
al., Science Robotics 2024, NTRS 20230005194; docs/DESIGN.md,
Provenance): multi-robot build plans are found with cooperative
A* / multi-label A* over a **time-expanded graph of discrete robot
poses**, where reservations prune conflicting states. This module is that
graph: states are ``(node, t)``, edges are *wait* (stay put one tick) and
*move* (one geometry step), both gated by the reservation table.

Lattice-agnostic by construction (binding rule, docs/DESIGN.md):
adjacency comes only from ``Geometry.neighbors``; nodes are opaque. The
tests exercise this graph on a non-square fake lattice to keep it honest.

Scope note (Slice 3a day 1): the search space only. Cooperative A* over
it is the next step, not this module.
"""

from __future__ import annotations

from typing import Hashable, Iterator, NamedTuple

from .geometry import Geometry
from .reservations import OwnerId, ReservationTable

Node = Hashable


class TENode(NamedTuple):
    """One state in the time-expanded graph: at ``node`` during tick ``t``."""

    node: Node
    t: int


class TimeExpandedGraph:
    """Successor generation over (node, tick), reservation- and
    geometry-gated.

    A successor of ``(n, t)`` is ``(n', t + 1)`` where either ``n' == n``
    (wait) or ``n'`` is a geometry neighbor of ``n`` (move), and:

    - the destination slot is free for ``owner`` in the reservation table
      (no other robot's lease; not deeded solid by ``t + 1``), and
    - for moves, the undirected edge at step ``t`` is free (no head-on
      swap with another robot), and
    - for waits, ``n`` must still be legal footing (the graph does not
      re-check footing on moves — ``Geometry.neighbors`` already yields
      only footing nodes).

    Footing is evaluated against the geometry's CURRENT world. Planning
    against future world states (structure that a deed says will exist)
    is choreographer logic layered on top — deliberately not decided in
    this skeleton.
    """

    def __init__(
        self,
        geometry: Geometry,
        reservations: ReservationTable,
        owner: OwnerId,
        static_obstacles: frozenset | set = frozenset(),
    ) -> None:
        self.geometry = geometry
        self.reservations = reservations
        self.owner = owner
        # Positions of robots that currently have NO plan. Their rolling
        # holds expire, but the robot does not: planning through such a
        # cell on an expiry gamble is how collisions happen. They are
        # walls until they plan (displacement is explicit, never implied).
        self.static_obstacles = frozenset(static_obstacles)

    def successors(self, state: TENode) -> Iterator[TENode]:
        n, t = state
        nxt_t = t + 1
        # Wait: hold position for one tick.
        if self.geometry.is_footing(n) and self.reservations.is_free(
            n, nxt_t, self.owner
        ):
            yield TENode(n, nxt_t)
        # Move: one geometry step; destination free now and forever-free
        # of parked robots; edge free (no head-on swap).
        for nbr in self.geometry.neighbors(n):
            if (
                nbr not in self.static_obstacles
                and self.reservations.is_free(nbr, nxt_t, self.owner)
                and self.reservations.edge_free(n, nbr, t, self.owner)
            ):
                yield TENode(nbr, nxt_t)
