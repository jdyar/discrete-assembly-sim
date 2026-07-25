# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Reservation table: who holds which lattice node at which tick.

Clean-room implementation of the node-reservation machinery described in
the ARMADAS system paper's planning supplementary (Gregg et al., Science
Robotics 2024, author-accepted manuscript NTRS 20230005194; see
docs/DESIGN.md, Provenance): robot
plans are coordinated over a time-expanded graph via reservations that
make collisions and deadlocks impossible by construction. No ARMADAS code
exists to consult; this is built from the published method only.

Implementation note (within their published framework, not novel — see
docs/DESIGN.md, Provenance): we distinguish two reservation kinds.

- **Lease** — movement. A robot holds a node (or edge) for exactly one
  tick and the hold expires with time. Leases are released wholesale when
  a robot replans (Slice 3c: repair replanning drops a robot's future
  leases while everyone else's stand).
- **Deed** — placement. Once a placement is committed, the target node is
  solid from ``t_commit`` forever: no robot may ever occupy it, and it
  joins the *permanent future graph* the entrapment check runs
  connectivity over. Deeds survive replans; only an explicit
  :meth:`clear_deed` (Slice 3c defect removal) retracts one.

Nodes are opaque hashables (Geometry contract — no 2D assumptions here).
The table is time-slotted per tick; the facade API is the stable surface,
so the slotted internals can become interval-based later without touching
the choreographer, the entrapment check, or the tests' consumers.

Scope note (Slice 3a day 1): data structures only. The cooperative A*
that populates this table is the next step, not this module.
"""

from __future__ import annotations

from typing import Hashable, Iterable, Sequence

Node = Hashable
OwnerId = Hashable


class ReservationConflict(Exception):
    """A write would overlap an existing lease or deed."""


def _edge_key(u: Node, v: Node, t: int) -> tuple:
    """Canonical undirected key: traversing u->v and v->u at tick t collide."""
    a, b = sorted((u, v), key=repr)
    return (a, b, t)


class ReservationTable:
    """Tick-slotted lease/deed table behind a semantic facade.

    Time convention: a lease on ``(node, t)`` means the owner occupies
    ``node`` during tick ``t``. An edge lease on ``(u, v, t)`` means the
    owner traverses between ``u`` and ``v`` in the step from tick ``t``
    to ``t + 1`` — held undirected, so head-on swaps conflict.

    Writes are atomic: every slot is checked before any is taken, so a
    failed reservation leaves the table untouched.
    """

    def __init__(self) -> None:
        self._leases: dict[tuple[Node, int], OwnerId] = {}
        self._edges: dict[tuple, OwnerId] = {}
        self._deeds: dict[Node, tuple[OwnerId, int]] = {}
        self._now = 0  # earliest tick still queryable; advance_to() moves it

    # -- queries -------------------------------------------------------------

    def is_free(self, node: Node, t: int, owner: OwnerId | None = None) -> bool:
        """May ``owner`` occupy ``node`` during tick ``t``?

        False if another owner leases the slot, or the node is deeded
        solid by tick ``t``. A deeded node is free to pass through at
        ticks BEFORE its commit tick. ``owner=None`` asks "is it free for
        anyone new".
        """
        if node in self._deeds and t >= self._deeds[node][1]:
            return False
        holder = self._leases.get((node, t))
        return holder is None or holder == owner

    def edge_free(
        self, u: Node, v: Node, t: int, owner: OwnerId | None = None
    ) -> bool:
        """May ``owner`` traverse between ``u`` and ``v`` at step ``t``?"""
        holder = self._edges.get(_edge_key(u, v, t))
        return holder is None or holder == owner

    def deed_holder(self, node: Node) -> tuple[OwnerId, int] | None:
        """(owner, t_commit) if ``node`` is deeded, else None."""
        return self._deeds.get(node)

    def permanent_nodes_at(self, t: int) -> set[Node]:
        """Nodes deeded solid by tick ``t`` — the reservation table's
        contribution to the permanent future graph. The entrapment check
        unions this with the world's existing solids (via the Geometry);
        the table itself knows nothing about the lattice.
        """
        return {n for n, (_o, tc) in self._deeds.items() if tc <= t}

    def owner_leases(self, owner: OwnerId) -> list[tuple[Node, int]]:
        """This owner's node leases, soonest first (introspection/tests)."""
        slots = [(t, n) for (n, t), o in self._leases.items() if o == owner]
        return [(n, t) for t, n in sorted(slots, key=lambda x: (x[0], repr(x[1])))]

    # -- writes --------------------------------------------------------------

    def reserve_path(self, owner: OwnerId, path: Sequence[Node], t0: int) -> None:
        """Lease a tick-per-step path: ``path[i]`` during ``t0 + i``, plus
        the undirected edge for every step. Atomic; raises
        :class:`ReservationConflict` (table unchanged) on any overlap.

        Persistence policy — what happens after the path ends — is the
        choreographer's decision, not the table's: use
        :meth:`reserve_hold` to pin the final node for a horizon.
        """
        self._check_path(owner, path, t0)
        for i, node in enumerate(path):
            self._leases[(node, t0 + i)] = owner
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            if u != v:  # waiting in place is not an edge traversal
                self._edges[_edge_key(u, v, t0 + i)] = owner

    def reserve_hold(self, owner: OwnerId, node: Node, t0: int, t1: int) -> None:
        """Lease ``node`` for every tick in ``[t0, t1]`` (inclusive). Atomic."""
        for t in range(t0, t1 + 1):
            if not self.is_free(node, t, owner):
                raise ReservationConflict(f"{node!r}@{t} not free for {owner!r}")
        for t in range(t0, t1 + 1):
            self._leases[(node, t)] = owner

    def reserve_hold_best_effort(
        self, owner: OwnerId, node: Node, t0: int, t1: int
    ) -> int:
        """Lease ``node`` from ``t0`` up to ``t1``, stopping at the first
        conflict. Returns the last tick actually held (``t0 - 1`` if
        none). Used for trailing park-holds: coverage is desirable, not
        mandatory — the owner's rolling renewal takes over next tick."""
        last = t0 - 1
        for t in range(t0, t1 + 1):
            if not self.is_free(node, t, owner):
                break
            self._leases[(node, t)] = owner
            last = t
        return last

    def reserve_deed(self, owner: OwnerId, node: Node, t_commit: int) -> None:
        """Commit a placement: ``node`` is solid from ``t_commit`` forever.

        Conflicts with any existing deed, and with any OTHER owner's lease
        at or after ``t_commit`` (a robot standing there when the voxel
        arrives). The placing owner's own leases up to commit are fine.
        """
        if node in self._deeds:
            raise ReservationConflict(f"{node!r} already deeded")
        for (n, t), holder in self._leases.items():
            if n == node and t >= t_commit and holder != owner:
                raise ReservationConflict(
                    f"deed {node!r}@{t_commit} overlaps {holder!r} lease @{t}"
                )
        self._deeds[node] = (owner, t_commit)

    def clear_deed(self, node: Node) -> None:
        """Retract a deed (Slice 3c: defective voxel removed from ``node``)."""
        if node not in self._deeds:
            raise KeyError(f"{node!r} holds no deed")
        del self._deeds[node]

    def release_owner(self, owner: OwnerId, from_t: int) -> None:
        """Drop ``owner``'s leases (nodes and edges) at ``t >= from_t``.

        Deeds stand — committed structure is not un-placed by a replan.
        This is the replan primitive: cancel a robot's future, keep its
        past and its placements.
        """
        self._leases = {
            k: o
            for k, o in self._leases.items()
            if not (o == owner and k[1] >= from_t)
        }
        self._edges = {
            k: o
            for k, o in self._edges.items()
            if not (o == owner and k[2] >= from_t)
        }

    def advance_to(self, t: int) -> None:
        """Garbage-collect leases strictly before tick ``t``. Deeds are
        permanent and never collected."""
        self._now = max(self._now, t)
        self._leases = {k: o for k, o in self._leases.items() if k[1] >= t}
        self._edges = {k: o for k, o in self._edges.items() if k[2] >= t}

    # -- internals -------------------------------------------------------------

    def _check_path(self, owner: OwnerId, path: Sequence[Node], t0: int) -> None:
        if not path:
            raise ValueError("empty path")
        for i, node in enumerate(path):
            if not self.is_free(node, t0 + i, owner):
                raise ReservationConflict(
                    f"{node!r}@{t0 + i} not free for {owner!r}"
                )
        for i in range(len(path) - 1):
            u, v = path[i], path[i + 1]
            if u != v and not self.edge_free(u, v, t0 + i, owner):
                raise ReservationConflict(
                    f"edge {u!r}->{v!r}@{t0 + i} not free for {owner!r}"
                )
