# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Pluggable lattice geometry — the seam the multi-lattice roadmap swaps.

Binding rule (docs/DESIGN.md): the
choreographer, planner, and reservation table never assume 2D or
square-lattice geometry. Everything lattice-specific — what a node is,
which nodes a robot can stand on, how it moves between them, where it can
place — lives behind this interface. Nodes are opaque hashable values;
only a Geometry implementation may look inside one.

Slice 3a ships :class:`SquareLattice2D`, the existing BILL-E
surface-locomotion rules (decided 2026-07-05, machine-verified — NOTES.md)
moved here verbatim from ``sim.robot``. the multi-lattice roadmap implements the
cuboctahedral lattice behind this same interface — not a port.
"""

from __future__ import annotations

from collections import deque
from typing import Hashable, Iterable, Iterator

from .world import DEFECT, EMPTY, GROUND, VOXEL, World

Node = Hashable


class Geometry:
    """What a lattice must answer about robot placement and movement.

    Required interface (per docs/DESIGN.md):

    - :meth:`neighbors` — movement edges: nodes reachable in one tick.
    - :meth:`is_footing` — may a robot occupy this node right now?
    - :meth:`reach_cells` — nodes a robot standing here can place into
      (or remove from).

    The planner derives placement stances for a target by inverting reach:
    ``{a in reach_cells(target) if is_footing(a) and target in
    reach_cells(a)}``. This is only complete when the reach relation is
    SYMMETRIC — implementations must keep it so (true for the square
    lattice's 8-neighborhood and the cuboct face-adjacency); if a future
    lattice breaks symmetry, extend this interface with an explicit
    ``approaches_for`` instead of patching call sites.

    Implementations are bound to a live world: answers reflect current
    occupancy and change as voxels are placed/removed.
    """

    def neighbors(self, node: Node) -> Iterator[Node]:
        raise NotImplementedError

    def is_footing(self, node: Node) -> bool:
        raise NotImplementedError

    def reach_cells(self, node: Node) -> list[Node]:
        raise NotImplementedError

    # -- planner heuristic hook (optional override) --------------------------

    def candidate_sort_key(self, target: Node, depot: Node) -> tuple:
        """Ordering hint for the build-order search: lower keys tried first.

        Lattice-specific knowledge (gravity direction, distance shape)
        belongs here, not in the planner. The default expresses no
        preference; the search stays correct, just slower.
        """
        return ()

    def heuristic_distance(self, a: Node, b: Node) -> int:
        """Admissible lower bound on ticks to move a -> b (A* heuristic).

        Must never overestimate. The default (0) degrades A* to Dijkstra —
        correct on any lattice, just slower.
        """
        return 0

    def bound_factory(self):
        """A ``world -> Geometry`` factory reproducing THIS geometry's
        configuration (motion model, etc.) on another world. Callers that
        re-run the build-order search on a projected world (e.g. the
        buildability gate) must use this, never a default factory — a
        default silently reintroduces lattice assumptions.
        """
        return type(self)

    def future_view(self, solid_nodes: Iterable[Node]) -> "Geometry":
        """A geometry as if ``solid_nodes`` were already built.

        The choreographer's connectivity gate runs over this: world solids
        plus deeded-but-unplaced voxels (ReservationTable.permanent_nodes_at).
        Only the lattice knows how added solids change footing/movement, so
        the projection lives here. Must NOT mutate the live world.
        """
        raise NotImplementedError


def bfs_path(
    geometry: Geometry, start: Node, goals: Iterable[Node]
) -> list[Node] | None:
    """Shortest footing-node path from ``start`` into ``goals`` (BFS).

    Geometry-agnostic: edges come from :meth:`Geometry.neighbors` only.
    Returns the path including ``start`` and the reached goal, or ``None``
    if no goal is reachable. ``start`` must itself be a legal footing node.
    """
    goals = set(goals)
    if start in goals:
        return [start]
    prev: dict[Node, Node] = {start: start}
    queue: deque[Node] = deque([start])
    while queue:
        cur = queue.popleft()
        for nxt in geometry.neighbors(cur):
            if nxt in prev:
                continue
            prev[nxt] = cur
            if nxt in goals:
                path = [nxt]
                while path[-1] != start:
                    path.append(prev[path[-1]])
                path.reverse()
                return path
            queue.append(nxt)
    return None


def bfs_reachable(geometry: Geometry, start: Node) -> set:
    """All footing nodes reachable from ``start`` (BFS over neighbors).

    Membership in the result is equivalent to ``bfs_path(geometry, start,
    {node}) is not None`` — computed once instead of per-goal (the
    planner's candidate loop was profiled at ~93k per-goal BFS runs per
    3D build; one reachable-set per world state replaces them all).
    """
    seen = {start}
    frontier = [start]
    while frontier:
        nxt = []
        for node in frontier:
            for nb in geometry.neighbors(node):
                if nb not in seen:
                    seen.add(nb)
                    nxt.append(nb)
        frontier = nxt
    return seen


class SquareLattice2D(Geometry):
    """The 2D square lattice under BILL-E surface-locomotion rules.

    Nodes are ``(row, col)`` tuples in numpy convention (ground = bottom
    row). Rules moved verbatim from sim.robot (decided 2026-07-05):

    - **Footing**: any empty in-bounds cell 4-adjacent to at least one
      solid (GROUND/VOXEL/DEFECT — a defective voxel is badly bonded but
      crawlable; it cannot be occupied, only gripped).
    - **Movement**: orthogonal to an adjacent footing cell, or diagonal
      when rounding a corner — exactly one of the two between-cells solid
      (a corner to pivot around, not a gap to cross or a pinch to squeeze
      through).
    - **Reach**: the 8 cells surrounding the stance (symmetric).
    """

    _SOLIDS = (GROUND, VOXEL, DEFECT)

    def __init__(self, world: World) -> None:
        self.world = world

    def _solid(self, node) -> bool:
        r, c = node
        return (
            0 <= r < self.world.rows
            and 0 <= c < self.world.cols
            and self.world.occupancy[r, c] in self._SOLIDS
        )

    def is_footing(self, node) -> bool:
        r, c = node
        if not (0 <= r < self.world.rows and 0 <= c < self.world.cols):
            return False
        if self.world.occupancy[r, c] != EMPTY:  # DEFECT counts as occupied
            return False
        return any(
            self._solid(n)
            for n in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1))
        )

    def neighbors(self, node) -> Iterator[Node]:
        r, c = node
        for nxt in ((r + 1, c), (r - 1, c), (r, c + 1), (r, c - 1)):
            if self.is_footing(nxt):
                yield nxt
        for dr in (-1, 1):
            for dc in (-1, 1):
                nxt = (r + dr, c + dc)
                # Corner rounding: pivot around exactly one solid between-cell.
                if self.is_footing(nxt) and (
                    self._solid((r + dr, c)) != self._solid((r, c + dc))
                ):
                    yield nxt

    def reach_cells(self, node) -> list[Node]:
        r, c = node
        return [
            (r + dr, c + dc)
            for dr in (-1, 0, 1)
            for dc in (-1, 0, 1)
            if (dr, dc) != (0, 0)
        ]

    def candidate_sort_key(self, target, depot) -> tuple:
        # Supported-first (nothing floats unless it must), bottom-up,
        # farthest-from-depot first — keeps a climbable ramp depot-side.
        r, c = target
        supported = (
            r + 1 < self.world.rows
            and self.world.occupancy[r + 1, c] in (GROUND, VOXEL)
        )
        return (not supported, -r, -abs(c - depot[1]))

    def heuristic_distance(self, a, b) -> int:
        # Chebyshev: every move (orthogonal or corner diagonal) changes
        # each coordinate by at most 1, so this never overestimates.
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]))

    def future_view(self, solid_nodes) -> "SquareLattice2D":
        clone = self.world.copy()
        for r, c in solid_nodes:
            if clone.occupancy[r, c] == EMPTY:
                clone.occupancy[r, c] = VOXEL
        return SquareLattice2D(clone)
