# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Cubic 3D lattice geometry + the parameterized motion model (Slice 4a).

BILL-E/SOLL-E-class surface locomotion generalized to 3D, behind the
unchanged :class:`sim.geometry.Geometry` interface — the choreographer,
planner, and reservation table run on these nodes without modification
(binding rule, docs/DESIGN.md).

Motion model (docs/DESIGN.md, Slice 4c direction): real relative robots
are not one-cell steppers — SOLL-E takes multi-voxel inchworm strides,
BILL-E-class robots place parts several cells away and cooperate to
extend reach (NTRS 20170006219; US10046820B2). Those capabilities are
CONFIG on this geometry, never choreographer changes:

- ``reach_radius`` — how far (Chebyshev) a stance can place/remove.
  Radius 1 is the conservative single-cell reach; radius up to ~4 models
  the observed placement span of current hardware. The Chebyshev ball is
  symmetric, so the planner's stance-inversion contract holds for every
  radius, and it is deliberately line-of-sight-free: a lattice robot's
  end effector articulates around corners (reach is along the structure,
  not through the air), so corner cells are NOT excluded.
- Future fields land here, not in the choreographer: stride (multi-cell
  inchworm steps = extra ``neighbors`` edges), climb limit, and
  coupled-robot extended reach (two robots attached = one stance with a
  larger radius).

Articulation rule (decided by Joshua 2026-07-16, implemented Slice 4c):
**snake arm** — a target is reachable iff a path of Chebyshev steps of
length <= reach_radius runs from the stance to the target whose
INTERMEDIATE cells are all empty (endpoints exempt: the stance is the
robot's cell; the target may be solid, for inspect/remove). The arm
articulates around corners and over ledges, matching observed
BILL-E/SOLL-E-class arm behavior; it never passes through solid volume.
At radius 1 there are no intermediates, so the rule reduces exactly to
the 26-cell shell — the machine-verified 2D convention lifted to 3D.
The relation is symmetric (the same path reversed), preserving the
planner's stance-inversion contract at every radius.
Also decided 2026-07-16: the cubic voxel grid stands (see world3d.py).
"""

from __future__ import annotations

from dataclasses import dataclass
from itertools import product
from typing import Iterator

from .geometry import Geometry, Node
from .world import DEFECT, EMPTY, GROUND, VOXEL
from .world3d import World3D

_AXES = ((1, 0, 0), (0, 1, 0), (0, 0, 1))


@dataclass(frozen=True)
class MotionModel:
    """Robot motion capabilities as data. Defaults = the conservative
    one-cell surface stepper; real systems override (Slice 4c)."""

    reach_radius: int = 1


class CubicLattice3D(Geometry):
    """The cubic voxel lattice under 3D surface-locomotion rules.

    Nodes are ``(level, row, col)``; level 0 is the ground plane, up is
    +level. Rules are the 2D BILL-E set (decided 2026-07-05, NOTES.md)
    generalized per axis-plane:

    - **Footing**: any empty in-bounds cell face-adjacent (6-neighborhood)
      to at least one solid (GROUND/VOXEL/DEFECT — a defective voxel is
      badly bonded but crawlable).
    - **Movement**: orthogonal to an adjacent footing cell, or an
      in-plane diagonal when rounding a corner — exactly one of the two
      between-cells solid (a corner to pivot around, not a gap to cross
      or a pinch to squeeze through). The rule applies in all three
      axis-planes, so it covers both walking around a wall corner and
      climbing over a ledge.
    - **Reach**: the Chebyshev ball of ``motion.reach_radius`` around the
      stance (symmetric; see module docstring).
    """

    _SOLIDS = (GROUND, VOXEL, DEFECT)

    def __init__(self, world: World3D, motion: MotionModel = MotionModel()) -> None:
        self.world = world
        self.motion = motion

    def _in_bounds(self, node) -> bool:
        l, r, c = node
        return (
            0 <= l < self.world.levels
            and 0 <= r < self.world.rows
            and 0 <= c < self.world.cols
        )

    def _solid(self, node) -> bool:
        return self._in_bounds(node) and self.world.occupancy[node] in self._SOLIDS

    def is_footing(self, node) -> bool:
        if not self._in_bounds(node):
            return False
        if self.world.occupancy[node] != EMPTY:  # DEFECT counts as occupied
            return False
        l, r, c = node
        return any(
            self._solid((l + s * dl, r + s * dr, c + s * dc))
            for dl, dr, dc in _AXES
            for s in (1, -1)
        )

    def neighbors(self, node) -> Iterator[Node]:
        l, r, c = node
        for axis in _AXES:
            for sign in (1, -1):
                nxt = (l + sign * axis[0], r + sign * axis[1], c + sign * axis[2])
                if self.is_footing(nxt):
                    yield nxt
        # In-plane corner rounding: for each pair of axes, pivot around
        # exactly one solid between-cell (same rule as 2D, per plane).
        for i in range(3):
            for j in range(i + 1, 3):
                ai, aj = _AXES[i], _AXES[j]
                for si, sj in product((1, -1), repeat=2):
                    between_i = (l + si * ai[0], r + si * ai[1], c + si * ai[2])
                    between_j = (l + sj * aj[0], r + sj * aj[1], c + sj * aj[2])
                    nxt = (
                        l + si * ai[0] + sj * aj[0],
                        r + si * ai[1] + sj * aj[1],
                        c + si * ai[2] + sj * aj[2],
                    )
                    if self.is_footing(nxt) and (
                        self._solid(between_i) != self._solid(between_j)
                    ):
                        yield nxt

    def reach_cells(self, node) -> list[Node]:
        """Snake-arm reach: BFS over Chebyshev steps from ``node``,
        depth <= reach_radius, where every intermediate cell must be
        empty. Every cell one step off the empty-path tree is a target
        (solid targets allowed — that is how inspect/remove reach a
        placed voxel). At radius 1 this is exactly the 26-cell shell.
        """
        R = self.motion.reach_radius
        l0, r0, c0 = node
        steps = [d for d in product((-1, 0, 1), repeat=3) if d != (0, 0, 0)]
        if R == 1:
            return [(l0 + dl, r0 + dr, c0 + dc) for dl, dr, dc in steps]
        seen_empty = {node}  # cells the arm may pass THROUGH
        frontier = [node]
        targets: set[Node] = set()
        for _depth in range(R):
            nxt = []
            for l, r, c in frontier:
                for dl, dr, dc in steps:
                    cell = (l + dl, r + dr, c + dc)
                    if cell == node or cell in targets:
                        continue
                    targets.add(cell)
                    # Only empty in-bounds cells extend the arm's path.
                    if (
                        cell not in seen_empty
                        and self._in_bounds(cell)
                        and self.world.occupancy[cell] == EMPTY
                    ):
                        seen_empty.add(cell)
                        nxt.append(cell)
            frontier = nxt
        return list(targets)

    def candidate_sort_key(self, target, depot) -> tuple:
        # Supported-first (nothing floats unless it must), bottom-up,
        # farthest-from-depot first — keeps a climbable ramp depot-side.
        l, r, c = target
        supported = l > 0 and self.world.occupancy[l - 1, r, c] in (GROUND, VOXEL)
        horiz = max(abs(r - depot[1]), abs(c - depot[2]))
        return (not supported, l, -horiz)

    def bound_factory(self):
        motion = self.motion
        return lambda world: CubicLattice3D(world, motion)

    def heuristic_distance(self, a, b) -> int:
        # Chebyshev: every move (orthogonal or in-plane corner diagonal)
        # changes each coordinate by at most 1 — never overestimates.
        return max(abs(a[0] - b[0]), abs(a[1] - b[1]), abs(a[2] - b[2]))

    def future_view(self, solid_nodes) -> "CubicLattice3D":
        clone = self.world.copy()
        for node in solid_nodes:
            if clone.occupancy[node] == EMPTY:
                clone.occupancy[node] = VOXEL
        return CubicLattice3D(clone, self.motion)
