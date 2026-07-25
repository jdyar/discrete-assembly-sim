# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Bring-your-own-lattice, worked example: a hexagonal lattice.

This lattice does not exist anywhere in the core library — it is
implemented HERE, in ~80 lines, and the unchanged coordination stack
(build-order planner, validator, reservation table, choreographer,
multi-robot swarm) runs on it. That is the repository's central claim,
demonstrated end to end:

1. **Geometry** — movement/footing/reach rules for your lattice
   (this file's HexLattice2D).
2. **World** — occupancy + blueprint storage satisfying the WorldLike
   protocol (sim/geometry.py). Hex cells fit a dense 2D array under
   "odd-r" offset coordinates, so the stock 2D World is reused UNCHANGED.
3. Pass the geometry as a factory. Nothing else.

Run it:  python examples/hex_lattice.py
"""

import sys
from pathlib import Path

try:  # installed package, or run from the repo root
    import sim  # noqa: F401
except ImportError:  # run as `python examples/hex_lattice.py` pre-install
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.geometry import Geometry, Node
from sim.planner import plan_build_order, validate_plan
from sim.swarm import Swarm
from sim.world import DEFECT, EMPTY, GROUND, VOXEL, World


def hex_neighbors_offsets(r: int):
    """The 6 hex neighbors in "odd-r" offset coordinates (odd rows shift
    right). See www.redblobgames.com/grids/hexagons for the layout."""
    if r % 2 == 0:
        return ((0, -1), (0, 1), (-1, -1), (-1, 0), (1, -1), (1, 0))
    return ((0, -1), (0, 1), (-1, 0), (-1, 1), (1, 0), (1, 1))


class HexLattice2D(Geometry):
    """A hex lattice under simple surface-locomotion rules.

    - **Footing**: an empty in-bounds cell with at least one solid hex
      neighbor (GROUND/VOXEL/DEFECT).
    - **Movement**: one hex step to an adjacent footing cell. (Note how
      much SIMPLER this is than the square lattice: hex adjacency has no
      diagonal-pinch case, so there is no corner-rounding rule at all.)
    - **Reach**: the 6 adjacent cells — symmetric, as the Geometry
      contract requires for stance inversion.
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
        if self.world.occupancy[r, c] != EMPTY:
            return False
        return any(self._solid((r + dr, c + dc)) for dr, dc in hex_neighbors_offsets(r))

    def neighbors(self, node):
        r, c = node
        for dr, dc in hex_neighbors_offsets(r):
            if self.is_footing((r + dr, c + dc)):
                yield (r + dr, c + dc)

    def reach_cells(self, node) -> list[Node]:
        r, c = node
        return [(r + dr, c + dc) for dr, dc in hex_neighbors_offsets(r)]

    def candidate_sort_key(self, target, depot) -> tuple:
        # Supported-first, bottom-up, far-from-depot first (same shape of
        # heuristic as the square lattice; entirely lattice-local).
        r, c = target
        supported = any(
            self._solid((r + dr, c + dc))
            for dr, dc in hex_neighbors_offsets(r)
            if dr == 1
        )
        return (not supported, -r, -abs(c - depot[1]))

    def heuristic_distance(self, a, b) -> int:
        # Exact hex distance via axial coordinates — admissible because
        # every tick-move is exactly one hex step.
        (ar, ac), (br, bc) = a, b
        aq = ac - (ar - (ar & 1)) // 2
        bq = bc - (br - (br & 1)) // 2
        dq, dr = aq - bq, ar - br
        return (abs(dq) + abs(dq + dr) + abs(dr)) // 2

    def future_view(self, solid_nodes) -> "HexLattice2D":
        clone = self.world.copy()
        for r, c in solid_nodes:
            if clone.occupancy[r, c] == EMPTY:
                clone.occupancy[r, c] = VOXEL
        return HexLattice2D(clone)


def main() -> None:
    # A small hex pyramid: base on the row above ground, narrowing upward.
    world = World(rows=7, cols=9)  # stock 2D world, reused unchanged
    for cell in [
        (5, 2), (5, 3), (5, 4), (5, 5), (5, 6),
        (4, 3), (4, 4), (4, 5),
        (3, 4),
    ]:
        world.blueprint[cell] = True
    depot = (5, 0)

    # 1) The unchanged build-order planner proves a safe order exists.
    plan = plan_build_order(world, depot, HexLattice2D)
    assert plan is not None, "no valid hex build order (unexpected)"
    ok, why = validate_plan(world, plan, depot, HexLattice2D)
    assert ok, why
    print(f"planner: {len(plan)} placements, independently validated: {why}")

    # 2) The unchanged multi-robot choreographer executes it.
    swarm = Swarm(
        world, depot,
        starts=[(5, 1), (5, 8)],
        geometry_factory=HexLattice2D,
    )
    while not swarm.done and swarm.tick_count < 2000:
        swarm.tick()
        positions = [r.pos for r in swarm.robots]
        assert len(positions) == len(set(positions)), "collision!"
    assert swarm.done and world.complete, "hex build did not finish"
    print(
        f"swarm: 2 robots built {world.built_count}/{world.blueprint_count} "
        f"hex voxels in {swarm.tick_count} ticks "
        f"(kickbacks={swarm.kickbacks}, collisions=0)"
    )
    print("hex lattice: coordination stack ran UNCHANGED. That's the point.")


if __name__ == "__main__":
    main()
