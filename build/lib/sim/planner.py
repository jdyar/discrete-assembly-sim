# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Build-order solver + plan validation.

Interface contract (NOTES.md): the planner computes the FULL build order
upfront; the robot executes it blindly as a queue and never mutates it. On
failure/defect (Slice 2) the robot halts and requests a replan from the
current world state — ``plan_build_order`` takes the world as-is, so calling
it again mid-build IS the replan.

A plan is a list of :class:`Task` (target cell + the exact approach stance
to place from). ``plan_build_order`` searches placement orders and only
returns sequences that satisfy, at every step, with a depot round trip
between placements:

1. **Reachable at placement time** — a crawlable path (BFS over the grip
   rule) exists depot -> approach stance, and the target is within
   placement reach from that stance.
2. **Never strands itself** — after each placement, a path approach ->
   depot exists; ditto after the final one.
3. **Never walls off unbuilt cells** — implied by 1: the search only
   accepts states from which every remaining cell is still placeable.

``validate_plan`` independently re-simulates a finished plan against the
same rules (defense in depth — run it before execution).

The lattice rules are injected as a :class:`~sim.geometry.Geometry`
(defaulting to the square lattice) — the planner itself never assumes 2D
or square-lattice geometry (binding rule, docs/DESIGN.md); all
lattice knowledge, including candidate-ordering heuristics, lives behind
the geometry interface.
"""

from __future__ import annotations

from typing import Callable, NamedTuple

from .geometry import Geometry, SquareLattice2D, bfs_path, bfs_reachable
from .world import EMPTY, VOXEL, World

Cell = tuple[int, int]

GeometryFactory = Callable[[World], Geometry]


class Task(NamedTuple):
    """One placement: put a voxel at ``cell``, standing at ``approach``."""

    cell: Cell
    approach: Cell


class SearchBudgetExceeded(Exception):
    """Search hit max_nodes: feasibility is UNDECIDED, not disproven."""


def _approaches_for(target: Cell, geometry: Geometry) -> list[Cell]:
    """Footing nodes from which ``target`` is within reach, right now.

    Inverts the reach relation via its symmetry (Geometry contract).
    """
    return [
        a
        for a in geometry.reach_cells(target)
        if geometry.is_footing(a) and target in geometry.reach_cells(a)
    ]


def plan_build_order(
    world: World,
    depot: Cell,
    geometry_factory: GeometryFactory = SquareLattice2D,
    max_nodes: int = 200_000,
) -> list[Task] | None:
    """Search for a complete, valid placement order for the blueprint.

    Depth-first search over sets of built cells with memoized dead ends.
    Candidate ordering comes from ``geometry.candidate_sort_key`` — for the
    square lattice: supported-first (floating placements are legal in the
    no-gravity world but used only when nothing supported works),
    bottom-up, farthest-from-depot first — the order that tends to keep a
    climbable ramp on the depot side.

    Returns None only when the search space was EXHAUSTED — a proof that no
    valid order exists. Raises :class:`SearchBudgetExceeded` if ``max_nodes``
    is hit first (feasibility undecided).

    The world is restored to its input state before returning.
    """
    if world.defect_count:
        # Contract (Slice 2): the robot removes a defect before requesting a
        # replan, so the planner only ever sees clean worlds. A DEFECT cell
        # here would be silently treated as built — fail loudly instead.
        raise ValueError("world contains defective voxels; remove before planning")
    remaining = [
        idx
        for idx in (
            tuple(int(x) for x in nd) for nd in zip(*world.blueprint.nonzero())
        )
        if world.occupancy[idx] == EMPTY
    ]
    if not remaining:
        return []

    geometry = geometry_factory(world)
    dead: set[frozenset[Cell]] = set()
    nodes = 0
    plan: list[Task] = []

    def dfs(remaining: frozenset[Cell]) -> bool:
        nonlocal nodes
        if not remaining:
            return True
        if remaining in dead:
            return False
        nodes += 1
        if nodes > max_nodes:
            raise SearchBudgetExceeded(
                f"no order found within {max_nodes} nodes; feasibility undecided"
            )

        reachable = bfs_reachable(geometry, depot)  # one BFS per world state
        candidates: list[tuple[tuple, Cell, Cell]] = []
        for target in remaining:
            for approach in _approaches_for(target, geometry):
                if approach not in reachable:
                    continue
                key = geometry.candidate_sort_key(target, depot)
                candidates.append((key, target, approach))
        candidates.sort(key=lambda x: x[0])

        for _key, target, approach in candidates:
            world.occupancy[target] = VOXEL
            # Never strand: the stance must still connect back to the depot.
            if bfs_path(geometry, approach, {depot}) is not None:
                plan.append(Task(target, approach))
                if dfs(remaining - {target}):
                    world.occupancy[target] = EMPTY
                    return True
                plan.pop()
            world.occupancy[target] = EMPTY
        dead.add(remaining)
        return False

    snapshot = world.occupancy.copy()
    try:
        found = dfs(frozenset(remaining))
    finally:
        world.occupancy = snapshot
    return list(plan) if found else None


def validate_plan(
    world: World,
    tasks: list[Task],
    depot: Cell,
    geometry_factory: GeometryFactory = SquareLattice2D,
) -> tuple[bool, str]:
    """Re-simulate ``tasks`` against footing/reach/round-trip rules.

    Checks every placement in order on a scratch copy of the world:
    depot -> approach path, footing at the approach, target within reach and
    empty, and approach -> depot path after placing. Returns (ok, reason).
    """
    scratch = world.copy()
    geometry = geometry_factory(scratch)

    for i, (target, approach) in enumerate(tasks):
        label = f"task {i} place {target} from {approach}"
        if not scratch.blueprint[target]:
            return False, f"{label}: target not in blueprint"
        if scratch.occupancy[target] != EMPTY:
            return False, f"{label}: target not empty"
        if not geometry.is_footing(approach):
            return False, f"{label}: approach is not a legal grip cell"
        if target not in geometry.reach_cells(approach):
            return False, f"{label}: target out of reach"
        if bfs_path(geometry, depot, {approach}) is None:
            return False, f"{label}: no path depot->approach"
        scratch.occupancy[target] = VOXEL
        if bfs_path(geometry, approach, {depot}) is None:
            return False, f"{label}: robot stranded after placement"
    if not scratch.complete:
        return False, "plan does not complete the blueprint"
    return True, "ok"
