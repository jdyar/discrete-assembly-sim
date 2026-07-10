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

The footing/reach rules are injected (defaulting to sim.robot's confirmed
rules) so rule variants can be evaluated without touching the robot.
"""

from __future__ import annotations

from typing import Callable, NamedTuple

from .robot import bfs_path, is_grip, reach_cells
from .world import EMPTY, GROUND, VOXEL, World

Cell = tuple[int, int]

ReachFn = Callable[[Cell], list[Cell]]


class Task(NamedTuple):
    """One placement: put a voxel at ``cell``, standing at ``approach``."""

    cell: Cell
    approach: Cell


class SearchBudgetExceeded(Exception):
    """Search hit max_nodes: feasibility is UNDECIDED, not disproven."""


def _approaches_for(target: Cell, reach_fn: ReachFn, world: World) -> list[Cell]:
    """Footing cells from which ``target`` is within reach, right now."""
    r, c = target
    # Scan the full stance neighborhood and invert reach_fn exactly.
    candidates = [
        (r + dr, c + dc)
        for dr in (-1, 0, 1)
        for dc in (-1, 0, 1)
        if (dr, dc) != (0, 0)
    ]
    return [
        a
        for a in candidates
        if is_grip(world, a) and target in reach_fn(a)
    ]


def _supported(world: World, cell: Cell) -> bool:
    r, c = cell
    return r + 1 < world.rows and world.occupancy[r + 1, c] in (GROUND, VOXEL)


def plan_build_order(
    world: World,
    depot: Cell,
    reach_fn: ReachFn = reach_cells,
    max_nodes: int = 200_000,
) -> list[Task] | None:
    """Search for a complete, valid placement order for the blueprint.

    Depth-first search over sets of built cells with memoized dead ends.
    Candidates are tried supported-first (floating placements are legal in
    the no-gravity world but used only when nothing supported works),
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
        (int(r), int(c)) for r, c in zip(*world.blueprint.nonzero())
        if world.occupancy[r, c] == EMPTY
    ]
    if not remaining:
        return []

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

        candidates: list[tuple[tuple, Cell, Cell]] = []
        for target in remaining:
            for approach in _approaches_for(target, reach_fn, world):
                if bfs_path(world, depot, {approach}) is None:
                    continue
                key = (
                    not _supported(world, target),  # supported first
                    -target[0],                     # lower rows first
                    -abs(target[1] - depot[1]),     # far from depot first
                )
                candidates.append((key, target, approach))
        candidates.sort(key=lambda x: x[0])

        for _key, target, approach in candidates:
            world.occupancy[target] = VOXEL
            # Never strand: the stance must still connect back to the depot.
            if bfs_path(world, approach, {depot}) is not None:
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
    reach_fn: ReachFn = reach_cells,
) -> tuple[bool, str]:
    """Re-simulate ``tasks`` against footing/reach/round-trip rules.

    Checks every placement in order on a scratch copy of the world:
    depot -> approach path, footing at the approach, target within reach and
    empty, and approach -> depot path after placing. Returns (ok, reason).
    """
    scratch = World(world.rows, world.cols)
    scratch.occupancy = world.occupancy.copy()
    scratch.blueprint = world.blueprint.copy()

    for i, (target, approach) in enumerate(tasks):
        label = f"task {i} place {target} from {approach}"
        if not scratch.blueprint[target]:
            return False, f"{label}: target not in blueprint"
        if scratch.occupancy[target] != EMPTY:
            return False, f"{label}: target not empty"
        if not is_grip(scratch, approach):
            return False, f"{label}: approach is not a legal grip cell"
        if target not in reach_fn(approach):
            return False, f"{label}: target out of reach"
        if bfs_path(scratch, depot, {approach}) is None:
            return False, f"{label}: no path depot->approach"
        scratch.occupancy[target] = VOXEL
        if bfs_path(scratch, approach, {depot}) is None:
            return False, f"{label}: robot stranded after placement"
    if not scratch.complete:
        return False, "plan does not complete the blueprint"
    return True, "ok"
