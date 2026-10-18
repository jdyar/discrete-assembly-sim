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
4. **Structurally stable, if asked** — when ``stability`` is a
   :class:`~sim.stability.StabilityPolicy`, every prefix must pass
   :func:`~sim.stability.is_stable`. Off unless the caller opts in, so
   a search that used to allow a floating placement still does. Before
   the search, :func:`stability_precheck` applies
   :func:`~sim.stability.necessary_stability_precheck` to the finished
   set. A voxel whose overhang there already exceeds the cantilever
   limit (or that floats there) is past the limit in every prefix, so
   the planner returns ``None`` without expanding a node.

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
from .stability import (
    SagCap,
    StabilityPolicy,
    StabilityPrecheck,
    assess_stability,
    is_stable,
    necessary_stability_precheck,
    occupied_voxels,
    robot_stance_ok,
    sag_within,
)
from .world import EMPTY, VOXEL, World

Cell = tuple[int, int]

GeometryFactory = Callable[[World], Geometry]


class Task(NamedTuple):
    """One placement: put a voxel at ``cell``, standing at ``approach``."""

    cell: Cell
    approach: Cell


class SearchBudgetExceeded(Exception):
    """Search hit max_nodes: feasibility is UNDECIDED, not disproven."""


def _finished_voxels(world: World) -> tuple[Cell, ...]:
    """Voxels the finished blueprint will hold.

    Cells already placed, plus every blueprint cell that is still empty
    and will be placed. Ground is not a voxel. A defective block counts:
    it is still material until something removes it. This is the set
    :func:`plan_build_order` checks at the end of a complete plan.
    """
    cells = set(occupied_voxels(world))
    for nd in zip(*world.blueprint.nonzero()):
        cell = tuple(int(x) for x in nd)
        if world.occupancy[cell] == EMPTY:
            cells.add(cell)
    return tuple(cells)


def stability_precheck(
    world: World, stability: StabilityPolicy | None
) -> StabilityPrecheck:
    """Necessary condition for a stable order of ``world``'s blueprint.

    ``stability is None`` does not reject: the gate is off, and a
    floating placement stays legal. Otherwise the finished voxel set is
    handed to :func:`~sim.stability.necessary_stability_precheck`.
    Coordinate geometry stays in that module; this function only decides
    which cells will exist when the blueprint is complete.

    An empty remaining blueprint still has a finished set (whatever is
    already placed). This function reports that set. :func:`plan_build_order`
    does not consult it when there is nothing left to place: the replan
    contract for an empty remainder is an empty plan.
    """
    if stability is None:
        return StabilityPrecheck(
            rejected=False,
            reason="stability gate is off",
            cell=None,
            lower_bound=None,
            max_cantilever=None,
        )
    return necessary_stability_precheck(_finished_voxels(world), stability)


def _robot_stance_ok(geometry: Geometry, approach, occupied, stability, *, carrying: bool) -> bool:
    """Foot-load check. No feet means the crane stance: always ok."""
    if stability is None:
        return True
    feet = geometry.foot_cells(approach)
    if not feet:
        return True
    ok, _reason = robot_stance_ok(
        occupied,
        feet,
        stability,
        carrying=carrying,
        **geometry.sag_parameters(),
    )
    return ok


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
    *,
    stability: StabilityPolicy | None = None,
    precheck: bool = True,
    sag: SagCap | None = None,
) -> list[Task] | None:
    """Search for a complete, valid placement order for the blueprint.

    Depth-first search over sets of built cells with memoized dead ends.
    Candidate ordering comes from ``geometry.candidate_sort_key`` — for the
    square lattice: supported-first (floating placements are legal in the
    no-gravity world but used only when nothing supported works),
    bottom-up, farthest-from-depot first — the order that tends to keep a
    climbable ramp on the depot side.

    Returns None only on a proof that no valid order exists: the search
    space was exhausted, or (when ``stability`` is set and ``precheck``
    is left on) :func:`stability_precheck` shows the finished structure
    itself fails the cantilever screen. That second proof does not expand
    a search node. Raises :class:`SearchBudgetExceeded` if ``max_nodes``
    is hit first (feasibility undecided). A precheck rejection is not a
    budget hit.

    The world is restored to its input state before returning. A rejection
    from the precheck does not mutate it.

    ``stability``, when given, rejects any prefix that fails
    :func:`~sim.stability.is_stable` and the search tries a different
    cell (the same backtracking it uses for a stranded robot). The
    check does not depend on which stance places the cell, so a cell
    that fails it is not retried from another approach in that state.
    Leave it ``None`` (the default) to keep the no-gravity search.

    ``sag``, when given, is a displacement cap on every prefix (and on
    the finished set, as a necessary condition). Units are those of
    :func:`~sim.stability.estimate_sag`: with the default ``SagCap``
    that is ``mass·g/k`` at ``k = 1``, which is not a fit. A singular
    prefix fails the cap. ``sag`` requires ``stability`` (the frame).
    Leave it ``None`` to skip the solve, including inside the
    buildability gate's re-search unless that caller passes a cap.

    ``precheck=False`` skips only the finished-structure screen. The
    per-prefix gate inside the search stays. The unscreened search cannot
    accept a blueprint the screen rejects, because the finished prefix
    would fail the same gate; turning the screen off only spends the
    budget. Tests use that to check the screen against an exhaustive
    search.
    """
    if sag is not None and stability is None:
        raise ValueError("a sag cap needs a StabilityPolicy for the frame")
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
    if stability is not None and precheck:
        screen = stability_precheck(world, stability)
        if screen.rejected:
            return None
        if sag is not None and not sag_within(_finished_voxels(world), stability, sag):
            return None

    geometry = geometry_factory(world)
    dead: set[frozenset[Cell]] = set()
    nodes = 0
    plan: list[Task] = []
    sag_cache: dict[frozenset[Cell], bool] = {}

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

        # Stability does not depend on the stance, so one refusal retires
        # the cell for this world state.
        rejected_unstable: set[Cell] = set()
        for _key, target, approach in candidates:
            if target in rejected_unstable:
                continue
            # The robot is still holding the block: check the structure
            # it is standing on, with the carried mass on its feet.
            if not _robot_stance_ok(
                geometry, approach, occupied_voxels(world), stability, carrying=True
            ):
                continue
            world.occupancy[target] = VOXEL
            if stability is not None and not is_stable(
                occupied_voxels(world), stability
            ):
                rejected_unstable.add(target)
                world.occupancy[target] = EMPTY
                continue
            if sag is not None:
                placed = frozenset(occupied_voxels(world))
                ok = sag_cache.get(placed)
                if ok is None:
                    ok = sag_within(placed, stability, sag)
                    sag_cache[placed] = ok
                if not ok:
                    rejected_unstable.add(target)
                    world.occupancy[target] = EMPTY
                    continue
            # After the place the block's weight is self-weight, not a
            # payload. A stance that only fails this is not a reason to
            # retire the cell: another foot placement may be lighter.
            if not _robot_stance_ok(
                geometry, approach, occupied_voxels(world), stability, carrying=False
            ):
                world.occupancy[target] = EMPTY
                continue
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
    *,
    stability: StabilityPolicy | None = None,
    sag: SagCap | None = None,
) -> tuple[bool, str]:
    """Re-simulate ``tasks`` against footing/reach/round-trip rules.

    Checks every placement in order on a scratch copy of the world:
    depot -> approach path, footing at the approach, target within reach and
    empty, and approach -> depot path after placing. When ``stability`` is
    set, each prefix must also pass :func:`~sim.stability.is_stable`.
    When ``sag`` is set, each prefix must also pass
    :func:`~sim.stability.sag_within`. Returns (ok, reason).
    """
    if sag is not None and stability is None:
        raise ValueError("a sag cap needs a StabilityPolicy for the frame")
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
        if stability is not None:
            ok_stance, why_stance = robot_stance_ok(
                occupied_voxels(scratch),
                geometry.foot_cells(approach),
                stability,
                carrying=True,
                **geometry.sag_parameters(),
            )
            if not ok_stance:
                return False, f"{label}: {why_stance}"
        scratch.occupancy[target] = VOXEL
        if stability is not None:
            report = assess_stability(occupied_voxels(scratch), stability)
            if not report.stable:
                return (
                    False,
                    f"{label}: partial structure is not stable ({report.reason})",
                )
            if sag is not None and not sag_within(
                occupied_voxels(scratch), stability, sag
            ):
                return False, f"{label}: partial structure exceeds the sag cap"
            ok_stance, why_stance = robot_stance_ok(
                occupied_voxels(scratch),
                geometry.foot_cells(approach),
                stability,
                carrying=False,
                **geometry.sag_parameters(),
            )
            if not ok_stance:
                return False, f"{label}: {why_stance}"
        if bfs_path(geometry, approach, {depot}) is None:
            return False, f"{label}: robot stranded after placement"
    if not scratch.complete:
        return False, "plan does not complete the blueprint"
    return True, "ok"
