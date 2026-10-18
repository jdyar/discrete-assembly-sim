# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Cooperative A* choreographer + connectivity gate (Slice 3a).

Clean-room from the ARMADAS system paper's planning description (Gregg et
al., Science Robotics 2024, NTRS 20230005194; docs/DESIGN.md,
Provenance): robot itineraries are found by **cooperative A*
over the time-expanded graph** — robots plan one at a time against the
shared :class:`~sim.reservations.ReservationTable`, so a later plan can
never conflict with an earlier one. Deadlock freedom in ARMADAS comes
from reservations + guaranteed free paths; our equivalent guarantee is
the **connectivity gate**: no placement is committed if the resulting
permanent future graph (world solids + all deeds) would disconnect any
robot's anchor position from the depot. A refused placement is kicked
back to the sequencer (receding-horizon coupling, docs/DESIGN.md
Architecture Requirements) rather than failing the run.

An optional third gate, :func:`stability_gate`, applies when the caller
passes a :class:`~sim.stability.StabilityPolicy`. It is off by default.
It asks whether the partial structure (voxels already placed, plus this
proposal) still passes :func:`~sim.stability.is_stable`. A deed is not
material — gravity does not honor a reservation — so unplaced deeds are
not supports. A refusal is the same kickback as the other two gates.

Policy (decided 2026-07-14, options presented): **per-task receding
horizon** — a robot plans ONE task when it becomes free (current pos ->
depot, PICK, depot -> approach stance, PLACE, INSPECT), reserving only
that task's timeline. Replanning after a defect (Slice 3c) is therefore
the same code path as normal operation: release the robot's future
leases and plan the next task.

Lattice-agnostic: movement comes from Geometry.neighbors via the
time-expanded graph; future projection from Geometry.future_view; no 2D
assumptions (binding rule).
"""

from __future__ import annotations

import heapq
from itertools import count
from typing import Hashable, Iterable, NamedTuple

from .geometry import Geometry, bfs_path
from .planner import SearchBudgetExceeded, plan_build_order
from .reservations import OwnerId, ReservationConflict, ReservationTable
from .stability import (
    JointCap,
    SagCap,
    StabilityPolicy,
    is_stable,
    joints_within,
    occupied_voxels,
    robot_stance_ok,
    sag_within,
)
from .texgraph import TENode, TimeExpandedGraph
from .world import EMPTY

Node = Hashable

# Itinerary actions.
MOVE = "MOVE"
WAIT = "WAIT"
PICK = "PICK"
PLACE = "PLACE"
INSPECT = "INSPECT"
REMOVE = "REMOVE"

MAX_HORIZON = 256  # ticks a single task plan may look ahead
PARK_TICKS = 16  # trailing hold at a plan's end node (renewed by the swarm)

# buildability_gate memo: (occupancy bytes, projected solids, remaining)
# -> verdict. Bounded; cleared wholesale at capacity (keys die naturally
# as the world changes, so eviction sophistication buys nothing).
_BUILDABILITY_CACHE: dict = {}
_BUILDABILITY_CACHE_MAX = 4096


class Step(NamedTuple):
    """One scheduled action: be at ``node`` during tick ``t`` doing ``action``.

    ``target`` is the acted-on cell for PLACE/INSPECT/REMOVE, else None.
    """

    t: int
    node: Node
    action: str
    target: Node | None = None


class TaskPlan(NamedTuple):
    steps: list  # list[Step], consecutive ticks
    target: Node
    approach: Node
    t_place: int  # deed commit tick
    t_end: int  # last reserved tick (incl. the spare REMOVE slot)


def cooperative_astar(
    graph: TimeExpandedGraph,
    start: Node,
    t0: int,
    goals: set,
    extra_free_ticks: int = 0,
    max_horizon: int = MAX_HORIZON,
) -> list[TENode] | None:
    """A* over the time-expanded graph from ``(start, t0)`` into ``goals``.

    Cost = ticks; heuristic = the geometry's admissible distance bound
    (min over goals). A goal state only counts if its node is ALSO free
    for this owner for ``extra_free_ticks`` ticks after arrival — the
    action ticks (PICK / PLACE+INSPECT+spare) that must happen there.
    Returns the (node, t) path including the start, or None within the
    horizon.
    """
    if not goals:
        return None
    # A goal whose reservation keys are already a static obstacle can
    # never be entered on this plan. Dropping those goals first keeps
    # the same answer and skips a horizon-long search.
    goals = {
        goal
        for goal in goals
        if all(key not in graph._blocked for key in graph._reservation_keys(goal))
    }
    if not goals:
        return None
    geom = graph.geometry
    res = graph.reservations
    owner = graph.owner

    def h(node: Node) -> int:
        # Same value at every tick of a node. One search asks for it
        # once per time-expanded state, and a removal goal set is large.
        hit = h_cache.get(node)
        if hit is None:
            hit = min(geom.heuristic_distance(node, g) for g in goals)
            h_cache[node] = hit
        return hit

    h_cache: dict = {}

    def at_goal(state: TENode) -> bool:
        if state.node not in goals:
            return False
        keys = geom.reservation_keys(state.node)
        return all(
            res.is_free(key, state.t + k, owner)
            for key in keys
            for k in range(1, extra_free_ticks + 1)
        )

    start_state = TENode(start, t0)
    tie = count()
    open_heap: list[tuple[int, int, int, TENode]] = [
        (h(start), 0, next(tie), start_state)
    ]
    came: dict[TENode, TENode] = {start_state: start_state}
    best_g: dict[TENode, int] = {start_state: 0}

    while open_heap:
        f, g, _, state = heapq.heappop(open_heap)
        if g > best_g.get(state, g):
            continue
        if at_goal(state):
            path = [state]
            while came[path[-1]] != path[-1]:
                path.append(came[path[-1]])
            path.reverse()
            return path
        if state.t - t0 >= max_horizon:
            continue
        for nxt in graph.successors(state):
            ng = nxt.t - t0
            if ng < best_g.get(nxt, ng + 1):
                best_g[nxt] = ng
                came[nxt] = state
                heapq.heappush(open_heap, (ng + h(nxt.node), ng, next(tie), nxt))
    return None


def connectivity_gate(
    geometry: Geometry,
    reservations: ReservationTable,
    proposed_target: Node,
    anchors: Iterable[Node],
    depot: Node,
) -> bool:
    """May ``proposed_target`` be committed without sealing anyone in?

    Builds the permanent future graph — the lattice as if every existing
    deed AND the proposed placement were already solid — and requires a
    path anchor -> depot for every robot anchor. Refusal means kickback,
    not failure (the blocking robot moves; the task is retried).
    """
    solids = reservations.permanent_nodes_at(10**9) | {proposed_target}
    future = geometry.future_view(solids)
    for anchor in anchors:
        if anchor == depot:
            continue
        if bfs_path(future, anchor, {depot}) is None:
            return False
    return True


def buildability_gate(
    geometry: Geometry,
    reservations: ReservationTable,
    proposed_target: Node,
    remaining_cells: Iterable[Node],
    depot: Node,
) -> bool:
    """May ``proposed_target`` be committed without stranding the BUILD?

    Robot-connectivity is not enough under claim-ahead concurrency:
    out-of-order placements can seal a pocket of UNBUILT blueprint cells
    (found by the repair-in-a-crowd trap — two pending cells walled in on
    all sides). This gate re-proves order feasibility: on the permanent
    future view (all deeds + the proposal), the reachability-aware
    sequencer (plan_build_order — connectivity proven at every step of
    its search) must still find a complete order for what remains.

    Cheap proximity filter: sealing needs adjacency, so the full probe
    runs only when the proposal is near a remaining cell. A search-budget
    blowup refuses conservatively (kickback retries later, when the
    world is simpler).
    """
    remaining = [c for c in remaining_cells if c != proposed_target]
    if not remaining:
        return True
    if all(
        geometry.heuristic_distance(proposed_target, c) > 3 for c in remaining
    ):
        return True
    solids = frozenset(reservations.permanent_nodes_at(10**9)) | {proposed_target}
    # The verdict depends only on (world occupancy, projected solids,
    # remaining cells) — memoized because kickback retries and sibling
    # robots re-ask the identical question many times per world state
    # (profiled: this probe was ~94% of a 3D build's wall clock).
    key = (geometry.world.occupancy.tobytes(), solids, frozenset(remaining))
    if key in _BUILDABILITY_CACHE:
        return _BUILDABILITY_CACHE[key]
    future = geometry.future_view(solids)
    try:
        order = plan_build_order(
            future.world, depot, geometry.bound_factory(), max_nodes=20_000
        )
        verdict = order is not None
    except (SearchBudgetExceeded, ValueError):
        # Budget blown, or a defect is transiently in the world (between
        # PLACE and REMOVE): refuse now, retry when things settle.
        # Budget verdicts are NOT cached — a retry may succeed later.
        return False
    if len(_BUILDABILITY_CACHE) >= _BUILDABILITY_CACHE_MAX:
        _BUILDABILITY_CACHE.clear()
    _BUILDABILITY_CACHE[key] = verdict
    return verdict


def stability_gate(
    geometry: Geometry,
    proposed_target: Node,
    policy: StabilityPolicy,
    approach: Node | None = None,
    sag: SagCap | None = None,
    joints: JointCap | None = None,
) -> bool:
    """May ``proposed_target`` be placed without an unstable partial build?

    The structure that has to stand is the voxels already in the world
    plus this cell. Other robots' unplaced deeds are ignored: a deed is
    a promise to place, and the block is not there yet. (The connectivity
    gate counts those deeds, because a cell that *will* be solid can
    trap a robot. Load and entrapment are different questions.)

    The frame — which axis is up, where the base is, how far a span may
    reach — comes entirely from ``policy``. This function does no
    coordinate arithmetic of its own.

    When ``approach`` has foot cells (on-lattice stance) the same policy
    also rejects the step if a foot would sit past ``max_cantilever``, or
    if ``max_sag`` is set and the sag **including that robot's loads**
    is over the cap. A crane stance has no foot cells, so this extra
    check does not run and the gate stays the combinatorial screen.
    The carried voxel is included on the structure the robot is
    standing on (before the new block is material) and omitted after
    the block is in the build, so its weight is not counted twice.

    ``sag``, when given, also refuses a prefix whose worst downward
    displacement exceeds the cap. That number is
    :func:`~sim.stability.estimate_sag`'s, in ``mass·g/k`` at the cap's
    stiffness (the default ``k = 1`` is not a fit). ``None`` accepts
    every prefix.

    ``joints``, when given, refuses a prefix whose face-joint demand
    exceeds ``capacity × safety_factor``. The demand is in the cap's
    force unit (``mass·g`` at ``k = mass = g = 1``). Foot loads are
    included when the stance has feet. ``None`` skips the check.
    """
    cells = set(occupied_voxels(geometry.world))
    target = tuple(int(coord) for coord in proposed_target)
    feet = geometry.foot_cells(approach) if approach is not None else ()
    params = geometry.sag_parameters()
    if feet and not robot_stance_ok(
        cells, feet, policy, carrying=True, joints=joints, **params
    )[0]:
        return False
    cells.add(target)
    if not is_stable(cells, policy):
        return False
    if not sag_within(cells, policy, sag):
        return False
    if not joints_within(cells, policy, joints):
        return False
    if feet and not robot_stance_ok(
        cells, feet, policy, carrying=False, joints=joints, **params
    )[0]:
        return False
    return True


def pin_pose(
    table: ReservationTable,
    geometry: Geometry,
    owner: OwnerId,
    node,
    t0: int,
    t1: int,
    *,
    best_effort: bool = False,
) -> int:
    """Lease every reservation key of ``node`` on ``[t0, t1]``.

    One-cell stances have a single key, so this is :meth:`reserve_hold`.
    A two-foot stance also holds each gripped face. Best-effort returns
    the last tick held by **every** key (the shortest of the per-key
    holds); a foot that could not be pinned pulls that horizon in.
    """
    keys = geometry.reservation_keys(node)
    if not best_effort:
        for key in keys:
            table.reserve_hold(owner, key, t0, t1)
        return t1
    last = t1
    for key in keys:
        held = table.reserve_hold_best_effort(owner, key, t0, t1)
        if held < last:
            last = held
    return last


def _reserve_leg(
    owner: OwnerId,
    leg: list[TENode],
    table: ReservationTable,
    geometry: Geometry,
) -> None:
    """Lease a leg: node-per-tick + edges (reserve_path), plus every
    swept intermediate of multi-cell strides for the transition window
    (both ticks — matching the time-expanded graph's gating exactly),
    plus any extra reservation keys of the pose (gripped faces)."""
    table.reserve_path(owner, [s.node for s in leg], leg[0].t)
    for state in leg:
        for key in geometry.reservation_keys(state.node):
            if key == state.node:
                continue
            table.reserve_hold(owner, key, state.t, state.t)
    for prev, cur in zip(leg, leg[1:]):
        if cur.node != prev.node:
            for c in geometry.move_footprint(prev.node, cur.node):
                table.reserve_hold(owner, c, prev.t, cur.t)


def _path_to_steps(path: list[TENode]) -> list[Step]:
    """Convert an A* (node, t) path (excluding its start state) to steps."""
    steps: list[Step] = []
    for prev, cur in zip(path, path[1:]):
        action = WAIT if cur.node == prev.node else MOVE
        steps.append(Step(cur.t, cur.node, action))
    return steps


def plan_task(
    owner: OwnerId,
    pos: Node,
    t_now: int,
    target: Node,
    stances: list,
    geometry: Geometry,
    table: ReservationTable,
    depot: Node,
    anchors: Iterable[Node],
    inspect_enabled: bool = True,
    static_obstacles: frozenset | set = frozenset(),
    remaining_cells: Iterable[Node] = (),
    stability: StabilityPolicy | None = None,
    sag: SagCap | None = None,
    check_buildability: bool = True,
    joints: JointCap | None = None,
) -> TaskPlan | None:
    """Plan and RESERVE one full task: pos -> depot, PICK, -> stance,
    PLACE (deed), INSPECT + spare REMOVE slot.

    All legs are solved before anything is written; the write is atomic
    per leg and rolled back wholesale (release_owner from t_now+1) if any
    reservation unexpectedly conflicts. Returns None — with the table
    unchanged from this attempt — when a leg is unplannable, the stance
    set is empty, or a gate refuses the placement.     ``stability``, when
    given, is the structural gate; leave it ``None`` to skip that gate.
    ``check_buildability=False`` skips the order re-search. A scripted
    scaffold run already proved an order and only needs connectivity
    plus the structural caps on the cell being committed.

    ``anchors`` are the other robots' committed end positions; this
    robot's own escape is checked from the chosen stance.
    """
    graph = TimeExpandedGraph(geometry, table, owner, static_obstacles)

    # Leg 1: to the depot; the pick tick needs the depot one extra tick.
    leg1 = cooperative_astar(graph, pos, t_now, {depot}, extra_free_ticks=1)
    if leg1 is None:
        return None
    t_arrive = leg1[-1].t
    t_pick = t_arrive + 1

    # Leg 2: depot -> a stance. After arrival the stance must stay free
    # for PLACE + INSPECT + spare REMOVE (3 ticks with inspection, 1
    # without) + one handoff tick before the rolling hold takes over.
    action_ticks = (3 if inspect_enabled else 1) + 1
    leg2 = cooperative_astar(
        graph, depot, t_pick, set(stances), extra_free_ticks=action_ticks
    )
    if leg2 is None:
        return None
    approach = leg2[-1].node
    t_at_stance = leg2[-1].t
    t_place = t_at_stance + 1
    t_end = t_place + (2 if inspect_enabled else 0)

    # Gates: the placement must sever no robot's permanent-future escape
    # (including this robot's own from its stance), and must leave the
    # remaining blueprint completable. The structural gate, when on,
    # runs before the build-order search: a span that will not stand is
    # refused without paying for that search. A refusal returns None
    # with nothing reserved, which the swarm kicks back for reordering.
    if not connectivity_gate(
        geometry, table, target, list(anchors) + [approach], depot
    ):
        return None
    if stability is not None and not stability_gate(
        geometry, target, stability, approach=approach, sag=sag, joints=joints
    ):
        return None
    if check_buildability and not buildability_gate(
        geometry, table, target, remaining_cells, depot
    ):
        return None

    # Commit: leases for both legs, action holds, the deed, and a trailing
    # park-hold at the stance — a robot occupies its end node until its
    # next plan, so that occupancy must be visible to everyone else's A*
    # (the next plan's release_owner clears it).
    try:
        _reserve_leg(owner, leg1, table, geometry)
        pin_pose(table, geometry, owner, depot, t_pick, t_pick)
        _reserve_leg(owner, leg2, table, geometry)
        pin_pose(table, geometry, owner, approach, t_place, t_end + 1)  # + handoff
        table.reserve_deed(owner, target, t_place)
    except ReservationConflict:
        table.release_owner(owner, t_now + 1)
        return None
    pin_pose(
        table, geometry, owner, approach, t_end + 2, t_end + PARK_TICKS,
        best_effort=True,
    )

    steps = _path_to_steps(leg1)
    steps.append(Step(t_pick, depot, PICK))
    steps.extend(_path_to_steps(leg2))
    steps.append(Step(t_place, approach, PLACE, target))
    if inspect_enabled:
        steps.append(Step(t_place + 1, approach, INSPECT, target))
    return TaskPlan(steps, target, approach, t_place, t_end)


def removal_keeps_paths(
    geometry: Geometry,
    target: Node,
    anchors: Iterable[Node],
    depot: Node,
) -> bool:
    """After ``target`` is emptied, every anchor still reaches the depot.

    The live world still holds the voxel. The check runs on a copy with
    that cell empty, so a robot whose only footing was the scaffold is
    refused. An anchor sitting in the cell being removed is refused too.
    """
    clone = geometry.world.copy()
    clone.occupancy[target] = EMPTY
    future = geometry.bound_factory()(clone)
    for anchor in anchors:
        if anchor == target:
            return False
        if not future.is_footing(anchor) and anchor != depot:
            return False
        if bfs_path(future, anchor, {depot}) is None:
            return False
    return True


def plan_remove_task(
    owner: OwnerId,
    pos: Node,
    t_now: int,
    target: Node,
    stances: list,
    geometry: Geometry,
    table: ReservationTable,
    depot: Node,
    anchors: Iterable[Node],
    static_obstacles: frozenset | set = frozenset(),
    stability: StabilityPolicy | None = None,
    sag: SagCap | None = None,
    joints: JointCap | None = None,
) -> TaskPlan | None:
    """Plan and reserve a scaffold removal: walk to a stance, REMOVE.

    No depot trip and no deed — the block is already there and is
    leaving. The post-removal structure must pass ``stability`` and
    ``sag`` when those are set, and every anchor, including the chosen
    stance, must still reach the depot after the cell is empty.
    """
    if not stances:
        return None
    cell = tuple(int(coord) for coord in target)
    remaining = set(occupied_voxels(geometry.world))
    remaining.discard(cell)
    if stability is not None and not is_stable(remaining, stability):
        return None
    if sag is not None and not sag_within(remaining, stability, sag):
        return None
    if joints is not None and not joints_within(remaining, stability, joints):
        return None

    graph = TimeExpandedGraph(geometry, table, owner, static_obstacles)
    anchor_list = list(anchors)
    open_stances = set(stances)
    chosen_leg = None
    approach = None
    while open_stances:
        leg = cooperative_astar(
            graph, pos, t_now, open_stances, extra_free_ticks=2
        )
        if leg is None:
            return None
        stance = leg[-1].node
        if removal_keeps_paths(geometry, target, anchor_list + [stance], depot):
            chosen_leg = leg
            approach = stance
            break
        open_stances.discard(stance)
    if chosen_leg is None or approach is None:
        return None

    t_remove = chosen_leg[-1].t + 1
    try:
        _reserve_leg(owner, chosen_leg, table, geometry)
        pin_pose(table, geometry, owner, approach, t_remove, t_remove + 1)
    except ReservationConflict:
        table.release_owner(owner, t_now + 1)
        return None
    pin_pose(
        table,
        geometry,
        owner,
        approach,
        t_remove + 2,
        t_remove + PARK_TICKS,
        best_effort=True,
    )
    steps = _path_to_steps(chosen_leg)
    steps.append(Step(t_remove, approach, REMOVE, target))
    return TaskPlan(steps, target, approach, t_remove, t_remove)


def plan_walk(
    owner: OwnerId,
    pos: Node,
    t_now: int,
    goal: Node,
    geometry: Geometry,
    table: ReservationTable,
    static_obstacles: frozenset | set = frozenset(),
    min_clear: int = 4,
) -> list | None:
    """Plan and reserve a plain relocation (park/displace walk; no deed,
    no gate — walking commits nothing permanent). Returns steps or None.

    ``min_clear``: the destination must be free for this many ticks after
    arrival. Parking where an inbound lease lands moments later is a
    death trap (no time to escape); urgent 1-tick hops may pass
    ``min_clear=1`` explicitly.
    """
    graph = TimeExpandedGraph(geometry, table, owner, static_obstacles)
    leg = cooperative_astar(graph, pos, t_now, {goal}, extra_free_ticks=min_clear)
    if leg is None or len(leg) == 1:
        return None
    t_arrive = leg[-1].t
    try:
        _reserve_leg(owner, leg, table, geometry)
        pin_pose(table, geometry, owner, goal, t_arrive + 1, t_arrive + 1)
    except ReservationConflict:
        table.release_owner(owner, t_now + 1)
        return None
    pin_pose(
        table, geometry, owner, goal, t_arrive + 2, t_arrive + PARK_TICKS,
        best_effort=True,
    )
    return _path_to_steps(leg)
