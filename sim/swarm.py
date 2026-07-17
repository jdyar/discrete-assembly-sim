# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Multi-robot executor: sequencer <-> choreographer coupled loop (Slice 3).

One Swarm owns the world, the reservation table, N robots, and the
sequencer. Each tick:

1. **Hold phase** — every robot without an itinerary pins its position in
   the reservation table for the planning horizon (nobody plans through a
   parked robot).
2. **Planning phase** (receding horizon) — each idle robot claims the
   nearest claimable task from the sequencer and asks the choreographer
   for a full task plan (pos -> depot -> PICK -> stance -> PLACE ->
   INSPECT). Gate refusals and unplannable legs are KICKED BACK: the task
   returns to the sequencer, the robot retries next tick (coupled loop,
   docs/DESIGN.md). Robots with no
   remaining work rest-walk home so they never block a cavity or corridor.
3. **Execution phase** — each robot performs its scheduled step for this
   tick. MOVE legality is re-verified against the current world first
   (another robot's placement can pinch a corner that was open at plan
   time); an invalidated itinerary is aborted and its task kicked back.

Slice 3c error correction rides the same loop: INSPECT failing schedules
REMOVE (the spare reserved tick), which removes the voxel, clears the
deed, resequences, and leaves the robot idle — its next plan is the
repair, threaded through everyone else's live reservations.

Watchdog: if every robot idles with work pending for too long, the
sequencer re-derives the build order from the current world
(plan_build_order IS the reachability-aware sequencer); if that cannot
help, SwarmStuck is raised rather than spinning forever.
"""

from __future__ import annotations

import random
from typing import Hashable

from .choreographer import (
    INSPECT,
    MOVE,
    PICK,
    PLACE,
    REMOVE,
    WAIT,
    MAX_HORIZON,
    Step,
    plan_task,
    plan_walk,
)
from .geometry import Geometry, SquareLattice2D
from .planner import _approaches_for, plan_build_order
from .reservations import ReservationConflict, ReservationTable
from .world import World

Node = Hashable

# Idle robots pin their cell with a SHORT rolling hold, renewed every tick.
# Long holds turn parked robots into permanent walls for everyone else's
# A*; short holds mean another robot may plan through the cell after the
# notice window — the renewal then conflicts, flags must_move, and the
# parked robot escapes in time (displacement, not deadlock).
HOLD_HORIZON = 16
CLAIM_COOLDOWN = 8  # ticks a robot avoids a cell it just failed to plan
RESEQUENCE_AFTER = 60  # all-idle-with-work ticks before resequencing
STUCK_AFTER = 400  # ... before giving up


class SwarmStuck(RuntimeError):
    """No robot can make progress and resequencing did not help."""


class Sequencer:
    """Ordered task supply, re-derived from the world on demand.

    The order comes from ``plan_build_order`` — the reachability-aware
    sequencer (connectivity proven at every step of its search). Claiming
    is nearest-first within a lookahead window of the order (frontier
    claiming, Slice 3b), which preserves the order's dependency structure
    approximately; the choreographer's gate + kickback covers the rest.
    """

    def __init__(
        self,
        world: World,
        depot: Node,
        claim_ahead: int = 6,
        geometry_factory=SquareLattice2D,
    ) -> None:
        self.world = world
        self.depot = depot
        self.claim_ahead = claim_ahead
        self.geometry_factory = geometry_factory
        self.queue: list[Node] = []
        self.claimed: set[Node] = set()
        self.resequence()

    def resequence(self) -> None:
        """Re-derive the build order for everything not yet built."""
        tasks = plan_build_order(self.world, self.depot, self.geometry_factory)
        if tasks is None:
            raise SwarmStuck("sequencer: no feasible build order exists")
        self.queue = [t.cell for t in tasks if t.cell not in self.claimed]

    def claim(
        self, near: Node, geometry: Geometry, skip: set | None = None
    ) -> Node | None:
        """Nearest claimable task within the lookahead window, or None.

        ``skip`` holds cells this robot recently failed to plan (claim
        cooldown) — without it, the nearest robot re-claims the same
        unplannable cell every tick and starves robots that could
        actually do it (livelock found by the repair-in-a-crowd trap).
        """
        window = [c for c in self.queue[: self.claim_ahead] if c not in (skip or ())]
        if not window:
            return None
        cell = min(window, key=lambda c: geometry.heuristic_distance(near, c))
        self.queue.remove(cell)
        self.claimed.add(cell)
        return cell

    def kickback(self, cell: Node) -> None:
        """Return a claimed task to the FRONT of the order (retry soon)."""
        self.claimed.discard(cell)
        self.queue.insert(0, cell)

    def complete(self, cell: Node) -> None:
        self.claimed.discard(cell)

    def repair(self, cell: Node) -> None:
        """A placed cell went bad and was removed: resequence from the
        world as it stands (the removed cell is EMPTY again and pending)."""
        self.claimed.discard(cell)
        self.resequence()

    @property
    def exhausted(self) -> bool:
        return not self.queue and not self.claimed


class SwarmRobot:
    """Execution state for one robot. Movement/coordination all live in
    the choreographer; this is a cursor over a scheduled itinerary."""

    def __init__(self, rid: int, start: Node) -> None:
        self.id = rid
        self.start = start
        self.pos = start
        self.carrying = False
        self.state = "IDLE"
        self.itinerary: list[Step] = []
        self.task: Node | None = None
        self.must_move = False  # spot lost to another's lease: escape now
        self.claim_cooldown: dict = {}  # cell -> tick until which not to re-claim
        self.storage: Node = start  # assigned park node (ARMADAS storage location)
        self.backoff_until = 0  # after a kickback: retreat before re-claiming

    @property
    def anchor(self) -> Node:
        """Where this robot will be when its current plan ends."""
        return self.itinerary[-1].node if self.itinerary else self.pos


class Swarm:
    def __init__(
        self,
        world: World,
        depot: Node,
        starts: list,
        *,
        defect_p: float = 0.0,
        defect_cells: set | None = None,
        rng: random.Random | None = None,
        inspect_enabled: bool = True,
        claim_ahead: int = 6,
        geometry_factory=SquareLattice2D,
    ) -> None:
        if len(set(starts)) != len(starts):
            raise ValueError("robot starts must be distinct")
        self.world = world
        self.depot = depot
        self.geometry: Geometry = geometry_factory(world)
        self.table = ReservationTable()
        self.robots = [SwarmRobot(i, s) for i, s in enumerate(starts)]
        self.sequencer = Sequencer(world, depot, claim_ahead, geometry_factory)
        self.defect_p = defect_p
        self.defect_cells = set(defect_cells or ())
        self._defects_pending = set(self.defect_cells)  # first attempt only
        self.rng = rng or random.Random(0)
        self.inspect_enabled = inspect_enabled
        self.tick_count = 0
        self.events: list[tuple] = []  # (tick, robot_id, kind, node)
        self.kickbacks = 0
        self.defects_found = 0
        self._idle_with_work = 0
        self._resequenced_at: int | None = None
        self._assign_storage()

    def _assign_storage(self) -> None:
        """Assign each robot a distinct STORAGE node, clean-room from the
        ARMADAS deadlock-freedom mechanism (R1 §5.1: assigned storage
        locations guarantee every robot always has a free path). Idle
        robots return to their storage node instead of parking wherever
        they finished — parked robots then never block the depot corridor,
        a cavity, or a stance.

        Choice: reachable footing nodes (BFS from the depot on the initial
        world), excluding the depot and blueprint cells, scored by
        distance-from-depot + distance-from-nearest-blueprint-cell (large
        = off the traffic); top distinct nodes, matched to robots by
        proximity of their starts.
        """
        bp = list(self.sequencer.queue)  # lattice-agnostic blueprint cells
        seen = {self.depot}
        frontier = [self.depot]
        candidates: list[Node] = []
        while frontier and len(seen) < 2000:
            nxt = []
            for node in frontier:
                for nb in self.geometry.neighbors(node):
                    if nb in seen:
                        continue
                    seen.add(nb)
                    nxt.append(nb)
                    if nb != self.depot and not self._is_blueprint(nb):
                        candidates.append(nb)
            frontier = nxt

        def score(n: Node) -> int:
            d_depot = self.geometry.heuristic_distance(n, self.depot)
            d_bp = min(
                (self.geometry.heuristic_distance(n, b) for b in bp), default=0
            )
            return d_depot + d_bp

        candidates.sort(key=score, reverse=True)
        chosen = candidates[: len(self.robots)]
        # Nearest-start matching (greedy) so robots don't cross each other
        # just to park.
        unmatched = list(self.robots)
        for node in chosen:
            r = min(
                unmatched,
                key=lambda r: self.geometry.heuristic_distance(r.start, node),
            )
            r.storage = node
            unmatched.remove(r)
        for r in unmatched:  # fewer candidates than robots: park at start
            r.storage = r.start

    # -- public surface ------------------------------------------------------

    @property
    def done(self) -> bool:
        return self.world.complete and self.sequencer.exhausted

    @property
    def settled(self) -> bool:
        """No work left to try: done, or (no-inspect baseline) everything
        attempted with defects left standing and all robots parked."""
        return self.done or (
            self.sequencer.exhausted
            and not any(r.itinerary for r in self.robots)
        )

    def tick(self) -> None:
        nxt = self.tick_count + 1

        # Phase 0: pin every planless robot before anyone plans. A robot
        # whose spot is already leased by someone else (possible after an
        # abort left it off its reserved trail) MUST move this round —
        # A*'s reservation awareness finds it a timing-valid escape.
        for r in self.robots:
            if not r.itinerary:
                self.table.release_owner(r.id, nxt)
                held_to = self.table.reserve_hold_best_effort(
                    r.id, r.pos, nxt, nxt + HOLD_HORIZON
                )
                # Partial hold = someone's lease is inbound: keep what we
                # could (protection until then) and get out of the way.
                r.must_move = held_to < nxt + HOLD_HORIZON

        # Phase 1: planning for idle robots.
        kicks_before = self.kickbacks
        for r in self.robots:
            if not r.itinerary:
                self._plan_for(r)
                if r.must_move and not r.itinerary:
                    self._escape(r)

        # Phase 1b: displacement wave. A failed plan usually means parked
        # robots wall a corridor (parked = static obstacle to others'
        # A*). Planless robots are then sent to their storage nodes,
        # farthest-from-depot first, so convoys un-file over a few ticks
        # — ARMADAS's storage-location discipline (R1 §5.1). If a stall
        # persists with everyone AT storage, storage itself is in the
        # way (e.g. it doubles as the last stance): sidestep.
        if self.kickbacks > kicks_before:
            idle = [r for r in self.robots if not r.itinerary]
            idle.sort(
                key=lambda r: -self.geometry.heuristic_distance(r.pos, self.depot)
            )
            for r in idle:
                if r.pos != r.storage:
                    self._yield_walk(r)
                elif self._idle_with_work >= 8:
                    self._sidestep_walk(r)

        # Phase 2: execute this tick's steps.
        for r in self.robots:
            if r.itinerary and r.itinerary[0].t == nxt:
                self._execute(r, r.itinerary.pop(0))
            elif not r.itinerary:
                r.state = "IDLE"

        self.tick_count = nxt
        self.table.advance_to(nxt)
        self._check_collisions()
        self._watchdog()

    # -- planning ------------------------------------------------------------

    def _plan_for(self, r: SwarmRobot) -> None:
        t = self.tick_count
        if t < r.backoff_until:
            # Fresh kickback: retreat toward storage instead of thrashing
            # through claim attempts from a contested spot.
            self._yield_walk(r)
            return
        skip = {c for c, until in r.claim_cooldown.items() if until > t}
        r.claim_cooldown = {c: u for c, u in r.claim_cooldown.items() if u > t}
        cell = self.sequencer.claim(r.pos, self.geometry, skip)
        if cell is None:
            # No claimable work right now: return to the assigned storage
            # node so an idle robot never squats in a cavity, corridor,
            # or someone's stance.
            self._yield_walk(r)
            return

        stances = _approaches_for(cell, self.geometry)
        anchors = [o.anchor for o in self.robots if o.id != r.id]
        self.table.release_owner(r.id, t + 1)  # drop the phase-0 hold
        plan = plan_task(
            r.id,
            r.pos,
            t,
            cell,
            stances,
            self.geometry,
            self.table,
            self.depot,
            anchors,
            self.inspect_enabled,
            static_obstacles=self._obstacles_for(r),
            remaining_cells=frozenset(self.sequencer.queue),
        )
        if plan is None:
            self._try_hold(r)
            self.sequencer.kickback(cell)
            r.claim_cooldown[cell] = t + CLAIM_COOLDOWN
            r.backoff_until = t + 3
            self.kickbacks += 1
            self._event(r, "kickback", cell)
            return
        r.itinerary = list(plan.steps)
        r.task = cell
        r.state = "TASKED"
        self._event(r, "claim", cell)

    def _try_hold(self, r: SwarmRobot) -> None:
        """Re-pin an idle robot (best effort); a partial hold marks it
        must-move — protected until the inbound lease, then displaced."""
        t = self.tick_count
        held_to = self.table.reserve_hold_best_effort(
            r.id, r.pos, t + 1, t + 1 + HOLD_HORIZON
        )
        r.must_move = held_to < t + 1 + HOLD_HORIZON

    def _yield_walk(self, r: SwarmRobot) -> None:
        """Send an idle robot to its assigned storage node (ARMADAS
        storage-location discipline). Already there, or no path right
        now: stay held and retry later."""
        if r.itinerary or r.pos == r.storage:
            return
        t = self.tick_count
        self.table.release_owner(r.id, t + 1)
        steps = plan_walk(
            r.id, r.pos, t, r.storage, self.geometry, self.table,
            static_obstacles=self._obstacles_for(r),
        )
        if steps:
            r.itinerary = steps
            r.state = "PARKING"
            return
        self._try_hold(r)

    def _sidestep_walk(self, r: SwarmRobot) -> None:
        """Storage itself is blocking someone (it can double as the last
        stance): temporarily relocate to the farthest reachable neutral
        node — not the depot, not blueprint, not anyone else's spot."""
        t = self.tick_count
        obstacles = self._obstacles_for(r)
        taken = {o.pos for o in self.robots} | {
            o.storage for o in self.robots if o.id != r.id
        }
        seen = {r.pos}
        frontier = [r.pos]
        candidates: list[Node] = []
        while frontier and len(seen) < 80:
            nxt = []
            for node in frontier:
                for nb in self.geometry.neighbors(node):
                    if nb in seen or nb in obstacles:
                        continue
                    seen.add(nb)
                    nxt.append(nb)
                    if (
                        nb != self.depot
                        and not self._is_blueprint(nb)
                        and nb not in taken
                    ):
                        candidates.append(nb)
            frontier = nxt
        candidates.sort(
            key=lambda n: -self.geometry.heuristic_distance(n, r.pos)
        )
        for goal in candidates[:3]:
            self.table.release_owner(r.id, t + 1)
            steps = plan_walk(
                r.id, r.pos, t, goal, self.geometry, self.table,
                static_obstacles=obstacles,
            )
            if steps:
                r.itinerary = steps
                r.state = "SIDESTEP"
                return
        self._try_hold(r)

    def _obstacles_for(self, r: SwarmRobot) -> frozenset:
        """Where OTHER robots are or will end up: walls for r's A*.

        A planless robot blocks its position; a robot with a plan blocks
        its plan's END node (its parking spot). Paths in between stay
        transient and are handled by leases. Without the anchor rule,
        short shuffle-walks flicker robots out of the obstacle set and
        long trains get planned through cells someone is about to park
        on — the source of every depot collision this suite caught.
        """
        return frozenset(
            o.anchor for o in self.robots if o.id != r.id
        )

    def _is_blueprint(self, node: Node) -> bool:
        try:
            return bool(self.world.blueprint[node])
        except (IndexError, TypeError):
            return False

    def _escape(self, r: SwarmRobot) -> None:
        """Someone's lease is inbound on this robot's cell and no task
        moved it: relocate — away from the depot first (the usual jam),
        then home, then depot-side. A* only returns timing-valid escapes.
        Failing everything is a real coordination breakdown, surfaced
        loudly."""
        self._yield_walk(r)
        if r.itinerary:
            r.state = "EVADING"
            return
        t = self.tick_count
        # Never escape TO the depot: it is the most future-leased cell in
        # the system, and a robot parked there meets the next pick train.
        for goal in (r.storage, r.start):
            if goal == r.pos or goal == self.depot:
                continue
            steps = plan_walk(
                r.id, r.pos, t, goal, self.geometry, self.table,
                static_obstacles=self._obstacles_for(r),
            )
            if steps:
                r.itinerary = steps
                r.state = "EVADING"
                return
        # Named goals unreachable (e.g. the inbound robot occupies the
        # only through-route): any safe nearby node beats standing still.
        self._sidestep_walk(r)
        if r.itinerary:
            r.state = "EVADING"
            return
        # Last resort: an urgent one-tick hop to any adjacent node —
        # min_clear=1 accepts brief refuges a normal park would reject.
        # Dead ends last: hopping into a degree-1 pocket (e.g. the depot
        # corner) ahead of an inbound train is how robots get cornered.
        hops = sorted(
            self.geometry.neighbors(r.pos),
            key=lambda nb: -len(list(self.geometry.neighbors(nb))),
        )
        for nb in hops:
            steps = plan_walk(
                r.id, r.pos, t, nb, self.geometry, self.table,
                static_obstacles=self._obstacles_for(r), min_clear=1,
            )
            if steps:
                r.itinerary = steps
                r.state = "EVADING"
                return
        # No escape found THIS tick. The best-effort hold protects the
        # robot up to the inbound lease, so retry next tick; the executor's
        # collision assert is the hard backstop if displacement truly fails.
        self._event(r, "escape_failed", r.pos)

    # -- execution -----------------------------------------------------------

    def _execute(self, r: SwarmRobot, step: Step) -> None:
        if step.action == MOVE:
            # Re-verify: a voxel placed since planning can pinch a corner
            # or fill a cell that was open. Abort + kickback on violation.
            if step.node not in set(self.geometry.neighbors(r.pos)):
                self._abort(r)
                return
            r.pos = step.node
            r.state = "MOVING"
            self._event(r, "move", step.node)
        elif step.action == WAIT:
            r.state = "WAITING"
        elif step.action == PICK:
            r.carrying = True
            r.state = "PICK"
            self._event(r, "pick", step.node)
        elif step.action == PLACE:
            defective = self._roll_defect(step.target)
            self.world.place_voxel(step.target, defective=defective)
            r.carrying = False
            r.state = "PLACE"
            self._event(r, "place", step.target)
            if not self.inspect_enabled:
                self.sequencer.complete(step.target)
                r.task = None
        elif step.action == INSPECT:
            r.state = "INSPECT"
            if self.world.is_defective(step.target):
                self.defects_found += 1
                # The spare reserved tick becomes the REMOVE.
                r.itinerary.insert(
                    0, Step(step.t + 1, r.pos, REMOVE, step.target)
                )
            else:
                self.sequencer.complete(step.target)
                r.task = None
        elif step.action == REMOVE:
            self.world.remove_voxel(step.target)
            self.table.clear_deed(step.target)
            r.state = "REMOVE"
            self._event(r, "defect_removed", step.target)
            self.sequencer.repair(step.target)
            r.task = None
            # Retraction re-verification: buildability gates that passed
            # while this cell was deeded/placed are now stale — an
            # in-flight neighbor placement may seal the reopened hole.
            # Abort nearby unplaced tasks; they re-gate on replan.
            self._abort_stale(step.target, exclude=r)
        else:  # pragma: no cover
            raise AssertionError(f"unknown action {step.action}")

    def _abort(self, r: SwarmRobot) -> None:
        """Plan invalidated mid-flight: release, kick the task back, hold."""
        t = self.tick_count
        r.itinerary = []
        self.table.release_owner(r.id, t + 1)
        self.table.reserve_hold(r.id, r.pos, t + 1, t + 1 + HOLD_HORIZON)
        if r.task is not None:
            retracted = r.task
            # Retract the deed only if the voxel was never placed (aborts
            # can only happen on MOVE steps, which precede PLACE).
            if (
                self.table.deed_holder(retracted) is not None
                and self.world.is_empty(retracted)
            ):
                self.table.clear_deed(retracted)
            self.sequencer.kickback(retracted)
            self.kickbacks += 1
            self._event(r, "kickback", retracted)
            r.task = None
            # Deed retraction staleness (see _abort_stale): bounded
            # recursion — every abort clears its robot's task first.
            self._abort_stale(retracted, exclude=r)
        if r.carrying:
            r.carrying = False  # scrap the part (conservative; rare)
        r.state = "IDLE"

    def _abort_stale(self, retracted: Node, exclude: SwarmRobot) -> None:
        """Abort in-flight tasks whose gate decisions may have relied on
        ``retracted`` being solid (proximity heuristic; conservative)."""
        for o in self.robots:
            if (
                o is not exclude
                and o.task is not None
                and self.world.is_empty(o.task)  # not yet placed
                and self.geometry.heuristic_distance(o.task, retracted) <= 2
            ):
                self._abort(o)

    # -- support -------------------------------------------------------------

    def _roll_defect(self, cell: Node) -> bool:
        if cell in self._defects_pending:
            self._defects_pending.discard(cell)
            return True
        return self.defect_p > 0 and self.rng.random() < self.defect_p

    def _event(self, r: SwarmRobot, kind: str, node: Node) -> None:
        self.events.append((self.tick_count + 1, r.id, kind, node))

    def _check_collisions(self) -> None:
        positions = [r.pos for r in self.robots]
        if len(positions) != len(set(positions)):
            raise AssertionError(
                f"collision at tick {self.tick_count}: {positions}"
            )

    def _watchdog(self) -> None:
        # Exhausted sequencer = nothing left to coordinate (done, homing,
        # or a no-inspect baseline stuck with defects by design): not a
        # stall. Also: never resequence a defect-bearing world.
        if (
            self.done
            or self.sequencer.exhausted
            or any(r.itinerary for r in self.robots)
        ):
            self._idle_with_work = 0
            return
        self._idle_with_work += 1
        if self._idle_with_work == RESEQUENCE_AFTER:
            if self._resequenced_at != self.tick_count:
                self.sequencer.resequence()
                self._resequenced_at = self.tick_count
        if self._idle_with_work >= STUCK_AFTER:
            raise SwarmStuck(
                f"no progress for {STUCK_AFTER} ticks at tick "
                f"{self.tick_count}; pending={len(self.sequencer.queue)}"
            )
