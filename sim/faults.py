# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Seeded fault injection. Rates are configuration, not a measurement.

Five channels, each a probability the caller sets:

* ``capture`` — a placement draw the precision model rejects. The draw
  is a scripted pose outside the claw envelope. :mod:`sim.precision`
  decides the miss. A cell with no mating voxel is a datum and seats,
  which is that model's rule, not a second coin flip.
* ``drop`` — a carried voxel is lost on the way to the site. The cell
  stays empty. The robot goes back to the depot.
* ``stall`` — the robot freezes mid-task, then the same task is routed
  again.
* ``robot_fail`` — the robot is down for a configured number of ticks
  and the task returns to the queue for someone else.
* ``fastener`` — one new face joint does not take. The face is a spring
  the joint model already names. The voxel is a bad bond, and the
  existing inspect / remove / replace path clears it.
* ``feed_outage`` — the feed station (the swarm's depot, or a shelter
  pickup) refuses a load for a configured duration.

Nothing here was fitted to a robot log. ``k`` is not an input. Sag and
joint demand elsewhere stay the unfitted ``k = 1``.

Recovery uses the stack that already exists. A miss or a failed
fastener is a defect: inspect, remove, resequence, re-place. A drop or
a stall releases the itinerary and the choreographer plans a new one
(re-route). A robot failure or a feed outage returns the task so
another robot or another station can take it (reassign). If the
resequence has no order and the finished structure is still inside the
caps, :func:`sim.scaffold.plan_with_scaffold` adds temporary support
and a removal. The stability, sag, and joint gates are the caller's.
This module does not turn them off.

A fault that starts while another fault is still open is a cascade.
So is a neighbor task aborted because a fault retracted a deed. The
cascade count is that consequence, not a second configured rate.
"""

from __future__ import annotations

from dataclasses import dataclass, field

# 5 cm along the face normal. The claw envelope is 1.34 cm, so the
# precision model rejects this draw. It is not a measured miss.
CAPTURE_MISS = (0.0, 0.0, 0.05, 0.0)

FAULT_KINDS = (
    "capture",
    "drop",
    "stall",
    "robot_fail",
    "fastener",
    "feed_outage",
)

RECOVERIES = ("re-place", "re-route", "reassign", "scaffold")

# Which recovery the channel asks for. A later replan can still retarget
# a stall onto ``reassign`` if the same robot cannot keep the task.
RECOVERY_FOR = {
    "capture": "re-place",
    "drop": "re-route",
    "stall": "re-route",
    "robot_fail": "reassign",
    "fastener": "re-place",
    "feed_outage": "reassign",
    "scaffold": "scaffold",
    "cascade": "reassign",
}


@dataclass(frozen=True)
class FaultRates:
    """Probabilities and durations. Configuration, not a histogram.

    Each probability is in ``[0, 1]``. Zero disables that channel and
    does not draw. Durations are the stall, the robot-down window, and
    the feed outage. Swarm code reads the tick fields. The shelter
    schedule reads ``outage_s`` and scales the tick fields by its step.
    """

    capture: float = 0.0
    drop: float = 0.0
    stall: float = 0.0
    robot_fail: float = 0.0
    fastener: float = 0.0
    feed_outage: float = 0.0
    stall_ticks: int = 3
    down_ticks: int = 8
    outage_ticks: int = 6
    outage_s: float = 40.0

    def __post_init__(self) -> None:
        for name in FAULT_KINDS:
            value = getattr(self, name)
            if not 0.0 <= float(value) <= 1.0:
                raise ValueError(f"{name} must be in [0, 1], got {value}")
        for name in ("stall_ticks", "down_ticks", "outage_ticks"):
            if int(getattr(self, name)) < 1:
                raise ValueError(f"{name} must be >= 1")
        if self.outage_s <= 0:
            raise ValueError("outage_s must be positive")

    def rate(self, kind: str) -> float:
        if kind not in FAULT_KINDS:
            raise KeyError(kind)
        return float(getattr(self, kind))

    @property
    def any(self) -> bool:
        return any(self.rate(kind) > 0.0 for kind in FAULT_KINDS)


@dataclass
class FaultEvent:
    """One injected fault, or one cascade that fault caused.

    ``rate`` is the configured probability of the channel. A cascade
    and a scaffold insertion record 0: they are not draws. ``recovery``
    is the action the existing path took. Times are swarm ticks or
    shelter seconds, whichever clock the caller passed.
    """

    kind: str
    rate: float
    seed: int
    recovery: str
    tick: int | None = None
    time_s: float | None = None
    robot: int | None = None
    role: str | None = None
    cell: tuple | None = None
    joint: tuple | None = None
    cascade: bool = False
    cause: int | None = None
    done_tick: int | None = None
    done_time_s: float | None = None
    utilization: float | None = None

    @property
    def done(self) -> bool:
        return self.done_tick is not None or self.done_time_s is not None

    @property
    def recovery_time(self) -> float | None:
        if self.done_tick is not None and self.tick is not None:
            return float(self.done_tick - self.tick)
        if self.done_time_s is not None and self.time_s is not None:
            return float(self.done_time_s - self.time_s)
        return None


def _as_cell(cell) -> tuple | None:
    if cell is None:
        return None
    return tuple(int(v) for v in cell)


@dataclass
class FaultInjector:
    """One seeded draw stream and the log of what it injected.

    ``script`` forces a sequence of channel names. While it is
    non-empty, :meth:`roll` returns true only for the head of the
    sequence and only that call consumes it. A rate of zero never
    draws, so an all-zero injector does not move :class:`random.Random`.
    """

    rates: FaultRates
    seed: int
    script: list[str] = field(default_factory=list)
    events: list[FaultEvent] = field(default_factory=list)
    _rng_state: object = field(default=None, repr=False)
    _open: list[int] = field(default_factory=list, repr=False)

    def __post_init__(self) -> None:
        import random

        self.seed = int(self.seed)
        self._rng = random.Random(self.seed)
        self.script = list(self.script)
        for name in self.script:
            if name not in FAULT_KINDS:
                raise ValueError(f"unknown fault kind {name!r}")

    def roll(self, kind: str) -> bool:
        """True if this channel fires. Zero rates do not consume the seed."""
        if kind not in FAULT_KINDS:
            raise KeyError(kind)
        if self.script:
            if self.script[0] != kind:
                return False
            self.script.pop(0)
            return True
        rate = self.rates.rate(kind)
        if rate <= 0.0:
            return False
        return self._rng.random() < rate

    def begin(
        self,
        kind: str,
        *,
        recovery: str | None = None,
        tick: int | None = None,
        time_s: float | None = None,
        robot: int | None = None,
        role: str | None = None,
        cell=None,
        joint=None,
        utilization: float | None = None,
        rate: float | None = None,
    ) -> int:
        """Append an open fault. A fault opened on top of another is a cascade."""
        if recovery is None:
            recovery = RECOVERY_FOR[kind]
        if recovery not in RECOVERIES:
            raise ValueError(f"unknown recovery {recovery!r}")
        cascade = bool(self._open)
        cause = self._open[-1] if cascade else None
        if rate is None:
            rate = 0.0 if kind not in FAULT_KINDS else self.rates.rate(kind)
        event = FaultEvent(
            kind=kind,
            rate=float(rate),
            seed=self.seed,
            recovery=recovery,
            tick=None if tick is None else int(tick),
            time_s=None if time_s is None else float(time_s),
            robot=None if robot is None else int(robot),
            role=role,
            cell=_as_cell(cell),
            joint=None if joint is None else tuple(_as_cell(side) for side in joint),
            cascade=cascade or kind == "cascade",
            cause=cause,
            utilization=None if utilization is None else float(utilization),
        )
        self.events.append(event)
        index = len(self.events) - 1
        self._open.append(index)
        return index

    def finish(
        self,
        index: int,
        *,
        tick: int | None = None,
        time_s: float | None = None,
    ) -> None:
        event = self.events[index]
        if event.done:
            return
        if tick is not None:
            event.done_tick = int(tick)
        if time_s is not None:
            event.done_time_s = float(time_s)
        if not event.done:
            return
        self._open = [item for item in self._open if item != index]

    def finish_kind(
        self,
        kind: str,
        *,
        robot: int | None = None,
        tick: int | None = None,
        time_s: float | None = None,
    ) -> None:
        for index in list(self._open):
            event = self.events[index]
            if event.kind != kind:
                continue
            if robot is not None and event.robot != robot:
                continue
            self.finish(index, tick=tick, time_s=time_s)

    def finish_cell(
        self,
        cell,
        *,
        tick: int | None = None,
        time_s: float | None = None,
        kinds: tuple[str, ...] | None = None,
    ) -> None:
        """Close open faults on ``cell``.

        Stalls and feed outages have their own clocks (a new route, a
        successful load) and are left open here.
        """
        target = _as_cell(cell)
        skip = {"stall", "feed_outage"}
        for index in list(self._open):
            event = self.events[index]
            if event.cell != target or event.kind in skip:
                continue
            if kinds is not None and event.kind not in kinds:
                continue
            self.finish(index, tick=tick, time_s=time_s)

    def retarget(self, kind: str, robot: int, recovery: str) -> None:
        if recovery not in RECOVERIES:
            raise ValueError(recovery)
        for index in reversed(self._open):
            event = self.events[index]
            if event.kind == kind and event.robot == robot:
                event.recovery = recovery
                return

    @property
    def n_cascades(self) -> int:
        return sum(1 for event in self.events if event.cascade)

    def recovery_times(self) -> list[float]:
        return [
            value
            for value in (event.recovery_time for event in self.events)
            if value is not None
        ]


def face_partner(cell, occupied) -> tuple | None:
    """One already-occupied face neighbor, or None.

    The choice is the lexicographically first neighbor, so a seed
    repeats. This is the face the joint model treats as one spring.
    Coordinate arithmetic stays in this module.
    """
    cell_t = _as_cell(cell)
    if cell_t is None or len(cell_t) not in (2, 3):
        return None
    present = {_as_cell(item) for item in occupied}
    found = []
    for axis in range(len(cell_t)):
        for sign in (-1, 1):
            other = list(cell_t)
            other[axis] += sign
            other_t = tuple(other)
            if other_t in present:
                found.append(other_t)
    if not found:
        return None
    return min(found)


def fastener_joint(cell, occupied, policy, cap):
    """Name the new face spring and, when it is cheap, its utilization.

    ``cap`` is a :class:`~sim.stability.JointCap`. The solve is the
    existing joint model at that cap's ``k``. Structures larger than
    64 cells skip the extra solve; the partner is still the face.
    Returns ``(joint, utilization)``. ``joint`` is ``None`` when the
    cell has no voxel neighbor yet (nothing to fasten).
    """
    partner = face_partner(cell, occupied)
    if partner is None:
        return None, None
    joint = (_as_cell(cell), partner)
    utilization = None
    cells = [_as_cell(item) for item in occupied]
    cell_t = _as_cell(cell)
    if cell_t not in cells:
        cells.append(cell_t)
    if cap is not None and policy is not None and len(cells) <= 64:
        from .stability import assess_joints

        try:
            report = assess_joints(cells, policy, cap)
        except ValueError:
            report = None
        if report is not None and report.worst is not None:
            pair = {report.worst.a, report.worst.b}
            if pair == set(joint):
                utilization = float(report.worst.utilization)
            else:
                utilization = float(report.worst_utilization)
    return joint, utilization


def script_capture_miss(model) -> None:
    """Queue one out-of-envelope draw on a precision model."""
    model.scripted.append(CAPTURE_MISS)
