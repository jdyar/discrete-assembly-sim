# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""ARMADAS shelter throughput model.

This is not the cooperative-A* choreographer and not a SOLL-E joint
trajectory. It is a three-role schedule on the cubic cell grid: cargo
robots load at a feed station and carry a voxel to its cell, a crane
places it, and a fastening robot closes each new face joint as its own
task. The structural gate is on. ``k = mass = g = 1``. Sag is in units
of ``mass·g/k`` and has not been fit.

What was checked against Gregg et al., Science Robotics 9, eadi2746
(2024), doi:10.1126/scirobotics.adi2746. The text below was read from
the NTRS distribution of that manuscript (20230005194), which is the
preprint-equivalent PDF. The Science HTML abstract matches it on the
points quoted here.

Confirmed from that manuscript
------------------------------
* 256 voxels in the shelter (laboratory construction test; Fig. 5).
* Bounding-box edge 304.8 mm (Fig. 1: "a cube with 304.8mm side lengths").
* Cumulative build time "approximately 100.4 hours or 4.2 days" of
  assembly with pauses and stoppages removed (Materials and Methods,
  "256 Voxel Demonstration Build"). 4.2 × 24 h = 100.8 h. Both figures
  are printed in the same sentence. Dividing 256 × (304.8 mm)³ by
  either clock rounds to the reported 20,000 mm³/s. This module does
  not pick one of them as a fit target.
* Three roles. Cargo SOLL-E carries a voxel from the pickup. Crane
  SOLL-E takes it off the cargo robot and places it. MMIC-I, the
  internal robot, fastens the new cell. The laboratory crew was two
  transport robots and one fastening robot.
* One pickup in the demonstration. The manuscript says voxels were
  loaded "at the beginning of the seed structure", plans cargo through
  "the depot node d", and attributes per-voxel time variation to
  "the distance the voxel needs to be transported from the voxel pick
  up location". Future work on the clock is named in the same section:
  "increased robot speed, more efficient path planning, multiple feed
  stations, parallelization, and more". It does not print the phrase
  "1 feed station". The single-station case is that one pickup.
* Five-voxel seed, and the robots started on it.
* Positioning capture envelopes (body text, unless noted):
  petals 3.0 cm in-plane; external grippers 1.34 cm out-of-plane;
  placement required within ±1.78 cm and ±3°; skis ±1.78 cm in-plane,
  enforcing <0.1 mm when extended; claws 1.34 cm out-of-plane. The
  body text says the claws enforce <0.1 mm at full grip. The Fig. 4
  caption says the claws enforce <0.2 mm in Y. Both are printed.
  Fastener actuation accommodates up to 0.2 cm in Y. The schedule
  places cells on grid centers. It does not add this error as noise.
  The placement draw and the ski/claw snap are ``sim.precision``, not
  this clock.

Not in that manuscript, and labeled as such
--------------------------------------------
* No per-action duration in seconds. Fig. S7 (main text) and Figure S4
  (Materials and Methods) are each described as the per-voxel elapsed
  time. The numbers are not in the text. Which figure is the right
  citation is not reconciled here.
* Park et al., IROS 2023 (NTRS 20230011353), the SOLL-E paper Gregg
  cites for the transport robot, says it took "on the order of 20
  seconds" for one unit step, on the earlier 164-voxel test. That is
  the configured step duration. It is not a Gregg 2024 measurement
  and it was not adjusted to approach 100.4 h.
* Feed loading, handoff, placement, and one fastening action per joint
  have no reported duration. They are configuration, set equal to that
  same 20 s quantum so the model has one timescale. They were chosen
  before the sweep was run and were not edited afterward.
* The cell list is not Fig. 5. The figure is a photograph and two CAD
  views. No coordinate table is in the manuscript. The shed below is
  an open-front 256-voxel stand-in with a five-voxel seed line and a
  roof eave, so the published count, the seed, and an overhang all
  exist. The second roof course is what makes the count 256. It is
  not a published course count. The lab robot bolted 596 faces. This
  geometry does not.

The sweep of feed stations and robots per role is the parallelism the
manuscript lists as future work. It is not a result from the paper.
"""

from __future__ import annotations

import heapq
from dataclasses import dataclass, field

from .stability import (
    SAG_ATOL,
    StabilityPolicy,
    assess_stability,
    estimate_sag,
)

# --- manuscript figures -----------------------------------------------------

N_VOXELS = 256
EDGE_MM = 304.8
# Both clocks are printed. 4.2 days is 100.8 h, not 100.4 h.
REPORTED_BUILD_HOURS = 100.4
REPORTED_BUILD_DAYS = 4.2
REPORTED_DAYS_AS_HOURS = 4.2 * 24.0
REPORTED_THROUGHPUT_MM3_S = 20_000.0

# Capture envelopes. Claw enforcement disagrees between body and caption.
PETAL_IN_PLANE_CM = 3.0
GRIPPER_OUT_OF_PLANE_CM = 1.34
PLACE_TOLERANCE_CM = 1.78
PLACE_TOLERANCE_DEG = 3.0
SKI_IN_PLANE_CM = 1.78
SKI_ENFORCED_MM = 0.1
CLAW_OUT_OF_PLANE_CM = 1.34
CLAW_ENFORCED_MM_BODY = 0.1
CLAW_ENFORCED_MM_CAPTION = 0.2
FASTENER_MISALIGN_CM = 0.2

# Operation totals. These are counts, not durations, and not sim targets.
CARGO_PRIMITIVES = 14_185
CRANE_PRIMITIVES = 478
FASTENER_STEPS = 711
FACES_BOLTED = 596
CARGO_BODY_LENGTHS = 4624
CARGO_TRAVEL_KM = 3.15
CRANE_BODY_LENGTHS = 522
CRANE_TRAVEL_KM = 0.356
FASTENER_BODY_LENGTHS = 754
FASTENER_TRAVEL_KM = 0.230
SEED_COUNT = 5

# --- configured durations ---------------------------------------------------

# Park et al. 2023, order-of-magnitude, 164-voxel test. Not Gregg's clock.
STEP_S = 20.0
STEP_SOURCE = "park-2023-order-of-20s"

# Not reported. Equal to STEP_S so there is one timescale. Not fitted.
FEED_LOAD_S = 20.0
HANDOFF_S = 20.0
PLACE_S = 20.0
FASTEN_JOINT_S = 20.0
CONFIG_SOURCE = "config"

K = 1.0
MASS = 1.0
G = 1.0

CARGO = "cargo"
CRANE = "crane"
FASTEN = "fasten"
ROLES = (CARGO, CRANE, FASTEN)

_FACE = (
    (1, 0, 0),
    (-1, 0, 0),
    (0, 1, 0),
    (0, -1, 0),
    (0, 0, 1),
    (0, 0, -1),
)


def face_neighbors(cell: tuple[int, int, int]) -> tuple[tuple[int, int, int], ...]:
    level, row, col = cell
    return tuple(
        (level + a, row + b, col + c) for a, b, c in _FACE
    )


def manhattan(a: tuple[int, ...], b: tuple[int, ...]) -> int:
    return abs(a[0] - b[0]) + abs(a[1] - b[1]) + abs(a[2] - b[2])


def shelter_cells() -> tuple[tuple[int, int, int], ...]:
    """Open-front shed of exactly 256 cells. Not the Fig. 5 coordinate list.

    Floor at level 1, rows 1..7, cols 0..7 (8 × 7). Walls on the left
    column, the right column, and the back row, levels 2..5. The front
    is open. A roof course at level 6 covers the footprint and cantilevers
    one cell to row 0 (the eave). A second roof course at level 7 covers
    the footprint only. That second course is how the count lands on 256.
    """
    cells: set[tuple[int, int, int]] = set()
    width, depth = 8, 7
    floor_rows = range(1, depth + 1)
    for row in floor_rows:
        for col in range(width):
            cells.add((1, row, col))
    for level in range(2, 6):
        for row in floor_rows:
            cells.add((level, row, 0))
            cells.add((level, row, width - 1))
        for col in range(1, width - 1):
            cells.add((level, depth, col))
    for row in floor_rows:
        for col in range(width):
            cells.add((6, row, col))
    for col in range(width):
        cells.add((6, 0, col))
    for row in floor_rows:
        for col in range(width):
            cells.add((7, row, col))
    if len(cells) != N_VOXELS:
        raise RuntimeError(f"shelter cell count {len(cells)} != {N_VOXELS}")
    return tuple(sorted(cells))


def shelter_seed() -> tuple[tuple[int, int, int], ...]:
    """Five collinear floor voxels at the open front, cols 0..4.

    The manuscript's seed is five voxels. Park et al. describe that
    seed as a line. The column the cargo is loaded from is the start
    of the line. Which five cells of Fig. 5 those were is not printed.
    """
    return tuple((1, 1, col) for col in range(SEED_COUNT))


def feed_station_cells(n_stations: int) -> tuple[tuple[int, int, int], ...]:
    """Pickup cells on the apron in front of the seed.

    Station 0 is level 1, row 0, col 0: the empty cell directly in
    front of the first seed voxel. That is the one demonstrated pickup,
    placed at the beginning of the seed. Stations 1..3 continue along
    that apron. Their positions are configuration. The manuscript does
    not locate a second feed station because the demonstration had one
    pickup.
    """
    if n_stations < 1:
        raise ValueError("n_stations must be >= 1")
    return tuple((1, 0, index) for index in range(n_stations))


@dataclass(frozen=True)
class ActionTimes:
    """Seconds per modeled action.

    ``step_s`` is the Park et al. order-of-magnitude figure. The other
    four are configuration. See the module docstring.
    """

    step_s: float = STEP_S
    feed_load_s: float = FEED_LOAD_S
    handoff_s: float = HANDOFF_S
    place_s: float = PLACE_S
    fasten_joint_s: float = FASTEN_JOINT_S

    def __post_init__(self) -> None:
        for name in (
            "step_s",
            "feed_load_s",
            "handoff_s",
            "place_s",
            "fasten_joint_s",
        ):
            if float(getattr(self, name)) <= 0:
                raise ValueError(f"{name} must be positive")

    def sources(self) -> dict[str, str]:
        return {
            "step_s": STEP_SOURCE,
            "feed_load_s": CONFIG_SOURCE,
            "handoff_s": CONFIG_SOURCE,
            "place_s": CONFIG_SOURCE,
            "fasten_joint_s": CONFIG_SOURCE,
        }


@dataclass(frozen=True)
class GatedOrder:
    """One placement order whose every prefix passes both gates.

    ``order`` excludes the seed. ``joints[i]`` are the already-built
    face neighbors of ``order[i]`` (seed or an earlier voxel). Each of
    those is one fastening task. ``deps[i]`` indexes the earlier voxels
    that share a face; they have to be fastened before this one is placed.
    """

    cells: tuple[tuple[int, int, int], ...]
    seed: tuple[tuple[int, int, int], ...]
    order: tuple[tuple[int, int, int], ...]
    joints: tuple[tuple[tuple[int, int, int], ...], ...]
    deps: tuple[tuple[int, ...], ...]
    finished_sag: float
    peak_sag: float
    peak_sag_cell: tuple[int, int, int] | None
    worst_overhang: int
    policy: StabilityPolicy


def _policy(max_cantilever: int | None, max_sag: float | None) -> StabilityPolicy:
    return StabilityPolicy(
        up_axis=0,
        up_sign=1,
        ground_plane=0,
        max_cantilever=max_cantilever,
        max_sag=max_sag,
    )


def _adjacent_to(cell: tuple[int, int, int], occupied: set[tuple[int, int, int]]) -> bool:
    return any(neighbor in occupied for neighbor in face_neighbors(cell))


def build_gated_order(
    cells: tuple[tuple[int, int, int], ...] | set[tuple[int, int, int]],
    seed: tuple[tuple[int, int, int], ...],
) -> GatedOrder:
    """Place ``cells`` starting from ``seed``, gates on, ``k = 1``.

    The cap is the finished structure: tightest integer cantilever that
    accepts it, and ``worst_sag`` at ``k = mass = g = 1``. Cells are
    added in order of finished overhang, then level, then lexicographic
    cell. Every prefix is checked. A prefix that fails is an error, not
    a silent skip: this stand-in was chosen so that order stays inside
    the finished caps.
    """
    occupied = tuple(sorted(set(cells)))
    seed_t = tuple(seed)
    seed_set = set(seed_t)
    if not seed_set <= set(occupied):
        raise ValueError("seed is not a subset of the structure")
    if len(seed_set) != len(seed_t):
        raise ValueError("seed cells must be unique")
    frame = _policy(None, None)
    finished = assess_stability(occupied, frame)
    if not finished.stable:
        raise RuntimeError(f"finished structure is not connected: {finished.reason}")
    limit = int(finished.worst_overhang)
    sag = estimate_sag(occupied, frame, k=K, mass=MASS, g=G)
    cap = float(sag.worst_sag)
    policy = _policy(limit, cap)
    overhang = dict(finished.overhang)
    remaining = sorted(
        (cell for cell in occupied if cell not in seed_set),
        key=lambda cell: (overhang[cell], cell[0], cell[1], cell[2]),
    )
    built = set(seed_set)
    order: list[tuple[int, int, int]] = []
    peak = 0.0
    peak_cell: tuple[int, int, int] | None = None
    for cell in remaining:
        if not _adjacent_to(cell, built):
            raise RuntimeError(f"{cell} is not face-adjacent to the prefix")
        built.add(cell)
        report = assess_stability(built, policy)
        estimate = estimate_sag(built, policy, k=K, mass=MASS, g=G)
        if estimate.worst_sag > peak:
            peak = float(estimate.worst_sag)
            peak_cell = estimate.worst_sag_cell
        if not report.stable or estimate.worst_sag > cap + SAG_ATOL:
            raise RuntimeError(
                f"prefix of {len(built)} fails the finished caps at {cell}: "
                f"stable={report.stable} sag={estimate.worst_sag:.6g} cap={cap:.6g}"
            )
        order.append(cell)
    joints: list[tuple[tuple[int, int, int], ...]] = []
    deps: list[tuple[int, ...]] = []
    index = {cell: i for i, cell in enumerate(order)}
    for i, cell in enumerate(order):
        partners: list[tuple[int, int, int]] = []
        earlier: list[int] = []
        for neighbor in face_neighbors(cell):
            if neighbor in seed_set or (neighbor in index and index[neighbor] < i):
                partners.append(neighbor)
            if neighbor in index and index[neighbor] < i:
                earlier.append(index[neighbor])
        if not partners:
            raise RuntimeError(f"{cell} attaches with no joint")
        joints.append(tuple(sorted(partners)))
        deps.append(tuple(sorted(earlier)))
    return GatedOrder(
        cells=occupied,
        seed=seed_t,
        order=tuple(order),
        joints=tuple(joints),
        deps=tuple(deps),
        finished_sag=cap,
        peak_sag=peak,
        peak_sag_cell=peak_cell,
        worst_overhang=limit,
        policy=policy,
    )


_SHELTER: GatedOrder | None = None


def shelter_model() -> GatedOrder:
    """The 256-voxel stand-in, order cached for the process."""
    global _SHELTER
    if _SHELTER is None:
        _SHELTER = build_gated_order(shelter_cells(), shelter_seed())
    return _SHELTER


def passes_gate(
    occupied: set[tuple[int, int, int]] | tuple,
    policy: StabilityPolicy,
    cap: float,
) -> tuple[bool, float]:
    """Cantilever screen and sag cap. Returns (ok, worst_sag).

    An empty set is ok and sags 0. A set the spring network cannot tie
    to the base fails.
    """
    cells = list(occupied)
    if not cells:
        return True, 0.0
    report = assess_stability(cells, policy)
    if not report.stable:
        return False, float("inf")
    try:
        estimate = estimate_sag(cells, policy, k=K, mass=MASS, g=G)
    except ValueError:
        return False, float("inf")
    return estimate.worst_sag <= cap + SAG_ATOL, float(estimate.worst_sag)


def inset_boxes_disjoint(
    cells: tuple[tuple[int, int, int], ...] | set[tuple[int, int, int]],
    inset: float = 0.96,
) -> bool:
    """Axis-aligned boxes of half-extent ``inset / 2`` do not overlap.

    Pitch is 1. ``inset`` 0.96 is the viewer's cuboctahedron scale, so
    face-adjacent solids keep a gap. ``inset`` 1 meets at a face and
    still has no interior overlap.
    """
    if inset <= 0 or inset > 1:
        raise ValueError("inset must be in (0, 1]")
    half = inset / 2.0
    unique = list(set(cells))
    if len(unique) != len(list(cells)):
        return False
    for i, a in enumerate(unique):
        for b in unique[i + 1 :]:
            separated = False
            for axis in range(3):
                if abs(a[axis] - b[axis]) >= 2 * half:
                    separated = True
                    break
            if not separated:
                return False
    return True


@dataclass
class SimResult:
    """One schedule. Times are seconds. Sag is ``mass·g/k`` at ``k = 1``."""

    n_feeds: int
    robots_per_role: int
    makespan_s: float
    cargo_busy_s: float
    crane_busy_s: float
    fasten_busy_s: float
    feed_busy_s: float
    cargo_util: float
    crane_util: float
    fasten_util: float
    feed_util: float
    bottleneck: str
    n_placed: int
    n_fasten_tasks: int
    n_joints: int
    cargo_steps: int
    crane_steps: int
    fasten_steps: int
    peak_sag: float
    finished_sag: float
    worst_overhang: int
    place_order: tuple[tuple[int, int, int], ...] = field(repr=False)
    events: tuple[tuple[float, str, str, int], ...] = field(repr=False)
    # Fault injection. Zero when the schedule was not given a FaultInjector.
    # Rates that produced n_faults are configuration, not a measurement.
    n_faults: int = 0
    n_cascades: int = 0
    n_scaffold: int = 0

    @property
    def makespan_h(self) -> float:
        return self.makespan_s / 3600.0

    def ratio_to_reported(self, reported_h: float = REPORTED_BUILD_HOURS) -> float:
        return self.makespan_h / reported_h


@dataclass
class _Job:
    index: int
    cell: tuple[int, int, int]
    partners: tuple[tuple[int, int, int], ...]
    deps: tuple[int, ...]
    stage: str = "queued"
    joints_done: int = 0
    inflight: int = 0
    holder: int | None = None

    @property
    def fastened(self) -> bool:
        return self.stage == "done"


@dataclass
class _Robot:
    role: str
    index: int
    pos: tuple[int, int, int]
    free: bool = True
    busy: float = 0.0
    job: int | None = None
    held_since: float | None = None
    down_until: float = 0.0
    stall_fault: int | None = None


@dataclass
class _Station:
    index: int
    pos: tuple[int, int, int]
    serving: int | None = None
    waiting: list[int] = field(default_factory=list)
    inbound: int = 0
    busy: float = 0.0
    down_until: float = 0.0


def simulate(
    model: GatedOrder,
    n_feeds: int = 1,
    robots_per_role: int = 1,
    times: ActionTimes | None = None,
    stations: tuple[tuple[int, int, int], ...] | None = None,
    check_gate: bool = True,
    faults=None,
    precision=None,
    joint_cap=None,
) -> SimResult:
    """Run one shelter (or a smaller gated order) to completion.

    Cargo robots load at a feed station, one robot per station at a time,
    then carry the voxel to its cell. A crane places it only if every
    face-neighbor dependency is already fastened and the cantilever and
    sag caps accept the fastened set plus this cell. Each new joint is
    then its own fasten task. Robots of one role are identical. They do
    not reserve lattice space against each other; the only shared
    exclusion is the feed station. That is the bottleneck this schedule
    is for.

    Fetching does not skip a hole. The earliest queued voxel that is not
    yet legal blocks later fetches, so a cargo cannot hold a cell the
    gate will never return to. Placement may still run several legal
    frontier cells at once, which is what extra cranes are for.

    ``faults`` is an optional :class:`~sim.faults.FaultInjector`. None
    leaves the schedule unchanged. Rates on it are configuration, not a
    measured histogram. ``precision`` is the capture model a capture
    fault is tested against. ``joint_cap`` is the utilization gate, off
    when omitted. ``k`` on that cap is the caller's; this module's sag
    gate stays ``k = 1``.
    """
    clock = times or ActionTimes()
    if robots_per_role < 1:
        raise ValueError("robots_per_role must be >= 1")
    station_cells = stations if stations is not None else feed_station_cells(n_feeds)
    if len(station_cells) != n_feeds:
        raise ValueError("stations must have one cell per feed")
    if len(set(station_cells)) != len(station_cells):
        raise ValueError("feed stations must be distinct")

    jobs = [
        _Job(i, cell, model.joints[i], model.deps[i])
        for i, cell in enumerate(model.order)
    ]
    origin = model.seed[0]
    cargos = [
        _Robot(CARGO, i, station_cells[i % len(station_cells)])
        for i in range(robots_per_role)
    ]
    cranes = [_Robot(CRANE, i, origin) for i in range(robots_per_role)]
    fasteners = [_Robot(FASTEN, i, origin) for i in range(robots_per_role)]
    feeds = [_Station(i, pos) for i, pos in enumerate(station_cells)]
    if precision is not None:
        for cell in model.seed:
            if cell not in precision.poses:
                precision.note_datum(cell)
    import random as _random

    precision_rng = _random.Random(0 if faults is None else faults.seed)
    fastened: set[tuple[int, int, int]] = set(model.seed)
    peak = 0.0
    events: list[tuple[float, str, str, int]] = []
    place_order: list[tuple[int, int, int]] = []
    cargo_steps = 0
    crane_steps = 0
    fasten_steps = 0
    t = 0.0
    seq = 0
    heap: list[tuple[float, int, str, tuple]] = []

    def push(delay: float, kind: str, payload: tuple) -> None:
        nonlocal seq
        seq += 1
        heapq.heappush(heap, (t + delay, seq, kind, payload))

    def log(role: str, action: str, job_index: int) -> None:
        events.append((t, role, action, job_index))

    def deps_done(job: _Job) -> bool:
        return all(jobs[dep].fastened for dep in job.deps)

    def committed(extra: tuple[int, int, int] | None = None) -> set[tuple[int, int, int]]:
        """Fastened cells plus voxels a crane has already committed to place.

        An unfastened voxel is not support for a dependency, but it is
        in the structure the sag cap has to accept. Two cranes must not
        each pass a cap that only the pair violates.
        """
        cells = set(fastened)
        for job in jobs:
            if job.stage in ("placing", "placed", "done"):
                cells.add(job.cell)
        if extra is not None:
            cells.add(extra)
        return cells

    def gate_ok(cell: tuple[int, int, int]) -> tuple[bool, float]:
        if not check_gate:
            return True, 0.0
        ok, sag = passes_gate(committed(cell), model.policy, model.finished_sag)
        if not ok:
            return False, sag
        if joint_cap is not None:
            from .stability import joints_within

            if not joints_within(committed(cell), model.policy, joint_cap):
                return False, sag
        return True, sag

    def _requeued_by_fault(cell: tuple[int, int, int]) -> bool:
        if faults is None:
            return False
        for index in faults._open:
            event = faults.events[index]
            if event.cell == cell and event.kind in ("capture", "drop", "feed_outage"):
                return True
        return False

    def close_role_fail(role: str, cell: tuple[int, int, int]) -> None:
        if faults is None:
            return
        for index in list(faults._open):
            event = faults.events[index]
            if (
                event.kind == "robot_fail"
                and event.role == role
                and event.cell == cell
            ):
                faults.finish(index, time_s=t)

    def close_finished_cell(cell: tuple[int, int, int]) -> None:
        """A finished voxel closes recoveries that were waiting on it.

        A second robot can fail on a cell the first robot already
        claimed. That extra failure has no later claim of its own.
        The voxel finishing is the recovery.
        """
        if faults is None:
            return
        for index in list(faults._open):
            event = faults.events[index]
            if event.cell == cell and event.kind in ("robot_fail", "cascade", "fastener"):
                faults.finish(index, time_s=t)

    def stall_extra(robot: _Robot, cell: tuple[int, int, int]) -> float:
        """Configured stall, in seconds. Zero when faults are off."""
        if faults is None or not faults.roll("stall"):
            return 0.0
        extra = faults.rates.stall_ticks * clock.step_s
        robot.stall_fault = faults.begin(
            "stall",
            time_s=t,
            robot=robot.index,
            role=robot.role,
            cell=cell,
        )
        return extra

    def close_stall(robot: _Robot) -> None:
        if faults is None or robot.stall_fault is None:
            return
        faults.finish(robot.stall_fault, time_s=t)
        robot.stall_fault = None

    def fail_robot(robot: _Robot, cell: tuple[int, int, int] | None) -> bool:
        """Take the robot down and leave the job for someone else."""
        if faults is None or robot.down_until > t:
            return False
        if not faults.roll("robot_fail"):
            return False
        robot.down_until = t + faults.rates.down_ticks * clock.step_s
        faults.begin(
            "robot_fail",
            time_s=t,
            robot=robot.index,
            role=robot.role,
            cell=cell,
        )
        push(robot.down_until - t, "robot_up", (robot.role, robot.index))
        return True

    def start_load(station: _Station, cargo: _Robot) -> None:
        job = jobs[cargo.job]  # type: ignore[index]
        if faults is not None and (
            station.down_until > t or faults.roll("feed_outage")
        ):
            if station.down_until <= t:
                station.down_until = t + faults.rates.outage_s
                faults.begin(
                    "feed_outage",
                    time_s=t,
                    robot=cargo.index,
                    role=CARGO,
                    cell=job.cell,
                )
                push(faults.rates.outage_s, "outage_done", (station.index,))
            job.stage = "queued"
            job.holder = None
            if cargo.held_since is not None:
                cargo.busy += t - cargo.held_since
                cargo.held_since = None
            cargo.free = True
            cargo.job = None
            cargo.pos = station.pos
            station.serving = None
            station.waiting = [item for item in station.waiting if item != cargo.index]
            return
        if cargo.held_since is not None:
            cargo.busy += t - cargo.held_since
            cargo.held_since = None
        station.serving = cargo.index
        station.waiting = [item for item in station.waiting if item != cargo.index]
        job.stage = "loading"
        station.busy += clock.feed_load_s
        cargo.busy += clock.feed_load_s
        log(CARGO, "load", job.index)
        if faults is not None:
            faults.finish_kind("feed_outage", time_s=t)
        push(clock.feed_load_s, "load_done", (cargo.index, station.index))

    def dispatch() -> None:
        nonlocal cargo_steps, crane_steps, fasten_steps
        # Serve cargos already queued at a free station.
        for station in feeds:
            if station.serving is not None or not station.waiting:
                continue
            cargo = cargos[station.waiting[0]]
            start_load(station, cargo)

        # Fetch a ready prefix. Stop at the first queued voxel the gate
        # or its dependencies do not yet allow, so later cells cannot
        # leapfrog a hole.
        for cargo in cargos:
            if not cargo.free or cargo.down_until > t:
                continue
            chosen: _Job | None = None
            for job in jobs:
                if job.stage != "queued":
                    continue
                if not deps_done(job):
                    chosen = None
                    break
                ok, sag = gate_ok(job.cell)
                if not ok:
                    chosen = None
                    break
                chosen = job
                break
            if chosen is None:
                break
            if fail_robot(cargo, chosen.cell):
                continue
            available = [item for item in feeds if item.down_until <= t]
            if not available:
                break
            station = min(
                available,
                key=lambda item: (
                    item.inbound + (1 if item.serving is not None else 0) + len(item.waiting),
                    manhattan(cargo.pos, item.pos),
                    item.index,
                ),
            )
            chosen.stage = "to_feed"
            chosen.holder = cargo.index
            cargo.free = False
            cargo.job = chosen.index
            station.inbound += 1
            steps = manhattan(cargo.pos, station.pos)
            cargo_steps += steps
            travel = steps * clock.step_s + stall_extra(cargo, chosen.cell)
            cargo.busy += travel
            log(CARGO, "to_feed", chosen.index)
            close_role_fail(CARGO, chosen.cell)
            push(travel, "arrive_feed", (cargo.index, station.index))

        # Place the ready holding prefix. Several may be legal at once.
        for crane in cranes:
            if not crane.free or crane.down_until > t:
                continue
            chosen_job: _Job | None = None
            for job in jobs:
                if job.stage in ("done", "placing", "placed"):
                    continue
                if job.stage != "holding" or not deps_done(job):
                    # A capture, drop, or feed fault puts a cell back in
                    # the queue after a later cell is already at the site.
                    # Both cargos can be the ones holding those later
                    # cells, so refusing to place them waits forever for
                    # a fetch that has no free cargo. Skip only a cell
                    # with that fault still open. An unfaulted hole still
                    # blocks, which keeps the rate-0 schedule identical.
                    if _requeued_by_fault(job.cell):
                        continue
                    break
                ok, _sag = gate_ok(job.cell)
                if not ok:
                    break
                chosen_job = job
                break
            if chosen_job is None:
                break
            if fail_robot(crane, chosen_job.cell):
                continue
            crane.free = False
            crane.job = chosen_job.index
            chosen_job.stage = "placing"
            steps = manhattan(crane.pos, chosen_job.cell)
            crane_steps += steps
            travel = steps * clock.step_s + stall_extra(crane, chosen_job.cell)
            crane.busy += travel
            log(CRANE, "to_place", chosen_job.index)
            close_role_fail(CRANE, chosen_job.cell)
            push(travel, "start_handoff", (crane.index, chosen_job.index))

        # One fasten task per remaining joint. Joints of one voxel can
        # run at the same time, on different fastening robots.
        slots: list[int] = []
        for job in jobs:
            if job.stage != "placed":
                continue
            open_joints = len(job.partners) - job.joints_done - job.inflight
            slots.extend([job.index] * open_joints)
        for bot in fasteners:
            if not bot.free or not slots:
                break
            if bot.down_until > t:
                continue
            job = jobs[slots.pop(0)]
            if fail_robot(bot, job.cell):
                slots.insert(0, job.index)
                continue
            job.inflight += 1
            bot.free = False
            bot.job = job.index
            steps = manhattan(bot.pos, job.cell)
            fasten_steps += steps
            travel = steps * clock.step_s + stall_extra(bot, job.cell)
            bot.busy += travel + clock.fasten_joint_s
            log(FASTEN, "fasten", job.index)
            close_role_fail(FASTEN, job.cell)
            push(travel + clock.fasten_joint_s, "fasten_done", (bot.index, job.index))

    def on_arrive_feed(cargo_index: int, station_index: int) -> None:
        cargo = cargos[cargo_index]
        station = feeds[station_index]
        job = jobs[cargo.job]  # type: ignore[index]
        station.inbound -= 1
        cargo.pos = station.pos
        close_stall(cargo)
        job.stage = "at_feed"
        if station.serving is None and not station.waiting:
            start_load(station, cargo)
        else:
            station.waiting.append(cargo.index)
            cargo.held_since = t

    def on_load_done(cargo_index: int, station_index: int) -> None:
        cargo = cargos[cargo_index]
        station = feeds[station_index]
        job = jobs[cargo.job]  # type: ignore[index]
        station.serving = None
        if faults is not None and faults.roll("drop"):
            faults.begin(
                "drop",
                time_s=t,
                robot=cargo.index,
                role=CARGO,
                cell=job.cell,
            )
            job.stage = "queued"
            job.holder = None
            cargo.free = True
            cargo.job = None
            cargo.pos = station.pos
            return
        job.stage = "to_site"
        steps = manhattan(cargo.pos, job.cell)
        cargo_steps_add = steps
        travel = steps * clock.step_s
        cargo.busy += travel
        cargo.pos = job.cell
        log(CARGO, "carry", job.index)
        push(travel, "arrive_site", (cargo.index, cargo_steps_add))

    def on_arrive_site(cargo_index: int, steps: int) -> None:
        nonlocal cargo_steps
        cargo = cargos[cargo_index]
        job = jobs[cargo.job]  # type: ignore[index]
        cargo_steps += steps
        job.stage = "holding"
        cargo.held_since = t
        log(CARGO, "hold", job.index)

    def on_start_handoff(crane_index: int, job_index: int) -> None:
        crane = cranes[crane_index]
        job = jobs[job_index]
        cargo = cargos[job.holder]  # type: ignore[index]
        if cargo.held_since is not None:
            cargo.busy += t - cargo.held_since
            cargo.held_since = None
        crane.pos = job.cell
        close_stall(crane)
        cargo.busy += clock.handoff_s
        crane.busy += clock.handoff_s
        log(CARGO, "handoff", job.index)
        log(CRANE, "handoff", job.index)
        push(clock.handoff_s, "handoff_done", (crane.index, job.index))

    def on_handoff_done(crane_index: int, job_index: int) -> None:
        job = jobs[job_index]
        cargo = cargos[job.holder]  # type: ignore[index]
        cargo.free = True
        cargo.job = None
        cargo.pos = job.cell
        job.holder = None
        cranes[crane_index].busy += clock.place_s
        log(CRANE, "place", job.index)
        push(clock.place_s, "place_done", (crane_index, job_index))

    def _capture_rejects(cell: tuple[int, int, int]) -> bool:
        if faults is None or not faults.roll("capture"):
            return False
        if precision is None:
            raise ValueError("a capture fault needs the precision model")
        from .faults import CAPTURE_MISS

        before = len(precision.scripted)
        precision.scripted.append(CAPTURE_MISS)
        # The scripted draw does not read this generator. A datum ignores
        # the script; the model then seats the cell and this is not a miss.
        outcome = precision.attempt(cell, precision_rng)
        unused = (
            len(precision.scripted) == before + 1
            and precision.scripted[-1] == CAPTURE_MISS
        )
        if unused:
            precision.scripted.pop()
        if outcome.captured or unused:
            precision.commit(cell, outcome.pose)
            return False
        return True

    def on_place_done(crane_index: int, job_index: int) -> None:
        nonlocal peak
        crane = cranes[crane_index]
        job = jobs[job_index]
        if _capture_rejects(job.cell):
            faults.begin(
                "capture",
                time_s=t,
                robot=crane.index,
                role=CRANE,
                cell=job.cell,
            )
            job.stage = "queued"
            job.holder = None
            crane.free = True
            crane.job = None
            crane.pos = job.cell
            return
        if precision is not None and job.cell not in precision.poses:
            outcome = precision.attempt(job.cell, precision_rng)
            precision.commit(
                job.cell, outcome.pose if outcome.captured else outcome.offered
            )
        ok, sag = gate_ok(job.cell)
        if not ok:
            raise RuntimeError(
                f"place of {job.cell} left the caps "
                f"(sag={sag:.6g}, cap={model.finished_sag:.6g})"
            )
        if sag > peak:
            peak = sag
        job.stage = "placed"
        place_order.append(job.cell)
        crane.free = True
        crane.job = None
        crane.pos = job.cell
        if faults is not None:
            faults.finish_cell(
                job.cell,
                time_s=t,
                kinds=("capture", "drop", "cascade"),
            )
        if not job.partners:
            job.stage = "done"
            fastened.add(job.cell)
            close_finished_cell(job.cell)

    def on_fasten_done(bot_index: int, job_index: int) -> None:
        bot = fasteners[bot_index]
        job = jobs[job_index]
        job.inflight -= 1
        close_stall(bot)
        if (
            faults is not None
            and job.joints_done < len(job.partners)
            and faults.roll("fastener")
        ):
            partner = job.partners[job.joints_done]
            faults.begin(
                "fastener",
                time_s=t,
                robot=bot.index,
                role=FASTEN,
                cell=job.cell,
                joint=(job.cell, partner),
            )
            bot.free = True
            bot.job = None
            bot.pos = job.cell
            return
        partner = job.partners[job.joints_done]
        job.joints_done += 1
        bot.free = True
        bot.job = None
        bot.pos = job.cell
        log(FASTEN, "joined", job.index)
        if faults is not None:
            # Every failed attempt on this face closes when the face takes.
            for index in list(faults._open):
                event = faults.events[index]
                if event.kind != "fastener" or event.cell != job.cell:
                    continue
                if event.joint is not None and event.joint[1] != partner:
                    continue
                faults.finish(index, time_s=t)
        if job.joints_done == len(job.partners):
            job.stage = "done"
            fastened.add(job.cell)
            close_finished_cell(job.cell)
        elif job.joints_done > len(job.partners):
            raise RuntimeError("fastened a joint that does not exist")

    def on_robot_up(_role: str, _index: int) -> None:
        return

    def on_outage_done(_station_index: int) -> None:
        return

    handlers = {
        "arrive_feed": on_arrive_feed,
        "load_done": on_load_done,
        "arrive_site": on_arrive_site,
        "start_handoff": on_start_handoff,
        "handoff_done": on_handoff_done,
        "place_done": on_place_done,
        "fasten_done": on_fasten_done,
        "robot_up": on_robot_up,
        "outage_done": on_outage_done,
    }

    dispatch()
    guard = 0
    limit = max(1000, 50 * max(1, len(jobs)) * robots_per_role)
    if faults is not None:
        limit *= 20
    while heap:
        guard += 1
        if guard > limit:
            raise RuntimeError(
                f"schedule did not finish in {limit} events at t={t:.3f}"
            )
        t, _, kind, payload = heapq.heappop(heap)
        handlers[kind](*payload)
        dispatch()

    if any(not job.fastened for job in jobs):
        stages = sorted({job.stage for job in jobs if not job.fastened})
        raise RuntimeError(f"unfinished voxels in stages {stages} at t={t:.3f}")
    if fastened != set(model.cells):
        raise RuntimeError("fastened set is not the full structure")

    makespan = t
    if makespan <= 0:
        raise RuntimeError("makespan must be positive")

    def util(busy: float, count: int) -> float:
        return busy / (count * makespan)

    cargo_busy = sum(robot.busy for robot in cargos)
    crane_busy = sum(robot.busy for robot in cranes)
    fasten_busy = sum(robot.busy for robot in fasteners)
    feed_busy = sum(station.busy for station in feeds)
    utils = {
        "feed": util(feed_busy, n_feeds),
        CARGO: util(cargo_busy, robots_per_role),
        CRANE: util(crane_busy, robots_per_role),
        FASTEN: util(fasten_busy, robots_per_role),
    }
    # Highest utilization is the binding pool when that pool is actually
    # busy. Below 0.5 nobody is saturated: the remaining makespan is the
    # dependency chain (a voxel cannot be placed until its supports are
    # fastened), and naming a slack pool would overclaim a bottleneck.
    top = max(utils.values())
    if top < 0.5:
        names = ["critical-path"]
    else:
        names = [name for name, value in utils.items() if top - value <= 0.01]
    n_joints = sum(len(partners) for partners in model.joints)
    return SimResult(
        n_feeds=n_feeds,
        robots_per_role=robots_per_role,
        makespan_s=makespan,
        cargo_busy_s=cargo_busy,
        crane_busy_s=crane_busy,
        fasten_busy_s=fasten_busy,
        feed_busy_s=feed_busy,
        cargo_util=utils[CARGO],
        crane_util=utils[CRANE],
        fasten_util=utils[FASTEN],
        feed_util=utils["feed"],
        bottleneck="+".join(names),
        n_placed=len(place_order),
        n_fasten_tasks=n_joints,
        n_joints=n_joints,
        cargo_steps=cargo_steps,
        crane_steps=crane_steps,
        fasten_steps=fasten_steps,
        peak_sag=peak,
        finished_sag=model.finished_sag,
        worst_overhang=model.worst_overhang,
        place_order=tuple(place_order),
        events=tuple(events),
        n_faults=0 if faults is None else len(faults.events),
        n_cascades=0 if faults is None else faults.n_cascades,
        n_scaffold=0,
    )
