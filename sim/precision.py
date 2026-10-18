# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Per-placement position and angle error, and a self-aligning capture.

This is a geometric model of one new voxel against the cells already
placed. It is not the spring network in :mod:`sim.stability`, and it
does not take ``k``. Sag elsewhere in this repository is still the
unfitted ``k = 1``.

What was read
-------------
Gregg et al., Science Robotics 9, eadi2746 (2024),
doi:10.1126/scirobotics.adi2746, from the NTRS 20230005194 distribution
and the Science HTML. The placement requirement and the correction
envelopes below are that text. The constants live in :mod:`sim.armadas`
so this module cannot drift from the shelter write-up.

* Cargo placement is required within ±1.78 cm in X, Y, and Z and ±3°
  of the ideal pose (body text, Fig. 4A).
* Alignment skis accommodate ±1.78 cm in-plane and, fully extended,
  enforce <0.1 mm in-plane (body, Fig. 4B).
* External grippers accommodate 1.34 cm out-of-plane. Claws accommodate
  1.34 cm out-of-plane. The body says the claws enforce <0.1 mm at full
  grip. The Fig. 4 caption says <0.2 mm in Y. Both sentences are printed.
* Petals accommodate 3.0 cm in-plane. That envelope is looser than the
  placement box, so it is not the joint gate.
* Fastener actuation accommodates 0.2 cm in Y. That is after the claws
  have already pulled the face in, and it is not a second fail gate.

Gregg and Cheung, "Precision in Assembled Discrete Lattice Space
Structures for Next-Generation ISAM Applications," 2024 IEEE Aerospace
Conference, doi:10.1109/aero58975.2024.10521407, NTRS 20240000666.
Read from that PDF.

* Plate RMS surface error is the RMS distance of surface nodes to a
  best-fit plane. Beam length error is the deformed end coordinate
  minus the nominal length, divided by the nominal length. This module
  uses those two definitions.
* Their error source is a random strut length (Gaussian or uniform),
  applied as a thermal strain in ABAQUS, with rigid voxel-to-voxel
  connections. It is not a robot placement error and not a capture
  envelope. The paper says it does not treat assemblability.
* Qualitative result, from the abstract and §4: average plate RMS grows
  with plate size, and thickness collapses it toward the part error.
  Stiffness is a weak influence. Part error is a strong one.
* Numbers that are in the text, and are not rows of this model's
  campaign: Wu et al. experimental RMS 0.0055 inch and simulated 0.0043
  inch; Greene's simulation 0.0023 inches; this paper's average RMS
  0.0041 inches on that 8 m truss. Fig. 3 caption: 3 inch pitch, 0.05
  inch beam width, Gaussian strut-length STD 0.001, 33 GPa, N=10.
  Fig. 4 caption: strut-length STD 0.00212 inch, 3 inch pitch, N=10.
  The plotted inch values on those figures are not transcribed here.
* Fig. 11 text: at 12 inch pitch, increasing thickness brings average
  RMS to sub-millimeter scale. The figure marks 1 mm. Point values off
  that curve are not transcribed.

Not in either paper, and labeled as such wherever a number is reported
-----------------------------------------------------------------------
* The manuscript states a requirement box, not a histogram. The default
  distribution is independent uniform inside that box. A Gaussian whose
  variance matches the box (``half / sqrt(3)``) is available and is not
  a measured sigma.
* The ±3° figure is one angle. This model draws one rotation about a
  random axis with magnitude inside ±3°. It does not draw three Euler
  angles. A separate ski rotation limit is not printed; the angle gate
  is the placement requirement.
* "Snap" is the elastic average of the poses implied by every mating
  neighbor that passed the envelope. The papers name elastic averaging
  and do not give this average. The snap is exact. The printed <0.1 mm
  enforcement is a bound, not a sample, so it is not added as noise.
  A caller can set a residual half-width; that distribution is invented.
* The parent of a new cell is its latest face-neighbor (the growth
  front), not a world-frame metrology frame. Cells with no voxel
  neighbor are datums at the ideal pose: the fixture, or a seed.
* A miss is returned to the caller as a failed capture. The swarm and
  the single robot, when given this model, mark that cell defective and
  use the existing inspect / remove / replace path. That mapping is
  this stack's. The papers' miss is a mechanical miss, not the
  fastening-current defect flag.
* This is not an ABAQUS model. IEEE figure values in inches are not
  reproduced and are not targets.
"""

from __future__ import annotations

import math
import random
from dataclasses import dataclass, field

import numpy as np

from .armadas import (
    CLAW_ENFORCED_MM_BODY,
    CLAW_ENFORCED_MM_CAPTION,
    CLAW_OUT_OF_PLANE_CM,
    EDGE_MM,
    FASTENER_MISALIGN_CM,
    GRIPPER_OUT_OF_PLANE_CM,
    PETAL_IN_PLANE_CM,
    PLACE_TOLERANCE_CM,
    PLACE_TOLERANCE_DEG,
    SKI_ENFORCED_MM,
    SKI_IN_PLANE_CM,
)

# Cited, not a gate. Petals are looser than the placement box.
# Grippers and claws are the same 1.34 cm in the manuscript.
assert GRIPPER_OUT_OF_PLANE_CM == CLAW_OUT_OF_PLANE_CM

PITCH_M = EDGE_MM / 1000.0
_ATOL = 1e-12

DISTRIBUTIONS = ("uniform", "gaussian", "zero")


def _gauss(rng: random.Random, sigma: float) -> float:
    """Box-Muller from ``rng.random``.

    ``random.Random.gauss`` is not the sampler. Two interpreters that
    both claim Box-Muller have disagreed before; this one only depends
    on ``random()``, which this repository already treats as stable.
    """
    if sigma == 0.0:
        return 0.0
    u1 = max(rng.random(), 1e-12)
    u2 = rng.random()
    return float(sigma * math.sqrt(-2.0 * math.log(u1)) * math.cos(math.tau * u2))


def _random_axis(rng: random.Random) -> np.ndarray:
    vector = np.array([_gauss(rng, 1.0), _gauss(rng, 1.0), _gauss(rng, 1.0)])
    norm = float(np.linalg.norm(vector))
    if norm == 0.0:
        return np.array([0.0, 0.0, 1.0])
    return vector / norm


def _cell_key(cell) -> tuple[int, ...]:
    key = tuple(int(v) for v in cell)
    if len(key) not in (2, 3):
        raise ValueError(f"cell must be 2D or 3D, got {cell!r}")
    return key


def _xyz(cell) -> np.ndarray:
    """Cell indices as a length-3 float vector.

    A 2D ``(row, col)`` sits in the yz plane so a face step is still one
    axis. A 3D ``(level, row, col)`` is unchanged.
    """
    key = _cell_key(cell)
    if len(key) == 2:
        return np.array([0.0, float(key[0]), float(key[1])])
    return np.array([float(v) for v in key])


def _face_delta(child, neighbor) -> np.ndarray | None:
    """Integer child-minus-neighbor if the two cells share a face."""
    if len(_cell_key(child)) != len(_cell_key(neighbor)):
        return None
    delta = np.rint(_xyz(child) - _xyz(neighbor))
    if np.any(np.abs(_xyz(child) - _xyz(neighbor) - delta) > 1e-6):
        return None
    nonzero = np.flatnonzero(delta)
    if len(nonzero) == 1 and abs(int(delta[nonzero[0]])) == 1:
        return delta.astype(float)
    return None


def _face_axes(offset: np.ndarray) -> tuple[int, int, int, float]:
    """Normal axis, two in-plane axes (lower index first), and normal sign."""
    axis = int(np.argmax(np.abs(offset)))
    sign = 1.0 if offset[axis] > 0.0 else -1.0
    others = [i for i in range(3) if i != axis]
    return axis, others[0], others[1], sign


def _rodrigues(axis: np.ndarray, degrees: float) -> np.ndarray:
    theta = math.radians(degrees)
    if abs(theta) < 1e-15:
        return np.eye(3)
    normal = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(normal))
    if norm == 0.0:
        return np.eye(3)
    x, y, z = normal / norm
    skew = np.array(
        [[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]],
        dtype=float,
    )
    return np.eye(3) + math.sin(theta) * skew + (1.0 - math.cos(theta)) * (skew @ skew)


def _rotation_angle_deg(rotation: np.ndarray) -> float:
    cosine = (float(np.trace(rotation)) - 1.0) / 2.0
    cosine = min(1.0, max(-1.0, cosine))
    return math.degrees(math.acos(cosine))


def _quat_from_rotation(rotation: np.ndarray) -> np.ndarray:
    """Unit quaternion ``(w, x, y, z)`` from a rotation matrix."""
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (rotation[2, 1] - rotation[1, 2]) / scale
        y = (rotation[0, 2] - rotation[2, 0]) / scale
        z = (rotation[1, 0] - rotation[0, 1]) / scale
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
        w = (rotation[2, 1] - rotation[1, 2]) / scale
        x = 0.25 * scale
        y = (rotation[0, 1] + rotation[1, 0]) / scale
        z = (rotation[0, 2] + rotation[2, 0]) / scale
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
        w = (rotation[0, 2] - rotation[2, 0]) / scale
        x = (rotation[0, 1] + rotation[1, 0]) / scale
        y = 0.25 * scale
        z = (rotation[1, 2] + rotation[2, 1]) / scale
    else:
        scale = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
        w = (rotation[1, 0] - rotation[0, 1]) / scale
        x = (rotation[0, 2] + rotation[2, 0]) / scale
        y = (rotation[1, 2] + rotation[2, 1]) / scale
        z = 0.25 * scale
    quat = np.array([w, x, y, z], dtype=float)
    norm = float(np.linalg.norm(quat))
    if norm == 0.0:
        return np.array([1.0, 0.0, 0.0, 0.0])
    return quat / norm


def _rotation_from_quat(quat: np.ndarray) -> np.ndarray:
    w, x, y, z = (float(v) for v in quat)
    return np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ]
    )


@dataclass
class Pose:
    """Rigid pose of a voxel center. ``t`` is meters. ``R`` maps body to world."""

    t: np.ndarray
    R: np.ndarray

    def __post_init__(self) -> None:
        self.t = np.asarray(self.t, dtype=float).reshape(3).copy()
        self.R = np.asarray(self.R, dtype=float).reshape(3, 3).copy()


def ideal_pose(cell, pitch_m: float = PITCH_M) -> Pose:
    return Pose(_xyz(cell) * float(pitch_m), np.eye(3))


@dataclass(frozen=True)
class Draw:
    """One placement error in the parent face frame.

    ``u`` and ``v`` are in-plane, meters, on the two body axes that are
    not the face normal (lower index first). ``w`` is out-of-plane,
    meters, positive along the parent-to-child normal. ``angle_deg`` is
    one rotation about ``axis``.
    """

    u: float
    v: float
    w: float
    angle_deg: float
    axis: tuple[float, float, float]


@dataclass(frozen=True)
class PlacementOutcome:
    cell: tuple[int, ...]
    captured: bool
    pose: Pose
    offered: Pose
    datum: bool
    n_mates: int
    n_axes: int


def _offered_pose(parent: Pose, offset: np.ndarray, pitch_m: float, draw: Draw) -> Pose:
    axis, i0, i1, sign = _face_axes(offset)
    shift = np.zeros(3)
    shift[i0] = draw.u
    shift[i1] = draw.v
    shift[axis] = draw.w * sign
    rotation = parent.R @ _rodrigues(np.array(draw.axis), draw.angle_deg)
    translation = parent.t + parent.R @ (float(pitch_m) * offset + shift)
    return Pose(translation, rotation)


def _implied_pose(neighbor: Pose, offset: np.ndarray, pitch_m: float) -> Pose:
    translation = neighbor.t + neighbor.R @ (float(pitch_m) * offset)
    return Pose(translation, neighbor.R.copy())


def _average_poses(poses: list[Pose]) -> Pose:
    translation = sum((pose.t for pose in poses), start=np.zeros(3)) / len(poses)
    acc = np.zeros((4, 4))
    for pose in poses:
        quat = _quat_from_rotation(pose.R)
        acc += np.outer(quat, quat)
    _values, vectors = np.linalg.eigh(acc)
    quat = vectors[:, -1]
    if quat[0] < 0.0:
        quat = -quat
    return Pose(translation, _rotation_from_quat(quat))


def misalignment(
    child: Pose, neighbor: Pose, offset: np.ndarray, pitch_m: float
) -> tuple[float, float, float, float]:
    """``(u, v, w, angle_deg)`` of ``child`` relative to ``neighbor``."""
    relative = neighbor.R.T @ (child.t - neighbor.t)
    error = relative - float(pitch_m) * np.asarray(offset, dtype=float)
    axis, i0, i1, sign = _face_axes(offset)
    return (
        float(error[i0]),
        float(error[i1]),
        float(error[axis] * sign),
        _rotation_angle_deg(neighbor.R.T @ child.R),
    )


def _normal_axis_count(offsets: list[np.ndarray]) -> int:
    return len({int(np.argmax(np.abs(offset))) for offset in offsets})


@dataclass
class PrecisionModel:
    """Placement draw plus the ski / claw capture test.

    ``snap`` replaces a captured pose with the elastic average of the
    mating neighbors. ``check`` turns the envelope into a fail. Both
    default on. ``check`` off and ``snap`` off is open-loop propagation:
    every draw is kept, and nothing fails. ``envelope_scale`` multiplies
    the ski and claw lengths and does not multiply the ±3° angle gate
    or the placement distribution.

    ``scripted`` is a queue of ``(u, v, w, angle_deg)`` in meters and
    degrees, consumed only when a cell has a mating face. A datum does
    not pop the queue. Tests use it. The campaign does not.
    """

    distribution: str = "uniform"
    position_half_cm: float = PLACE_TOLERANCE_CM
    angle_half_deg: float = PLACE_TOLERANCE_DEG
    ski_cm: float = SKI_IN_PLANE_CM
    claw_cm: float = CLAW_OUT_OF_PLANE_CM
    pitch_m: float = PITCH_M
    sigma_position_cm: float | None = None
    sigma_angle_deg: float | None = None
    snap: bool = True
    check: bool = True
    envelope_scale: float = 1.0
    residual_in_plane_mm: float = 0.0
    residual_out_of_plane_mm: float = 0.0
    scripted: list[tuple[float, float, float, float]] | None = None
    poses: dict[tuple[int, ...], Pose] = field(default_factory=dict, repr=False)
    _when: dict[tuple[int, ...], int] = field(default_factory=dict, repr=False)
    _clock: int = 0

    def __post_init__(self) -> None:
        if self.distribution not in DISTRIBUTIONS:
            raise ValueError(
                f"distribution must be one of {DISTRIBUTIONS}, got {self.distribution!r}"
            )
        if self.position_half_cm < 0 or self.angle_half_deg < 0:
            raise ValueError("placement half-widths must be >= 0")
        if self.ski_cm < 0 or self.claw_cm < 0:
            raise ValueError("capture envelopes must be >= 0")
        if self.pitch_m <= 0:
            raise ValueError("pitch_m must be positive")
        if self.envelope_scale <= 0:
            raise ValueError("envelope_scale must be positive")
        if self.residual_in_plane_mm < 0 or self.residual_out_of_plane_mm < 0:
            raise ValueError("residual half-widths must be >= 0")
        self.scripted = list(self.scripted or [])
        self.reset()

    def reset(self) -> None:
        """Drop realized poses. The scripted queue is left in place."""
        self.poses = {}
        self._when = {}
        self._clock = 0

    @property
    def position_half_m(self) -> float:
        return self.position_half_cm / 100.0

    @property
    def in_plane_m(self) -> float:
        return self.ski_cm * self.envelope_scale / 100.0

    @property
    def out_of_plane_m(self) -> float:
        return self.claw_cm * self.envelope_scale / 100.0

    @property
    def sigma_position_m(self) -> float:
        """Gaussian sigma. Default matches the uniform box. Not measured."""
        cm = (
            self.position_half_cm / math.sqrt(3.0)
            if self.sigma_position_cm is None
            else self.sigma_position_cm
        )
        return cm / 100.0

    @property
    def sigma_angle(self) -> float:
        """Gaussian angle sigma, degrees. Default matches the uniform box."""
        if self.sigma_angle_deg is None:
            return self.angle_half_deg / math.sqrt(3.0)
        return float(self.sigma_angle_deg)

    def note_datum(self, cell) -> None:
        key = _cell_key(cell)
        self.commit(key, ideal_pose(key, self.pitch_m))

    def commit(self, cell, pose: Pose) -> None:
        key = _cell_key(cell)
        self.poses[key] = pose
        self._when[key] = self._clock
        self._clock += 1

    def forget(self, cell) -> None:
        key = _cell_key(cell)
        self.poses.pop(key, None)
        self._when.pop(key, None)

    def _mates(self, key: tuple[int, ...]) -> list[tuple[tuple[int, ...], np.ndarray]]:
        found = []
        for other in self.poses:
            delta = _face_delta(key, other)
            if delta is not None:
                found.append((other, delta))
        return found

    def _sample(self, rng: random.Random) -> Draw:
        if self.scripted:
            u, v, w, angle = self.scripted.pop(0)
            return Draw(float(u), float(v), float(w), float(angle), (0.0, 0.0, 1.0))
        if self.distribution == "zero":
            return Draw(0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 1.0))
        if self.distribution == "uniform":
            half = self.position_half_m
            return Draw(
                rng.uniform(-half, half),
                rng.uniform(-half, half),
                rng.uniform(-half, half),
                rng.uniform(-self.angle_half_deg, self.angle_half_deg),
                tuple(float(v) for v in _random_axis(rng)),
            )
        sigma = self.sigma_position_m
        return Draw(
            _gauss(rng, sigma),
            _gauss(rng, sigma),
            _gauss(rng, sigma),
            _gauss(rng, self.sigma_angle),
            tuple(float(v) for v in _random_axis(rng)),
        )

    def _within(self, u: float, v: float, w: float, angle: float) -> bool:
        return (
            abs(u) <= self.in_plane_m + _ATOL
            and abs(v) <= self.in_plane_m + _ATOL
            and abs(w) <= self.out_of_plane_m + _ATOL
            and angle <= self.angle_half_deg + 1e-9
        )

    def _add_residual(self, pose: Pose, offset: np.ndarray, rng: random.Random) -> Pose:
        if self.residual_in_plane_mm == 0.0 and self.residual_out_of_plane_mm == 0.0:
            return pose
        # Uniform inside a caller-chosen bound. Not the printed <0.1 mm
        # sentence: that sentence is a bound, and this draw is invented.
        axis, i0, i1, sign = _face_axes(offset)
        in_half = self.residual_in_plane_mm / 1000.0
        out_half = self.residual_out_of_plane_mm / 1000.0
        shift = np.zeros(3)
        shift[i0] = rng.uniform(-in_half, in_half)
        shift[i1] = rng.uniform(-in_half, in_half)
        shift[axis] = sign * rng.uniform(-out_half, out_half)
        return Pose(pose.t + pose.R @ shift, pose.R.copy())

    def attempt(self, cell, rng: random.Random) -> PlacementOutcome:
        """Draw one placement. Does not commit.

        No mating face: the cell is a datum at the ideal pose, and the
        scripted queue is not touched. Otherwise the draw is in the
        latest neighbor's face frame. With ``check`` on, every mating
        face must sit inside the ski, the claw, and the ±3° gate. With
        ``snap`` on, a capture replaces the draw with the average of the
        poses those neighbors imply.
        """
        key = _cell_key(cell)
        mates = self._mates(key)
        if not mates:
            pose = ideal_pose(key, self.pitch_m)
            return PlacementOutcome(key, True, pose, pose, True, 0, 0)
        parent_key, parent_offset = max(mates, key=lambda item: self._when[item[0]])
        draw = self._sample(rng)
        offered = _offered_pose(self.poses[parent_key], parent_offset, self.pitch_m, draw)
        n_axes = _normal_axis_count([offset for _key, offset in mates])
        if not self.check:
            return PlacementOutcome(
                key, True, offered, offered, False, len(mates), n_axes
            )
        for other, offset in mates:
            u, v, w, angle = misalignment(
                offered, self.poses[other], offset, self.pitch_m
            )
            if not self._within(u, v, w, angle):
                return PlacementOutcome(
                    key, False, offered, offered, False, len(mates), n_axes
                )
        if self.snap:
            implied = [
                _implied_pose(self.poses[other], offset, self.pitch_m)
                for other, offset in mates
            ]
            pose = self._add_residual(_average_poses(implied), parent_offset, rng)
        else:
            pose = offered
        return PlacementOutcome(key, True, pose, offered, False, len(mates), n_axes)


def armadas_precision(**kwargs) -> PrecisionModel:
    """Published envelopes, uniform inside the placement box, snap on."""
    return PrecisionModel(**kwargs)


def uniform_pass_probability(n_axes: int, model: PrecisionModel) -> float:
    """Pass probability for one uniform draw against perfect, axis-aligned mates.

    ``n_axes`` is how many distinct face normals the cell already touches.
    An axis that is someone's normal is limited by the claw. Each other
    axis is limited by the ski. The three draw components are independent
    and uniform on the placement box. The angle draw is constructed to
    lie inside the ±3° gate, so this formula does not multiply by an
    angle term. It is the wrong formula for a Gaussian, for a rotated
    parent, and for ``check=False``.
    """
    if model.distribution != "uniform":
        raise ValueError("the closed form is for the uniform placement box")
    if not model.check:
        return 1.0
    if n_axes < 0 or n_axes > 3:
        raise ValueError("n_axes must be in 0..3")
    half = model.position_half_m
    if half == 0.0:
        return 1.0
    probability = 1.0
    for _ in range(n_axes):
        probability *= min(1.0, model.out_of_plane_m / half)
    for _ in range(3 - n_axes):
        probability *= min(1.0, model.in_plane_m / half)
    return probability


def expected_uniform_fail_rate(order, datums, model: PrecisionModel) -> float:
    """Mean per-attempt miss rate if every capture snaps back to ideal.

    The walk only needs the mating normals, which do not depend on the
    draws once a snap has put each parent on its ideal pose. Open-loop
    propagation does not have this rate: a wandered parent moves the
    next test.
    """
    if not model.snap or not model.check:
        raise ValueError("the closed form assumes snap and the envelope check")
    present = {_cell_key(cell) for cell in datums}
    rates: list[float] = []
    for cell in order:
        key = _cell_key(cell)
        offsets = []
        for other in present:
            delta = _face_delta(key, other)
            if delta is not None:
                offsets.append(delta)
        if not offsets:
            raise RuntimeError(f"{key} is not face-adjacent to the prefix")
        n_axes = _normal_axis_count(offsets)
        rates.append(1.0 - uniform_pass_probability(n_axes, model))
        present.add(key)
    if not rates:
        return 0.0
    return sum(rates) / len(rates)


def translation_tip_rms(
    n_steps: int,
    half_m: float,
    *,
    claw_m: float | None = None,
    ski_m: float | None = None,
) -> float:
    """RMS of the tip's distance to its ideal center, translation only.

    ``n_steps`` independent face draws, identity rotation, chain along
    one axis. The face normal is that axis, so the claw truncates the
    along-chain component and the ski truncates the two transverse
    components. Omit ``claw_m`` and ``ski_m`` for the untruncated box:
    RMS = ``half_m * sqrt(n_steps)``. A uniform on ``[-b, b]`` has
    variance ``b**2 / 3``.
    """
    if n_steps < 0:
        raise ValueError("n_steps must be >= 0")
    if half_m < 0:
        raise ValueError("half_m must be >= 0")
    along = half_m if claw_m is None else min(half_m, claw_m)
    transverse = half_m if ski_m is None else min(half_m, ski_m)
    variance = n_steps * (2.0 * transverse**2 + along**2) / 3.0
    return math.sqrt(variance)


@dataclass
class AssemblyResult:
    poses: dict[tuple[int, ...], Pose]
    order: tuple[tuple[int, ...], ...]
    datums: tuple[tuple[int, ...], ...]
    n_attempts: int
    n_failures: int
    n_placed: int
    complete: bool

    @property
    def fail_rate(self) -> float:
        if self.n_attempts == 0:
            return 0.0
        return self.n_failures / self.n_attempts


def cantilever_chain(n: int) -> tuple[tuple[tuple[int, ...], ...], tuple[tuple[int, ...], ...]]:
    """Datum at the origin, then ``n`` voxels along +col. Size ``n``."""
    if n < 1:
        raise ValueError("n must be >= 1")
    return ((0, 0, 0),), tuple((0, 0, i) for i in range(1, n + 1))


def bridge_chain(span: int) -> tuple[tuple[tuple[int, ...], ...], tuple[tuple[int, ...], ...]]:
    """Two datum piers ``span`` columns apart, deck left to right.

    The deck is one level up, ``span + 1`` cells, including both caps.
    The last cap mates to the previous deck cell and to the far pier.
    """
    if span < 1:
        raise ValueError("span must be >= 1")
    datums = ((0, 0, 0), (0, 0, span))
    order = tuple((1, 0, i) for i in range(span + 1))
    return datums, order


def assemble(
    model: PrecisionModel,
    order,
    datums=(),
    rng: random.Random | None = None,
    max_attempts: int = 40,
) -> AssemblyResult:
    """Place ``order`` in sequence, retrying a miss up to ``max_attempts``.

    Datums are written first, at the ideal pose. A cell in ``order`` that
    does not touch the prefix raises: it is not silently promoted to a
    datum. A cell that never captures stops the walk; earlier poses stay.
    """
    if max_attempts < 1:
        raise ValueError("max_attempts must be >= 1")
    rng = rng or random.Random(0)
    model.reset()
    datum_keys = tuple(_cell_key(cell) for cell in datums)
    for key in datum_keys:
        model.note_datum(key)
    order_keys = tuple(_cell_key(cell) for cell in order)
    failures = 0
    attempts = 0
    placed = 0
    complete = True
    for key in order_keys:
        if key in model.poses:
            continue
        seated = False
        for _ in range(max_attempts):
            attempts += 1
            outcome = model.attempt(key, rng)
            if outcome.datum:
                raise RuntimeError(f"{key} has no mating face and is not a datum")
            if outcome.captured:
                model.commit(key, outcome.pose)
                seated = True
                break
            failures += 1
        if not seated:
            complete = False
            break
        placed += 1
    return AssemblyResult(
        poses=dict(model.poses),
        order=order_keys,
        datums=datum_keys,
        n_attempts=attempts,
        n_failures=failures,
        n_placed=placed,
        complete=complete,
    )


def rms_position(result: AssemblyResult, pitch_m: float, cells=None) -> float:
    """RMS of distance from each realized center to its ideal center.

    Datums are exact by construction and are left out. An incomplete
    walk scores the cells that did seat.
    """
    chosen = list(result.order if cells is None else cells)
    errors = []
    for cell in chosen:
        key = _cell_key(cell)
        pose = result.poses.get(key)
        if pose is None:
            continue
        ideal = _xyz(key) * float(pitch_m)
        errors.append(float(np.sum((pose.t - ideal) ** 2)))
    if not errors:
        return 0.0
    return math.sqrt(sum(errors) / len(errors))


def _centered(points: np.ndarray) -> np.ndarray:
    array = np.asarray(points, dtype=float).reshape(-1, 3)
    if len(array) == 0:
        return array
    return array - array.mean(axis=0)


def rms_line(points) -> float:
    """RMS distance to the best-fit line. A beam metric, not the plate one."""
    centered = _centered(np.asarray(points, dtype=float))
    if len(centered) < 2:
        return 0.0
    _u, _s, vt = np.linalg.svd(centered, full_matrices=False)
    direction = vt[0]
    along = centered @ direction
    perpendicular = centered - np.outer(along, direction)
    return float(np.sqrt(np.mean(np.sum(perpendicular**2, axis=1))))


def rms_plane(points) -> float:
    """RMS distance to the best-fit plane.

    This is the plate definition in Gregg and Cheung, IEEE Aerospace
    2024: fit a plane, then the RMS of the node-to-plane distances.
    Their nodes were strut ends from an ABAQUS strut-length model.
    These points are voxel centers from this placement model.
    """
    centered = _centered(np.asarray(points, dtype=float))
    if len(centered) < 3:
        return rms_line(centered)
    _u, _s, vt = np.linalg.svd(centered, full_matrices=False)
    normal = vt[-1]
    distance = centered @ normal
    return float(np.sqrt(np.mean(distance**2)))


def surface_points(result: AssemblyResult, cells) -> np.ndarray:
    rows = []
    for cell in cells:
        key = _cell_key(cell)
        pose = result.poses.get(key)
        if pose is None:
            continue
        rows.append(pose.t)
    if not rows:
        return np.zeros((0, 3))
    return np.vstack(rows)


def tip_length_error(
    result: AssemblyResult,
    root,
    tip,
    pitch_m: float,
    axis: int = 2,
) -> tuple[float, float, float]:
    """``(tip_error_m, length_error_m, length_error_ratio)``.

    Length error is the tip-minus-root coordinate on ``axis``, minus
    ``n * pitch``. The ratio is that difference over the nominal length.
    Gregg and Cheung report the ratio for a beam, using the average of
    the end nodes. One voxel center is the one-cell version of that
    average. It is not their ABAQUS output.
    """
    root_key = _cell_key(root)
    tip_key = _cell_key(tip)
    if root_key not in result.poses or tip_key not in result.poses:
        return (float("nan"), float("nan"), float("nan"))
    root_t = result.poses[root_key].t
    tip_t = result.poses[tip_key].t
    tip_error = float(np.linalg.norm(tip_t - _xyz(tip_key) * float(pitch_m)))
    n_steps = abs(int(round(float(_xyz(tip_key)[axis] - _xyz(root_key)[axis]))))
    nominal = n_steps * float(pitch_m)
    length_error = float(tip_t[axis] - root_t[axis] - nominal)
    ratio = length_error / nominal if nominal else 0.0
    return tip_error, length_error, ratio


# Re-exported so a reader of this module sees the cited residuals without
# opening the shelter module. They are not samples in the default model.
CITED_RESIDUAL_MM_BODY = SKI_ENFORCED_MM  # <0.1 mm, also the claw body text
CITED_CLAW_CAPTION_MM = CLAW_ENFORCED_MM_CAPTION
CITED_FASTENER_CM = FASTENER_MISALIGN_CM
CITED_PETAL_CM = PETAL_IN_PLANE_CM
CITED_CLAW_BODY_MM = CLAW_ENFORCED_MM_BODY
