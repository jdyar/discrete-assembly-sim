# DESIGN.md — architecture rules & provenance

The binding design rules this codebase is built around, and where each
piece of the coordination stack comes from. Code docstrings cite this
file; this file cites the published literature.

## The two invariants (the point of the project)

**1. The coordination logic never assumes a lattice.**
The world model exposes a *geometry interface* — `neighbors(node)`,
`is_footing(node)`, `reach_cells(node)` (`sim/geometry.py`) — as
pluggable per-lattice implementations. The choreographer, planner,
sequencer, and reservation table treat nodes as opaque hashable values:
no coordinate arithmetic, no hardcoded adjacency counts, no
row/column assumptions outside the geometry implementation itself.
This is enforced by test (`tests/test_reservations.py` runs the stack on
a string-node triangle-lattice fake) and demonstrated by the 3D pivot:
`CubicLattice3D` (`sim/geometry3d.py`) replaced the 2D square lattice
and the coordination stack ran **unchanged**.

**2. The motion model is a parameter, not an assumption.**
Robot capabilities — step distance, climb limit, placement reach —
live in `MotionModel` (`sim/geometry3d.py`) behind the geometry
interface, never in the choreographer. Real lattice robots are not
one-cell steppers: NASA's SOLL-E takes multi-voxel inchworm strides,
and BILL-E-class robots place parts several cells away and can couple
into longer arms (Jenett & Cheung, NTRS 20170006219; US patent
10,046,820). Those capabilities are *config* on this stack. The
coordination logic (reservations, lease/deed, entrapment gates) is
invariant to the motion model — that invariance is the claim, and it is
what lets a lab with different hardware reuse the stack by editing a
config, not the planner.

Decided reach semantics (radius > 1, lands with the motion-realism
milestone): **snake arm** — a target is reachable iff an empty-cell
path of length ≤ `reach_radius` exists from stance to target (the arm
articulates around corners and over ledges, never through solid volume).

## Coupled, not pipelined

The sequencer (build order) and choreographer (robot itineraries) form
a receding-horizon loop: the reachability-aware sequencer proposes, the
choreographer's gates dispose, and refusals are *kicked back* for
reordering rather than failing the run. A build order is never handed
over as a fait accompli.

Two gates run before any placement is committed:

- **Connectivity gate** — the placement must not disconnect any robot's
  committed future position from the depot, evaluated on the *permanent
  future graph* (world solids + all deeded-but-unplaced voxels).
- **Buildability gate** — the placement must leave the remaining
  blueprint completable (re-proved by the sequencer's search on the
  projected future world). Catches out-of-order placements that would
  seal a pocket of unbuilt cells.

A third gate, **structural stability**, is off unless the caller passes
a `StabilityPolicy` (`sim/stability.py`). It refuses a placement whose
partial build is not face-connected to a column bearing on the base, or
whose horizontal overhang exceeds `max_cantilever`. A refusal is the
same kickback. The coordinate arithmetic stays in `sim/stability.py`;
the planner and the choreographer only see the policy the lattice (or
the caller) built, which is what keeps the "no lattice assumptions"
rule intact. The gate counts voxels already in the world plus the
proposal. It does not count an unplaced deed as a support: a
reservation does not carry load. Connectivity still counts deeds,
because a cell that will be solid can trap a robot.

The sag solve is a dense `Ku = mg` (voxels as nodes, face joints as
axial and shear springs). It reports the worst displacement of a
partial build. It becomes a gate only when the caller passes a
`SagCap`: `sag_within` then runs on every prefix inside the search.
A `JointCap`, when one is passed, is a second gate on the same
prefixes (`joints_within` in `sim/stability.py`). Utilization is
demand/capacity, and the allowable is capacity times a safety factor
in `(0, 1]`. A component left unset is not checked. The force is
`k` times relative displacement, so at the defaults it is `mass·g`,
not a newton. The derived ARMADAS face compression stays in newtons
and is not that cap. Both caps are off unless passed. The moment
slot exists; the spring network has no rotational degree of freedom,
so a solved load carries no moment.

Default-off is load-bearing: `max_cantilever=0` rejects spans, including
a hollow-box roof. The planner applies that screen to the finished
blueprint before it searches (`stability_precheck`): a voxel's overhang
there is a lower bound on every prefix, so a roof cell with no column
and a bound past the limit is infeasible without expanding the order
tree. The check does not reject a blueprint whose finished structure
passes. §6.1–§6.5 of the whitepaper were not run with the gate on.
§6.6 is the measurement (`python main.py stability`), and the
stiffness in that run is the unfitted default k = 1.

When the finished blueprint passes those caps and no reachable order
does, `plan_with_scaffold` (`sim/scaffold.py`) adds the smallest set of
temporary supports in the column under the blueprint, builds with them,
and removes them in an order that stays inside the same caps.
When a `JointCap` is passed, every prefix and every removal also stays
inside it (§6.10 of the whitepaper, `python main.py joints`). Scaffold
is solid, so a robot can walk on it, and it is not a blueprint cell.
The search minimizes the count, then a single-robot travel score.
The swarm tick count is a separate measurement (`python main.py
scaffold`). That campaign's inchworm is `OnLatticeGeometry` with the
on-lattice gait, entering from a planted level-1 pad that is not under
the deck. The crane walks the unmodified bridge. A finished structure that already fails the caps is not
rescued. Sag in that cap is `worst_sag` at k = mass = g = 1, units of
mass·g/k, stiffness not fit. This is not the ant-colony / FEM scaffold
search in the Ozo thesis (WHITEPAPER §8); that PDF is embargoed.

The same estimate can be written into the run log. A `StructureLog`
attached to a `RunLog` is off by default; without it, frames are
unchanged. When it is set, each sampled occupancy change — a placement
or a removal, every `every` such changes — stores that tick's per-voxel
displacement, a per-joint strain proxy (relative displacement over
pitch, axial and shear separate), and the spring force. When a
strength is set, the sample also stores utilization. The replay
viewer colors from those samples, including utilization, and still
opens logs that omit them. Blocks stay on
the cells where they were placed unless the exaggerate slider is
moved; that slider draws the longest arrow as a fraction of the
voxel pitch and labels the picture exaggerated and normalized.
The numbers are the model. A stiffness fit to printed specimens, and a
displacement limit inside the gate, are still open.

## Provenance (clean-room rule)

The multi-robot choreography is implemented clean-room from published
papers only:

- **Cooperative A\* over a time-expanded graph, node reservations,
  storage-location deadlock-freedom**: the ARMADAS system paper, Gregg
  et al., *Science Robotics* 2024 (NTRS 20230005194), and its planning
  supplementary (NTRS 20250002467).
- **Lease (movement) vs deed (placement) semantics** are this project's
  implementation note *within* that published reservation framework —
  documented as such, not claimed as novel.
- **Trap fixtures** are grounded in published failure classes: TERMES
  intermediate-configuration deadlocks and single-path-additive interior
  constraints (Werfel, Petersen, Nagpal, *Science* 2014; Petersen et
  al., "Errors in Collective Robotic Construction"), and the MAPF
  1-wide-corridor swap pathology (Stern et al., "Multi-Agent
  Pathfinding: Definitions, Variants, and Benchmarks",
  arXiv:1906.08291).
- Unlicensed academic code in this space (notably pb-construction) was
  run only as a black-box benchmark and never read while writing this
  code. MIT-licensed repos may be read and are attributed where used.

## World & lattice decisions

- **3D is a simple cubic voxel grid** (`sim/world3d.py`). Cuboctahedral
  voxels, when a caller asks for them
  (`CuboctahedralLattice3D` in `sim/cuboct.py`), are assembled *in that
  same array*: centers on the integer lattice, six face-neighbors,
  the same surface locomotion as `CubicLattice3D`. What changes is the
  solid (12 vertices at the edge midpoints of the bounding cube, not 8
  corners) and the in-plane joint sites (four corners of the shared
  square face, rotated 45° from the cube's face corners). The face
  center, where `estimate_sag` anchors one lumped axial/shear spring,
  is the same point. The cubic lattice remains the default. This is a
  geometry and joint-location model of the ARMADAS face-joined voxel
  (Gregg et al., *Science Robotics* 2024, doi:10.1126/scirobotics.adi2746),
  not a validated stiffness of that hardware, and not the 2013
  vertex-connected octahedron (Cheung & Gershenfeld, *Science*). A
  strut-level or fastener-resolved geometry would be another Geometry.
- **Surface locomotion** generalizes the machine-verified 2D rules per
  axis-plane: footing = empty cell face-adjacent to a solid; movement =
  orthogonal steps plus in-plane corner rounding (exactly one of the
  two between-cells solid — a corner to pivot around, not a gap to
  cross); reach = Chebyshev ball of `reach_radius`. This is the
  default. The robot stands in an empty cell and reaches out.
- **On-lattice inchworm** (`sim/inchworm.py`,
  `OnLatticeGeometry`) is opt-in. Both feet grip exposed voxel faces,
  exactly one foot swings per step, and a carried voxel is placed
  into an empty cell that shares a face with the structure and is
  within `place_reach` of a foot. The cell outside a gripped face is
  not a placement: that would bury the foot. Top faces are the
  default surface. Side faces, inverted faces, stride, span, and
  place reach are configuration. BILL-E (AIAA 2017-1876) and SOLL-E
  (Park et al., IROS 2023, NTRS 20230011353; the transport robot in
  Gregg et al. 2024) are the hardware the gait is abstracted from.
  The cubic-face edges are not a replay of either joint trajectory,
  and BILL-E walks a different lattice (vertex-connected octahedra).
  A stance reserves both faces, so two robots that share a foot
  conflict in the same reservation table and cooperative A*. Robot
  weight is a point load at the foot voxels
  (`split_foot_loads`), off unless a caller sets a mass.

## Test discipline

Adversarial trap fixtures are authored **spec-first** — written and
committed before the implementation they test exists, from the
published failure classes above — so test and implementation stay
separated in time. Every fixture's docstring states what trap it sets
and why a naive coordinator falls into it. They double as a benchmark
suite: if you have your own planner, point it at `tests/test_traps.py`
and `tests/test_traps3d.py`.
