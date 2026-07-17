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

- **3D is a simple cubic voxel grid** (`sim/world3d.py`), faithful to
  ARMADAS at the choreography level: cuboctahedral voxels are assembled
  *in a cubic array* — voxel centers form a cubic lattice and robots
  step voxel-to-voxel on the structure exterior. The cuboct shape
  matters for parts/physics (future work), not for coordination. A
  strut-level geometry, if ever needed, is just another Geometry
  implementation.
- **Surface locomotion** generalizes the machine-verified 2D rules per
  axis-plane: footing = empty cell face-adjacent to a solid; movement =
  orthogonal steps plus in-plane corner rounding (exactly one of the
  two between-cells solid — a corner to pivot around, not a gap to
  cross); reach = Chebyshev ball of `reach_radius`.

## Test discipline

Adversarial trap fixtures are authored **spec-first** — written and
committed before the implementation they test exists, from the
published failure classes above — so test and implementation stay
separated in time. Every fixture's docstring states what trap it sets
and why a naive coordinator falls into it. They double as a benchmark
suite: if you have your own planner, point it at `tests/test_traps.py`
and `tests/test_traps3d.py`.
