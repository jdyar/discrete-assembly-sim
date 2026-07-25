# Examples: plug your system into the coordination stack

These are the two integration paths, each a single runnable file. Both
run through the **unchanged** coordination stack — no forks, no patches.

```bash
python examples/hex_lattice.py   # bring your own LATTICE
python examples/my_robot.py      # bring your own ROBOT
```

Both are exercised by the test suite (`tests/test_examples.py`), so they
cannot silently rot.

## Path 1 — your lattice (`hex_lattice.py`)

The core library ships square-2D and cubic-3D lattices. This example
implements a **hexagonal lattice** — which exists nowhere in the core —
in ~80 lines, then runs the stock build-order planner, validator, and
two-robot swarm on it. What you implement:

1. **A `Geometry`** (`sim/geometry.py`): four required methods —
   `neighbors` (legal one-tick moves), `is_footing` (may a robot occupy
   this node), `reach_cells` (placeable cells from a stance; must be
   symmetric), `future_view` (your lattice as if given cells were
   built) — plus two optional performance hooks (`heuristic_distance`,
   `candidate_sort_key`) that default to correct-but-slower.
2. **A world** satisfying the `WorldLike` protocol (documented in
   `sim/geometry.py`). If your cells fit a dense array under any
   coordinate convention — the hex example uses "odd-r" offset
   coordinates — **reuse the stock `World`/`World3D` unchanged** and
   skip this step entirely.

Nodes are opaque to the stack (any hashable value), so nothing stops a
strut-graph or non-periodic lattice: back it with a dict-based world
that satisfies `WorldLike`, and the same planner and choreographer run.

## Path 2 — your robot (`my_robot.py`)

If the cubic lattice fits but your robot's *capabilities* differ, you
don't even implement a Geometry — capabilities are `MotionModel` fields
(`sim/geometry3d.py`):

| Field | Meaning | Hardware analogue |
|---|---|---|
| `reach_radius` | snake-arm placement reach (articulates around corners, never through solids) | BILL-E-class arm span |
| `stride` | surface cells per tick-move (swept cells are reservation-protected) | SOLL-E inchworm gait |
| `climb` | max level change per tick-move (0 = ground-bound) | TERMES-class climber = 1 |
| `.coupled(other)` | two attached robots as one logical robot, summed reach | BILL-E cooperative manipulation |

The example builds one blueprint under four hardware configurations and
prints the tick counts — the "what does my hardware buy me" experiment,
ready for your numbers.

## When the interface can't express your system

That is the most valuable bug report this project can receive. Open an
issue describing the capability that wouldn't fit — interface gaps are
treated as defects, per [CONTRIBUTING.md](../CONTRIBUTING.md).
