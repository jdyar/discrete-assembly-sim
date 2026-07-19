# NOTES — engineering log

Working notes for this simulator: design decisions with their rationale,
per-slice session history, and the data contracts the code must honor.
(Slices 0–2 and the multi-robot choreographer complete; 3D multi-robot
choreography is the active slice — see README.)

## North star

An open toolchain for discrete lattice assembly: **blueprint → validated
build sequence → robot choreography → verification/error-correction**.
Reference architecture: NASA Ames ARMADAS (cuboctahedral voxels, SOLL-E
inchworm builders, autonomous meter-scale builds, 2024). Where this sim's
model diverges from the published hardware approach, the divergence is
flagged in the relevant decision below.

## Architecture decision — Slice 2 defect model (decided 2026-07-05)

Picks: **bad-bond-in-place** defects (probability p per placement, voxel
crawlable but must be replaced) · **inspect-after-each-place** (1 tick,
from the placement stance) · **remove + discard** (1 tick, scrap leaves
the world). Replan integration: robot removes locally, then halts with
`needs_replan`; the executor calls the planner on the current world
(0 ticks — planning is offboard software, mirroring the ARMADAS split
between robot firmware and planning stack). Baseline mode
(`inspect_enabled=False`) places blind.

References:
- ARMADAS (Science Robotics 2024): the system builds and error-corrects
  with no machine vision — detection comes from fastening-time feedback
  (bolting servo current/status), which inspect-after-place models
  directly. Placement and fastening are separate steps in the hardware;
  our defect = "placed but badly bonded".
- Petersen et al., "Errors in Collective Robotic Construction": error
  taxonomy distinguishes fatal vs non-fatal failures; misplacement (their
  dominant mode) is deferred here as a possible second defect channel.

Flagged divergences from ARMADAS: (1) defective voxels assumed crawlable;
(2) scrap discarded in place rather than returned to the depot
(config-flag candidate once multi-robot congestion matters).

## Architecture decision — movement regime (decided 2026-07-05)

**Adopted: BILL-E surface locomotion** (MIT BILL-E → MILAbot lineage;
same robot class as ARMADAS's SOLL-E builders — inchworm robots walking
on the lattice they build).

- Grip rule: robot occupies any empty cell 4-adjacent to GROUND/VOXEL
  (tops, faces, undersides). Moves orthogonally along surfaces; diagonal
  only when rounding a corner (exactly one of the two between-cells solid).
- Reach: places into any of the 8 surrounding cells.

Why: a previously considered step-climber regime (ground + climbing one
step at a time, front/down-front placement) was PROVEN infeasible for the
6x4 wall by exhaustive search — even a 1x2 tower fails (no stance for the
2nd voxel; depot round trips force the depot-side column to stay ≤1 tall
while needing to reach height 4). That regime matches the known TERMES
setting (Werfel/Petersen/Nagpal, Science 2014), where the published fix is
ramped blueprints + compiler-verified buildability — kept as a possible
future comparison study. Under BILL-E rules the wall builds bottom-up with
ZERO floating placements (gravity-safe for a future 3D slice) and the
robot descends the face at the end (never stranded). Machine-verified
before adoption; the footing/reach rule functions are injectable in the
planner so rule variants can be evaluated without touching the robot.

## Architecture decision — 3D-first pivot (decided 2026-07-15)

2D multi-robot coordination was measured degenerate and dropped, not
fixed. Evidence: (a) a 2-robot 2D wall build took exactly the ticks of a
1-robot build (406 = 406) — the second robot starved with 117 task
kickbacks and zero placements, because a 2D wall build is a single-file
corridor with the depot at one end (the known MAPF 1-wide-corridor
pathology, Stern et al. arXiv:1906.08291); (b) a pencil-and-paper
two-depot variant forces non-crossing robots into staircase-shaped
intermediate structures that grow beyond the blueprint footprint —
worse than one robot. No practical discrete-assembly system targets 2D
structures. The 2D sim stays as the regression base; the active build is
3D (cubic voxel grid — see docs/DESIGN.md, lattice decisions).

Same decision set (2026-07-16): reach articulation for radius > 1 is
**snake arm** (empty-cell path within the ball, never through solids);
the cubic voxel grid is confirmed over strut-level cuboct geometry.

## Session log

### 2026-07-19 (later) — motion config complete + ship experiments
(1) MotionModel gains stride and climb; stride is reservation-sound:
Geometry.move_footprint exposes a multi-cell move's swept intermediates,
the time-expanded graph refuses moves whose footprint isn't free across
the transition window, and the choreographer leases footprints with
each leg. Coupled robots modeled as one logical robot with a combined
arm chain (MotionModel.coupled) — coupling is swarm-setup config;
dynamic rendezvous is future work. (2) The swarm-scale yield experiment
found a real repair race: one robot's post-repair resequence hit the
planner's dirty-world refusal while another robot's defect was still
standing. Fixed with a deferred resequence (bounded window — every
standing defect's detector already holds its REMOVE step); regression
test pins the interleaving. (3) Results: corrected yield 100% at every
p x N; uncorrected swarms JAM (standing defects poison buildability
proofs) far below the (1-p) line. Knee-vs-size: x4.07 at N=5 on a
96-voxel box vs the N~4 knee on 40 voxels — the knee is a property of
the structure, not the stack. 98 tests green.

### 2026-07-19 — 3D at scale: perf 28x, fuzz, repair, snake-arm reach
(1) Profiled the 2-robot 3D build at 512s wall — 94% in the
buildability gate re-running the build-order search per placement.
Two semantics-preserving fixes (identical tick-for-tick run):
one depot-reachable-set BFS per world state instead of one BFS per
candidate stance, and a gate memo keyed on (occupancy, projected
solids, remaining). 512s -> 18s. (2) Snake-arm reach implemented:
radius > 1 is a BFS whose intermediate cells must be empty — around
corners, never through solids; symmetric at every radius; radius 1
reduces exactly to the 26-shell. (3) Repair-in-a-crowd-3D green
(defect mid-swarm, repair through live reservations, liveness proven)
+ a fuzz harness (randomized boxes x hollow x robots x defects x
reach; 40-case campaign clean). (4) Scaling to N=4 exposed a real
bug: an aborting robot re-pinned its cell with a STRICT hold and
crashed when another robot's lease already crossed that cell — now a
best-effort hold + displacement flag, consistent with the rest of the
loop. (5) Charts: speedup N=1..5 (x2.81, knee at N~4) and ticks vs
reach radius (x1.75 at reach 4, coordination unchanged). 88 tests.

### 2026-07-15 — 3D slice: world + geometry + traps + first real speedup
3D trap fixtures authored spec-first (tomb trap, tunnel standoff,
backfill trap — each grounded in a published failure class, reasoning in
docstrings), then World3D + CubicLattice3D + MotionModel(reach_radius)
behind the unchanged geometry interface. The coordination stack —
sequencer, choreographer, reservation table, time-expanded A* — ran on
3D nodes with ZERO logic changes; the spec-first fixtures caught exactly
two latent 2D-factory defaults (a default geometry_factory in
Sequencer.resequence and in the buildability gate — the latter's crash
was silently swallowed by its defect-guard except clause, refusing every
3D placement; Geometry.bound_factory() is the fix). Demo: hollow 4x4x3
box (40 voxels): 1 robot 472 ticks, 2 robots 278 (x1.70), 3 robots 211
(x2.24) — versus x1.00 in 2D with the same coordination code
(docs/speedup_3d.png). Zero collisions, zero deadlocks; run-log contract
v3 (3D occupancy + levels meta); the Three.js viewer renders v1/v2/v3.
82 tests green (2D suite untouched).

### 2026-07-14 — multi-robot choreographer (2D) + trap fixtures
Cooperative A* over a time-expanded graph against a shared reservation
table (lease = movement, deed = placement — an implementation note
within the published ARMADAS reservation framework, see
docs/DESIGN.md, Provenance). Placement gates: connectivity (no
placement may disconnect any robot's committed future from the depot,
proven on the permanent future graph) and buildability (the remaining
blueprint must stay completable). Sequencer<->choreographer are coupled
(kickback + receding horizon), not pipelined. Adversarial fixtures
authored spec-first before the swarm existed: pocket trap, corridor
standoff, repair-in-a-crowd — all green, plus the fixture-revision
history (what each trap caught) recorded in tests/test_traps.py.

### 2026-07-13 — geometry seam + reservation structures
Geometry interface extracted (neighbors/is_footing/reach_cells +
heuristic hooks); the 2D BILL-E rules moved verbatim behind it; all
prior tests unchanged. Clean-room reservation table (leases with
swap-safe undirected edges, deeds permanent from commit tick,
release_owner as the replan primitive) + time-expanded graph. A
string-node triangle-lattice fake in the tests proves no 2D assumptions.

### 2026-07-10 — packaging pass (public release)
Packaging only, no feature changes: MIT LICENSE + per-file header notices;
README (replay GIF, yield-vs-p chart, quickstart, architecture, background
& references); docs/ with committed charts + replay.gif (runs/ stays
gitignored). Fresh-clone experience verified in an isolated clone + clean
venv: `pip install -r requirements.txt`, 38 tests OK, all four `main.py`
modes run. Corrected stale slice-1 numbers (current planner: 402 ticks /
16.8 ticks-per-voxel). Design was also grounded in a survey of 42 related
open research codebases (summarized in the README's background section).

### 2026-07-05 — Slice 2 complete (error correction, thesis demo)
World gains DEFECT state; robot gains INSPECT (1 tick) + REMOVE (1 tick) +
needs_replan halt; executor replans via plan_build_order (planner rejects
dirty worlds). Demo run p=0.12 seed=7: 7 defects detected+repaired, final
yield 24/24, 527 ticks (runs/latest.json — watch the red voxel appear,
get inspected, and vanish). Experiment: p in [0, 0.15] x 10 seeds x
{corrected, baseline}: corrected yield 100% everywhere (spec >=99% at
p=0.08: PASS); baseline decays ~(1-p) to 83% at p=0.15. Chart:
runs/yield_vs_p.png. 38 tests green.

### 2026-07-05 — Slice 1 complete (full wall)
plan_build_order (DFS + memoized dead ends, returns None only on exhaustive
proof, raises SearchBudgetExceeded otherwise) + validate_plan (independent
re-simulation: reach, grip, depot round trips, strand check). Robot executes
planner Tasks (cell + exact approach). Run at the time: 24/24 voxels, 378
ticks, 281 moves, 0 idle, 15.8 ticks/voxel (later planner tweaks: 402 /
16.8). First metrics chart: runs/ticks_per_voxel.png (upper courses cost
more — longer crawls). 30 tests green.

### 2026-07-04 — Slice 1 support tooling
Per-tick JSON run logging (`RunLog` in `sim/metrics.py`), Three.js replay
viewer (`replay_viewer.html`: drop in a run log; play/pause/scrub/orbit),
matplotlib animation fallback (`python -m sim.render runs/latest.json
[out.gif]`). `python main.py` writes `runs/latest.json` (gitignored).

### 2026-07-04 — Slice 0 done
The ugly loop works end-to-end: 7x10 numpy grid with a ground row, hardcoded
6x4 wall blueprint, one blueprint cell filled per tick (bottom-up so nothing
floats), ASCII frame per tick. Completes in 24 ticks, 6 tests green.

- `sim/world.py` — occupancy grid (EMPTY/GROUND/VOXEL) + blueprint mask.
- `sim/render.py` — ASCII: `#` voxel, `o` pending blueprint, `=` ground, `.` air.
- `sim/metrics.py` — RunLog with per-tick records (feeds the charts).
- `sim/parts.py` — placeholder until the typed-parts slice.

Run: `python main.py` · Test: `python -m unittest`

## Architecture decision — planner vs robot (decided 2026-07-04)

The planner computes the FULL build order upfront; the robot executes it
blindly as a queue.

Why not greedy (robot picks nearest unbuilt cell):
- The no-stranding constraint is global — whether a placement is safe
  depends on all future placements. Greedy has no lookahead and can wall
  off unbuilt regions or strand itself.
- Upfront plans are deterministic → same blueprint, same plan, every run
  → testable.
- Keeps the robot maximally dumb/local per spec. A robot choosing targets
  is planning smuggled into the robot.

Known limitation (deliberate): a fixed plan breaks when placements can
fail. The solution is NOT greedy — it's plan / execute / replan-on-failure
(receding horizon): robot reports the defect, planner re-derives the queue
from current world state. This is exactly what Slice 2 implemented.

## Interface contract (designed for replanning from day one)

- `planner.plan(world_state, blueprint) -> ordered queue of placement tasks`
- robot consumes the queue; on failure/defect, halts and requests a replan
  with current world state
- robot never mutates the plan

## Run-log data contract (v1) — what robot.py/planner.py must emit

The sim loop calls `RunLog.log_tick(tick, world, robot)` once per tick
(tick 0 = initial state). Full spec in `sim/metrics.py`; consumers are
`replay_viewer.html` and `sim.render.animate_run` — same JSON feeds both.

Robot instances must expose (duck-typed; missing attrs raise at log time):
- `pos` — `(row, col)`, numpy convention, ground = bottom row
- `state` — current state-machine label, any `str()`-able value
  (shown verbatim in the viewer)
- `carrying` — truthy iff holding a voxel

Planner: nothing logged per tick. Tasks carry `cell` + `approach` as
`(row, col)`; to record the plan for debugging, stash JSON-able extras in
`RunLog.meta` (e.g. `meta["plan"] = [...]`).

JSON shape:
```json
{"version": 1,
 "meta": {"rows": 7, "cols": 10, "legend": {"EMPTY": 0, "GROUND": 1, "VOXEL": 2}},
 "blueprint": [[row, col], ...],
 "ticks": [{"tick": 0, "occupancy": [[...]], "placed": 0,
            "robot": null | {"pos": [row, col], "state": "MOVING", "carrying": true}}]}
```

## Slice 1 invariants (tested)

- Every placement cell is adjacent to a cell the robot can legally stand
  on at placement time
- Robot path exists (ground + built voxels only) from its position to each
  approach cell
- No placement ever disconnects the robot from the depot
- Full 6x4 wall completes
