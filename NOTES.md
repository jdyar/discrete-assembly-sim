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

### 2026-10-18 — joint failure: strength, utilization gate, ARMADAS face compression

Each face spring can carry a `JointStrength` (axial compression, axial tension, shear, optional moment) beside `k`. `joint_loads` is `k·Δu` from the existing solve, self-weight plus point loads. At `k = mass = g = 1` the force is `mass·g`. Pitch does not enter it. The solve writes `moment = None`. `joints_within` fails when utilization (demand/capacity) exceeds `safety_factor` (default 1, in `(0, 1]`). Planner, choreographer, and `plan_with_scaffold` take an optional `JointCap`. A finished blueprint that already fails is not rescued.

Science Robotics HTML of Gregg et al. doi:10.1126/scirobotics.adi2746, read 2026-10-18: 11.38 kPa (SD ±0.51 kPa), failure load more than 9000 N, stiffness 1.1129 MPa. NTRS 20230005194 prints 11.17 kPa (±0.77 kPa) and stiffness SD ±0.0430 MPa. Not averaged. HTML stiffness SD "±0.779 kPa" on a MPa value is unconfirmed. Gross area assumed `(3 × 0.3048 m)² = 0.83612736 m²`. Implied load `11.38 kPa × A = 9515.1293568 N` (derived, not printed). Two interfaces in series, so the section force is `F`, not `F/2`. Nine faces: `F/9 = 1057.2365952 N`, SD `47.3805504 N` from the strength SD only. Center-column failures mean equal share is an idealization. Fastener-pair 797.3 N ± 8.71 N is a fixture tension and is not the face capacity. Shear and moment unmeasured. `k` stays 1. The newton figure is not applied to the campaign rows.

`python main.py joints`. Clamp cantilevers: shear-limited (n=3 shear 3, sag 6; n=6 shear 6, sag 21). Clamp towers: compression-limited (n=3 compression 3; n=6 compression 6). Tip load P=2 on n=3: shear 3→5, sag 6→12. Bridges and the shelter (sag 48.7104, compression 10.344, tension 0.0934695, shear 3.164) fail a half-cap on compression, shear, and sag. Scaffold: crane spans 4–6 stay 0 at the finished demand and at the shear mid-gap; inchworm span 4 stays 3. Half the finished demand is infeasible. The count does not step up. Tower n_free=6 uses reach 8 so the refusal at half is the cap; reach 4 cannot touch that top. `docs/VALIDATION.md` still has every cantilever cell `PENDING`. Those prints give `k_shear`. A connector-print strength table is a second `PENDING` hook. The 1057 N figure is not in it.

Viewer: utilization ramp green–amber–red. Stills at deform 0, box 0.96. Cantilever shear row is seven boxes with 3–5 px gaps on the 1280×720 frame (widths about 130 px). Root is the high-shear end. Utilization of that row goes from the root at 1 to the tip. Tower axial is blue at the root (compression) with a gap between courses. Span-5 finished deck stays at or under 1 and sits in the air between two piers. The one-sided frame, same capacities, is past 1 at the deck root; the far pier is a ghost. Shelter seams stay visible. No box shares a cell with its neighbor. `python -m unittest`: 229 tests, OK, 226.6 s, Python 3.12.3.

### 2026-10-18 — temporary scaffold, then a stable removal

`sim/scaffold.py`. Candidates are open cells in the column under the blueprint, down to level 1. Search by increasing count; the first count that admits a place order and a removal order is minimal. Ties break on `sequence_cost` (depot round trip per task under the stride heuristic, plus pick/place/inspect or removal). That score charges a return after the last task, so it is not a proved lower bound on swarm ticks. Span-5 crane seed 2 finishes in 86 ticks against a score of 88. Caps are the finished structure: tightest `max_cantilever`, and `worst_sag` at k = mass = g = 1 (mass·g/k, not a fit). A finished blueprint that already fails is not rescued.

Rebased onto the on-lattice inchworm (`OnLatticeGeometry`, the gait in WHITEPAPER §6.7). Scaffold is a voxel that gait can grip. The campaign inchworm enters on a planted level-1 pad that is not under the deck; the crane walks the unmodified bridge. The pad did not move overhang or finished sag. Crane scaffold count stays 0, costs 71, 88, 105, 122, 139. Inchworm scaffold is 3 on span 4 (`(1,2,4)`, `(1,2,5)`, `(1,2,6)`) and 2 on spans 5–8 (span 5 `(1,2,4)` and `(1,2,6)`; spans 6 and 7 `(1,2,5)` and `(1,2,7)`; span 8 `(1,2,5)` and `(1,2,8)`). 88/90 swarms finished. Span 6 with 9 robots, seed 2, and span 8 with 4 robots, seed 2, were still going at tick 4000. The ratio is a robot count, not §6.7's mass ratio. Python 3.12.3. Rows in `docs/scaffold_campaign.csv`. Whitepaper §4.4 and §6.8.

Citation checked against the TU Delft repository record on 2026-10-18: O.G.J.I. Ozo, "Physics-Informed Planning of Discrete Lattice Assembly Sequences," Master's thesis, Delft University of Technology, Faculty of Aerospace Engineering, 2026, graduation date 15 October 2026. https://repository.tudelft.nl/record/uuid:76f63bc9-3e53-419a-b9fb-7faa376c666c and https://resolver.tudelft.nl/uuid:76f63bc9-3e53-419a-b9fb-7faa376c666c. The PDF `FinalReportGabriel-3.pdf` is embargoed until 01-10-2027. The abstract describes ACO plus FEM and a quality function that minimizes scaffold voxels. This search is exact enumeration of under-column supports. The PDF was not read.

Viewer: scaffold stays `#f5c518` in every color mode, including displacement. Box size stays 0.96. Stills re-rendered from the on-lattice span-5 run: cells `(1, 2, 4)` and `(1, 2, 6)`, one robot, 185 ticks, frame 183 still has both supports and the finished deck (8 blueprint voxels plus the 27-cell pad). Pixel check on the 1280×720 frames, excluding HUD rows above y=110 and the colorbar below y=520: the lit yellow (median 158, 126, 14) is two supports under the deck, it shares no pixels with the blue voxels, the removed frame has none of that yellow, and displacement color keeps it. The legend swatch is exact `#f5c518`. `python -m unittest`: 197 tests, OK, 175.8 s, Python 3.12.3.

### 2026-10-18 — bridge picture: true geometry, normalized exaggeration

The two-pier log was already a span (18 cells, deck at level 2, empty cell under midspan column 6, final occupancy equal to the blueprint). The viewer was multiplying the `mg/k` vector by the slider. At `k = 1` the midspan sample is 8, so 0.15×–0.50× dropped the deck by voxel-lengths into the piers and the ground. Default drawing is now the as-built cells, color only. The slider, off until moved, sets the longest arrow as a fraction of the voxel pitch (0.20 mild, cap 0.40) and the view is labeled exaggerated and normalized. Block size shrinks if a neighbor gap would close. Robots stand on the solid under their cell. `tests/test_structure_log.py` checks the air under the midspan deck and that the finished log matches the blueprint with no duplicate placements. 169 tests.

### 2026-10-18 — v0.2 milestone 6: write-up, validation plan, demo capture

README is the front page: pitch, quickstart, strain-colored bridge, what v0.2 adds, gate numbers from `docs/stability_cost.csv` with n and min–max, and the open validation. Whitepaper §3.4 holds the sag assumptions and closed forms; §4.1 the precheck; §3.3 the cuboct solid; §6.6 the campaign; §9 points at `docs/VALIDATION.md`. That file specifies the PLA cantilevers (n = 3 and n = 6 free bays, 20 mm frames, 3 mm struts, clamp block, tip tab, Bambu P2S, 0.2 mm layers, 3 walls, 0–100 g in 10 g steps) and leaves every results cell PENDING. Self-weight tip ratio 3.5 vs Euler–Bernoulli 16. `CHANGELOG.md` marks v0.2.0 unreleased; package version stays 0.1.0. `scripts/render_demo.py` is the headless viewer capture. 168 tests. The hollow-roof 7.12 s figure stays a NOTES citation: it is the pre-precheck campaign, not a row in the current CSV.

### 2026-10-18 — stability precheck: the hollow roof is infeasible at limit 0

The 3×3×2 roof center `(2, 3, 4)` has no column under it. Finished
overhang is 1, which lower-bounds every prefix, so `max_cantilever = 0`
cannot have a stable order. `necessary_stability_precheck` rejects that
before search. Limits 1 and 2 allow the bound and still plan. Campaign
re-run (Python 3.12.3): 202 feasible, 110 infeasible, 0 undecided.
Hollow limit 0 is 6/6 infeasible, median 0.096 ms (was undecided,
median 7.12 s). Build metrics other than that status are unchanged.
168 tests, with the cuboctahedral solid from milestone 4.

### 2026-10-18 — v0.2 milestone 4: optional cuboctahedral solid

`CuboctahedralLattice3D` in `sim/cuboct.py`, behind the existing
Geometry interface. Default stays `CubicLattice3D`. Same 6-neighbor
face graph, same locomotion, same stability frame, same lumped
face-spring sag (closed forms still hold). Different vertices (12,
edge midpoints) and different square-face joint sites (four corners of
the 45° square; the face center does not move). Viewer toggle in
`replay_viewer.html`; `meta.voxel_shape` is omitted for cubic logs.
Citations checked: Gregg et al., Science Robotics 9, eadi2746 (2024),
doi:10.1126/scirobotics.adi2746 — not 2018. Cheung & Gershenfeld,
Science 341, 1219–1221 (2013), doi:10.1126/science.1240889, is the
vertex-connected octahedron, which this does not implement. Jenett &
Cheung, AIAA 2017-1876, doi:10.2514/6.2017-1876, put BILL-E on that
octahedron. Not a mechanical model of ARMADAS hardware. 158 tests.

### 2026-10-18 — v0.2 milestone 3: cost of the stability gate

`experiments/stability_cost.py`, `python main.py stability`. 312 trials,
Python 3.12.3, k = 1 (uncalibrated model units). Named shapes × seeds
{0, 1, 2} × N ∈ {1, 2} × {off, 0, 1, 2, 3, none}; fuzz seeds 1001–1004
(one wall, one span-6 bridge, one cantilever, one ledge) × start seeds
{0, 1}. Search cap 200 nodes; a cap hit is undecided.

Span-5 bridge: gate-off peak sag median 15 (range 10–15, four trials at
15), finished sag 6. Limit 2, the tightest feasible setting, peaks at 6
on all 6 trials. N = 1 makespan 78–85 → 104–135; N = 2 makespan 50–56 →
69–112, kickbacks 4–26, idle robot-ticks 24–80. Limits 0 and 1 proved
infeasible, 6/6. Span-6: gate-off peak median 21 (15–21) → 10 at limit
3, finished sag 8. Cantilever 4 peaks at 15 whenever it builds, is
proved infeasible at limits 0–3, and drops from 5 singular sag samples
per gate-off trial to 0 with the connectivity rule on. Hollow 3×3×2 at
limit 0: finished structure fails the screen, order search undecided
6/6 (median 7.12 s). Successful searches stay ~0.03 s. 202 feasible,
104 infeasible, 6 undecided. Charts and CSV in docs/. 142 tests.

### 2026-10-17 — v0.2 milestone 2: strain in the run log and the viewer

Opt-in only. `StructureLog` on a `RunLog` samples `estimate_sag` when the
occupied set changes (not every tick), every `every` changes, and writes
per-voxel displacement plus a per-joint strain proxy (relative
displacement / pitch, axial vs shear) onto that frame. Default runs do
not grow the key. A singular partial build is an `error` string on the
frame, not a crashed run. `joint_strains` is the definition the log and
the tests share; the shear-chain closed form checks the proxy. The
viewer colors by displacement or by the worst incident joint strain,
draws a colorbar labeled `mg/k` unless `si=True`, exaggerates deflection
with a scale slider, and cages `worst_sag_cell`. Old logs still load.
`python main.py bridge` is a two-pier deck with the gate on
(`max_cantilever=3`) and the sampler on at the default stiffness of 1.
The picture is not a calibrated measurement. 135 tests.

### 2026-10-17 — v0.2 milestone 1: structural stability gate (off by default)

`sim/stability.py`. `is_stable` requires every voxel to face-connect to
a column that bears on the base, and limits horizontal face-steps from
that column (`max_cantilever`). `estimate_sag` is a separate
small-displacement spring network (voxels = nodes, face joints = axial
and shear springs, numpy `Ku = f`) and reports worst-case displacement.
It is not inside the search. The planner, the validator, and the
choreographer take an optional `StabilityPolicy` and otherwise behave
exactly as before; a refused step uses the existing kickback path.
Deeds are not structural supports. The "42 codebases" / "legally
unusable planner" lines in the README and whitepaper are now flagged as
author notes: the survey list was never committed. Package version
stays 0.1.0 because the default build is unchanged. 124 tests.

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
