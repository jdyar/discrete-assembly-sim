# NOTES

## Next concrete step
Repo-digest round 1 is DONE (42/44 digested; see RESEARCH/repos/00-SUMMARY.md
+ LEADS.md). Research op continues: synthesize **40-gaps.md** from the 10
deep-research reports + the digest round. The digest round's input to it:
no open codebase covers blueprint->sequence->choreography->verification->
error-correction; choreography and error correction are empty everywhere
(error correction exists only as 2014 theory + two disabled/broken attempts);
the one buyable component is conmech (MIT stiffness checker); the codebase
to beat is paul-richard/discrete-robotic-assembly-scf (MIT, 2025).
Optional pre-step: round-2 clones from LEADS.md (pb-construction, choreo,
amiraa/swarm-morphing, Langford GIK repos).

**For Joshua**: gitlab.cba.mit.edu now refuses anonymous clones
("HTTP Basic: Access denied") — falcone/structural_robotics was never
fetched. Check auth/visibility before round 2 (several round-2 leads are
also on CBA GitLab). Slices 3-5 remain PARKED pending 40-gaps.md.

## Architecture decision — Slice 2 defect model (decided 2026-07-05)
Picks (Joshua): **bad-bond-in-place** defects (probability p per placement,
voxel crawlable but must be replaced) · **inspect-after-each-place**
(1 tick, from the placement stance) · **remove + discard** (1 tick, scrap
leaves the world). Replan integration: robot removes locally, then halts
with needs_replan; executor calls the planner on the current world (0 ticks
— offboard software). Baseline mode (inspect_enabled=False) places blind.

KB citations:
- ARMADAS (Science Robotics 2024, via landscape doc + NTRS paper): system
  "builds and error-corrects on its own with no machine vision" — detection
  comes from fastening-time feedback (MMIC-I bolting, servo current/status),
  which inspect-after-place models directly. Placement vs fastening are
  separate steps in hardware; our defect = "placed but badly bonded".
- TERMES errors paper (Petersen et al., "Errors in Collective Robotic
  Construction"): error taxonomy distinguishes fatal vs non-fatal failures;
  misplacement (their dominant mode) deferred as a future second channel.

Flagged divergences from ARMADAS: (1) defective voxels assumed crawlable;
(2) scrap discarded in place rather than returned (config-flag candidate
for Slice 3 congestion realism).

## North star (updated 2026-07-05, per project revamp)
The sim targets the software gap named in `research/10-landscape-v0.md`
(strategic read #3): an open **blueprint -> validated build sequence ->
robot choreography -> verification** toolchain. Reference architecture:
**NASA Ames ARMADAS** (cuboctahedral voxels, SOLL-E inchworm builders,
MMIC-I fastener, autonomous meter-scale builds 2024). Where our spec
diverges from their published approach, flag to Joshua as a decision.
Design decisions cite the research KB here in NOTES.md.

## Architecture decision — movement regime (decided 2026-07-05)
**Adopted: BILL-E surface locomotion** (project's own reference lineage,
MIT BILL-E -> MILAbot; matches 2024-26 bleeding edge at MIT CBA).
KB citation (added post-hoc, revamp day): aligns with the reference
architecture — ARMADAS SOLL-E builders are exactly this class of inchworm
robot walking on the lattice they build (`research/10-landscape-v0.md`,
Lineage map / NASA Ames ARMADAS [S]).
- Grip rule: robot occupies any empty cell 4-adjacent to GROUND/VOXEL
  (tops, faces). Moves orthogonally along surfaces; diagonal only when
  rounding a corner (exactly one of the two between-cells solid).
- Reach: places into any of the 8 surrounding cells.

Why: the previously confirmed step-climber + front/down-front rules were
PROVEN infeasible for the 6x4 wall by exhaustive search (even a 1x2 tower
fails — no stance for the 2nd voxel; depot round trips mean the depot-side
column must stay <=1 tall yet must reach 4). This is the known TERMES regime
(Werfel/Petersen/Nagpal, Science 2014) where the fix is ramped blueprints +
compiler-verified buildability — kept as a possible future comparison study.
Under BILL-E rules the wall builds bottom-up with ZERO floating placements
(gravity-safe for Slice 5) and the robot descends the face at the end (never
stranded). Machine-verified before adoption; rule fns injectable in planner.

## Session log

### 2026-07-10 — packaging pass (public-ready repo, pending README approval)
Packaging only, no feature/scope changes: MIT LICENSE + one-line header
notice in every source file (py + replay_viewer.html); README.md drafted
(hero replay GIF, yield-vs-p chart + thesis paragraph, quickstart,
architecture, Background & References grounded in 10-landscape-v0.md,
roadmap, license rationale); docs/ added with committed copies of the
charts + docs/replay.gif (runs/ stays gitignored). Fresh-clone experience
verified in an isolated clone + clean venv: pip install -r requirements.txt,
38 tests OK, all four main.py modes run, render smoke OK. Corrected stale
slice-1 numbers (current planner: 402 ticks / 16.8 ticks-per-voxel).
NOT committed — README draft awaits Joshua's review per instruction.

### 2026-07-08/10 — repo-digest round 1 (research op)
Cloned 44 target repos (CBA GitLab via API enumeration + GitHub sweep per
RESEARCH/30-repo-digest-brief.md); digested 42 with one deep-reading agent
each (5 priority-tier with build-vs-buy pipeline verdicts: metavoxels-code,
DMDesign, pychoreo, coop_assembly, ASAP). Outputs: 42 digests in
RESEARCH/repos/, ranked roll-up 00-SUMMARY.md, LEADS.md (round-2 clone
list, people map, paper list, verify list). Headline: the pipeline gap is
real — build fresh, buy only conmech; scf (MIT 2025) is benchmark #1;
Slice 2's inspect/remove/replan loop has no working competitor in 42 repos
(milabot-v0's FIX/undo shipped disabled; Manufacturing_Errors has the
closed-form yield theory to cite). Not digested: falcone/structural_robotics
(CBA GitLab auth), yijiangh/choreo (clone never landed — round 2).

### 2026-07-05 — Slice 2 complete (error correction, thesis demo)
World gains DEFECT state; robot gains INSPECT (1 tick) + REMOVE (1 tick) +
needs_replan halt; executor replans via plan_build_order (planner rejects
dirty worlds). Demo run p=0.12 seed=7: 7 defects detected+repaired, final
yield 24/24, 527 ticks (runs/latest.json — watch the red voxel appear,
get inspected, and vanish). Experiment: p in [0, 0.15] x 10 seeds x
{corrected, baseline}: corrected yield 100% everywhere (spec >=99% at
p=0.08: PASS); baseline decays ~(1-p) to 83% at p=0.15. Chart:
runs/yield_vs_p.png (the publishable one). 38 tests green.

### 2026-07-05 — Slice 1 complete (full wall)
plan_build_order (DFS + memoized dead ends, returns None only on exhaustive
proof, raises SearchBudgetExceeded otherwise) + validate_plan (independent
re-simulation: reach, grip, depot round trips, strand check). Robot executes
planner Tasks (cell + exact approach). Run: 24/24 voxels, 378 ticks, 281
moves, 0 idle, 15.8 ticks/voxel. First metrics chart:
runs/ticks_per_voxel.png (upper courses cost more — longer crawls).
30 tests green.

### 2026-07-04 — Slice 1 support tooling (Claude)
Per-tick JSON run logging (`RunLog` in `sim/metrics.py`), Three.js replay
viewer (`replay_viewer.html`: drop in a run log; play/pause/scrub/orbit),
matplotlib animation fallback (`python -m sim.render runs/latest.json
[out.gif]`). `python main.py` now writes `runs/latest.json` (gitignored).
robot.py/planner.py untouched except contract docs in their stub docstrings.

### 2026-07-04 — Slice 0 done
The ugly loop works end-to-end: 7x10 numpy grid with a ground row, hardcoded
6x4 wall blueprint, one blueprint cell filled per tick (bottom-up so nothing
floats), ASCII frame per tick. Completes in 24 ticks, 6 tests green.

- `sim/world.py` — occupancy grid (EMPTY/GROUND/VOXEL) + blueprint mask.
- `sim/render.py` — ASCII: `#` voxel, `o` pending blueprint, `=` ground, `.` air.
- `sim/metrics.py` — RunLog with per-tick records (feeds future charts).
- `sim/robot.py`, `sim/planner.py` — docstring stubs, reserved for Joshua.
- `sim/parts.py` — placeholder until Slice 4.
- Installed numpy + matplotlib into `.venv` (matplotlib unused until Slice 1).

Run: `python main.py` · Test: `python -m unittest`

## Architecture decision — planner vs robot (decided 2026-07-04)
The planner computes the FULL build order upfront; the robot executes it blindly as a queue.

Why not greedy (robot picks nearest unbuilt cell):
- The no-stranding constraint is global — whether a placement is safe depends on all
  future placements. Greedy has no lookahead and can wall off unbuilt regions or strand itself.
- Upfront plans are deterministic → same blueprint, same plan, every run → testable.
- Keeps the robot maximally dumb/local per spec. A robot choosing targets is planning
  smuggled into the robot.

Known limitation (deliberate): a fixed plan breaks in Slice 2 when placements can fail.
Solution then is NOT greedy — it's plan / execute / replan-on-failure (receding horizon):
robot reports defect, planner re-derives the queue from current world state.

## Interface contract (design for replanning now)
- planner.plan(world_state, blueprint) -> ordered queue of placement tasks
- robot consumes queue; on failure/defect, halts and requests replan with current world state
- robot never mutates the plan

## Run-log data contract (v1) — what robot.py/planner.py must emit
The sim loop calls `RunLog.log_tick(tick, world, robot)` once per tick
(tick 0 = initial state). Full spec in `sim/metrics.py`; consumers are
`replay_viewer.html` and `sim.render.animate_run` — same JSON feeds both.

Robot instances must expose (duck-typed; missing attrs raise at log time):
- `pos` — `(row, col)`, numpy convention, ground = bottom row
- `state` — current state-machine label, any `str()`-able value
  (shown verbatim in the viewer, e.g. "FETCHING", "MOVING", "PLACING")
- `carrying` — truthy iff holding a voxel

Planner: nothing logged per tick. Tasks should carry `cell` +
`approach_cell` as `(row, col)`; to record the plan for debugging, stash
JSON-able extras in `RunLog.meta` (e.g. `meta["plan"] = [...]`).

JSON shape:
```json
{"version": 1,
 "meta": {"rows": 7, "cols": 10, "legend": {"EMPTY": 0, "GROUND": 1, "VOXEL": 2}},
 "blueprint": [[row, col], ...],
 "ticks": [{"tick": 0, "occupancy": [[...]], "placed": 0,
            "robot": null | {"pos": [row, col], "state": "MOVING", "carrying": true}}]}
```

## Slice 1 invariants to test
- Every placement cell is adjacent to a cell the robot can legally stand on at placement time
- Robot path exists (ground + built voxels only) from its position to each approach cell
- No placement ever disconnects the robot from the depot
- Full 6x4 wall completes; then the arch