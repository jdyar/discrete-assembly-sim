# An Open Coordination Stack for Discrete Lattice Assembly: Motion-Invariant Multi-Robot Choreography with In-Situ Error Correction

**Joshua Dyar** · July 2026 · [github.com/jdyar/discrete-assembly-sim](https://github.com/jdyar/discrete-assembly-sim) · MIT License

---

## Abstract

Robots that assemble large structures from discrete lattice blocks exist: NASA's ARMADAS has demonstrated autonomous meter-scale builds, and the BILL-E and TERMES lineages established the underlying locomotion and construction models. The *coordination software* — the layer that turns a blueprint into a validated build order, choreographs multiple robots through shared space, and repairs defective placements mid-build — has no open, reusable implementation. We present one. The stack contributes: (1) a coordination core (cooperative A\* over a time-expanded graph, a lease/deed reservation table, and two placement gates that provably prevent robot entrapment and unbuildable states) that is **invariant to lattice geometry and robot motion capability**, both of which are injected as configuration; (2) an in-situ error-correction loop that detects, removes, and replaces defective placements *through live reservations*, without halting the swarm; (3) an adversarial trap-fixture suite, grounded in published failure classes, usable as a benchmark by other planners; and (4) a replayable run-log format with viewer and chart tooling. We report supporting experiments: a measured negative result for 2D multi-robot assembly (zero speedup; single-file corridor topology); 3D speedup of ×2.81 (5 robots, 40-voxel structure) rising to ×4.07 on a 96-voxel structure, locating the congestion knee as a property of structure size rather than of the stack; placement reach as pure configuration (×1.75 build-time improvement from reach 1→4 with zero coordination changes); and swarm-scale yield experiments showing corrected yield of 100% across all tested defect rates and robot counts, while *uncorrected* swarms jam far below the classical (1−p) decay — standing defects poison build-order feasibility proofs, making error correction a liveness requirement at swarm scale, not merely a yield optimization. All results regenerate from a pure-Python repository with a 98-test suite.

---

## 1. The software gap

Discrete ("digital") lattice assembly builds large structures from small families of reversibly joined blocks. The approach descends from digital materials [Cheung & Gershenfeld 2013] and is embodied in hardware today: relative robots that walk on the lattice they build — MIT's BILL-E [Jenett et al.], Harvard's TERMES [Werfel, Petersen & Nagpal 2014], and most completely NASA Ames' ARMADAS, whose SOLL-E robots autonomously assembled a meters-scale shelter from cuboctahedral voxels [Gregg et al., *Science Robotics* 2024]. The hardware thesis is proven.

The software that coordinates such robots is where the field is thin. In mid-2026 we surveyed 44 public research codebases across the MIT CBA, NASA-adjacent, TERMES, and assembly-planning literatures (42 digested in depth). Three findings motivated this work:

1. **No open end-to-end toolchain.** Nothing public and maintained covers blueprint → validated build sequence → multi-robot choreography → verification/error-correction. Fragments exist; the pipeline does not.
2. **The best academic planner is legally unusable.** The most complete multi-robot construction-planning codebase we found (pb-construction, RSS 2020 lineage) carries **no license** — all rights reserved. It cannot be built upon.
3. **Mid-build repair does not exist as working code.** Across 42 codebases, error correction during construction appears only as closed-form theory or as shipped-disabled fragments. No surveyed system detects, removes, and replaces a defective placement while other robots keep working.

Consequently every lab with new lattice-robot hardware re-solves coordination privately. This paper and its repository are an attempt to make that unnecessary: an MIT-licensed reference implementation whose coordination core is deliberately *independent of the robot* — labs plug in their own lattice and kinematics as configuration.

**Contributions.** C1: a motion- and lattice-invariant coordination stack (§3–§4). C2: placement gating that makes entrapment and unbuildable states unreachable, coupled to sequencing by kickback (§4). C3: repair through live reservations at swarm scale (§4, §6.4). C4: an adversarial trap suite as a reusable benchmark (§5). C5: an open interchange format and tooling (§7). We claim engineering synthesis and openness, not algorithmic novelty: the planning core follows ARMADAS's published method (clean-room; §8), and our reservation vocabulary is an implementation note within that framework.

## 2. Problem setting

**World.** A bounded cubic voxel grid. Nodes are `(level, row, col)`; level 0 is a solid ground plane. Each cell is empty, ground, a placed voxel, or a *defective* voxel. A blueprint marks the cells that must hold good voxels. (A 2D square-lattice world with identical interfaces serves as regression base and as the setting for the negative result of §6.1.)

**Robots.** Surface-locomoting relative robots: a robot occupies one empty cell adjacent to structure, moves one *tick-move* at a time along the surface (orthogonal steps plus corner rounding, generalized per axis-plane from the machine-verified 2D rule set), carries one block, picks at a depot cell, and places into reachable cells. Robots sense only adjacent cells plus their task; global knowledge is limited to the blueprint and depot location. Motion capability beyond this baseline — reach radius, stride, climb, coupled pairs — is configuration (§3.2).

**Defect model.** Following ARMADAS's vision-free error detection (fastening-time feedback rather than machine vision), each placement is defective with probability *p*: the block occupies its cell and is crawlable, but does not count toward completion and is detectable only by a one-tick INSPECT from an adjacent stance. Repair = remove (one tick), discard, and re-sequence.

**Required properties.** Throughout every run: (S1) no two robots co-located; (S2) no robot ever entrapped — a path to the depot exists on the current world for every robot at every tick; (S3) the remaining blueprint stays completable; (L1) under defects, the swarm makes progress during repair (no global freeze). These are asserted per-tick by every test in the suite, including fuzz.

## 3. Architecture: coordination invariant to lattice and motion

The design bet of the stack is that **coordination logic should never know what lattice it runs on or what the robot's body can do.** Two seams enforce this.

### 3.1 The Geometry seam

A `Geometry` implementation answers five questions: `neighbors(node)` (legal tick-moves), `is_footing(node)` (may a robot occupy this node), `reach_cells(node)` (cells placeable/removable from this stance), `move_footprint(a, b)` (cells swept by a multi-cell move, §3.2), and `future_view(solids)` (the lattice as if given cells were already built — the substrate for the gates of §4.2). Nodes are opaque hashable values. The sequencer, choreographer, reservation table, and time-expanded graph perform no coordinate arithmetic; the test suite includes a string-node triangle-lattice fake to keep them honest.

The invariance claim is tested, not asserted: the stack was developed against a 2D square lattice, and the 3D cubic lattice was added **without changing a line of coordination logic**. (Two latent *defaults* — call sites that fell back to the 2D geometry factory — were exposed immediately by the spec-first 3D fixtures and fixed by giving Geometry a `bound_factory()`; we report this because it is the honest shape of such claims: the architecture held, the defaults did not, and the fixtures caught them.)

### 3.2 The MotionModel seam

Real lattice robots are not one-cell steppers. SOLL-E takes multi-voxel inchworm strides; BILL-E-class arms place blocks several cells away, articulate around corners, and can couple into longer chains [Jenett; NTRS 20170006219; US 10,046,820]. `MotionModel` expresses these as data:

- **`reach_radius`** with **snake-arm articulation**: a target is reachable from a stance iff a path of Chebyshev steps of length ≤ radius connects them whose *intermediate* cells are all empty — the arm bends around corners and over ledges, never through solid volume. Endpoints are exempt (the stance is the robot's own cell; a solid target is how inspect/remove address a placed block), which keeps the relation symmetric — the property the planner's stance inversion requires — at every radius. Radius 1 reduces exactly to the 26-cell shell.
- **`stride`**: a tick-move chains up to `stride` surface steps. Soundness is the interesting part: a stride move *sweeps through* cells it does not end on. `move_footprint` reports the swept cells; the time-expanded graph refuses moves whose footprint is not free across the whole transition window (both ticks — conservative, so a robot merely *departing* a swept cell still conflicts); the choreographer leases footprints along with each leg. This was the single interface extension that motion realism forced on the coordination layer — and it is the correct one to force: the coordination stack learns only that "a move may occupy more than its endpoints," never *why*.
- **`climb`**: maximum |level change| per tick-move. 0 yields a ground-bound gantry robot; 1 the TERMES-class climber; larger values pair with stride.
- **Coupled robots**: `MotionModel.coupled(other)` models two rigidly attached robots as *one* logical robot whose arm chain spans both — reach is the sum of radii; stride and climb take the pair's minimum. Coupling is thus a swarm-setup choice (spawn N pairs instead of 2N solos) requiring zero coordination changes. Dynamic mid-run rendezvous, attachment, and detachment require a pairing protocol in the task layer and are explicitly future work (§9).

A\*'s admissible heuristic scales with stride (⌈Chebyshev/stride⌉), and everything above is exercised at swarm scale in the test suite: stride-2 pairs threading live traffic, a coupled pair completing a build, crossing-safety through swept cells.

## 4. The coupled sequencer–choreographer loop

### 4.1 Sequencing as proof

`plan_build_order` searches placement orders depth-first with memoized dead ends, accepting only orders in which every placement is reachable from a legal stance at placement time, with a depot round trip between placements, and no state from which the robot or any unbuilt cell is cut off. Its return contract is unusual and deliberate: `None` only on *exhaustive* proof that no valid order exists; exceeding the search budget *raises* (`SearchBudgetExceeded`) — feasibility undecided is never silently conflated with infeasible. An independent validator re-simulates every accepted plan against the same rules (defense in depth).

### 4.2 Choreography under reservations

Robots plan one at a time (cooperative A\*, following ARMADAS's published time-expanded-pose-graph method) against a shared **reservation table** distinguishing two kinds of hold — our implementation note within their framework:

- **Lease** — movement: a robot holds a node (and undirected edge, so head-on swaps conflict) for exactly one tick; leases are released wholesale when a robot replans.
- **Deed** — placement: from its commit tick, a deeded node is solid *forever*: never enterable, surviving replans, retractable only by explicit defect removal.

A robot's full task — current position → depot, PICK, depot → stance, PLACE (deed), INSPECT plus a spare tick that becomes REMOVE on a bad bond — is planned and reserved atomically, per-task (receding horizon), so repair replanning is the same code path as normal operation.

Before any placement commits, two gates run on the **permanent future graph** (world solids ∪ all deeds ∪ the proposal, projected through `Geometry.future_view`):

- **Connectivity gate**: the placement must not disconnect any robot's committed end-position from the depot. This is the anti-entrapment guarantee, and in 3D it is sharper than intuition suggests: a one-cell room entombs its occupant when the *fourth wall* goes up, not the roof — the cell above the cavity has no footing until the roof itself exists (§5, tomb trap).
- **Buildability gate**: on the projected world, the sequencer's search must still find a complete order for the remaining blueprint. Robot-connectivity alone is insufficient under concurrent claiming — out-of-order placements can seal a pocket of *unbuilt* cells, a failure mode surfaced by the repair-in-a-crowd fixture.

A refused placement is not a failure: the task is **kicked back** to the sequencer for reordering and the robot moves on. The sequencer and choreographer are thus coupled in a receding-horizon loop — a build order is never a fait accompli, and the gate is the final authority on every placement.

### 4.3 Liveness machinery, each mechanism traced to the trap that forced it

Deadlock-freedom in practice came from a short list of mechanisms, none speculative — each exists because a named fixture failed without it: assigned **storage locations** (ARMADAS's discipline) so idle robots never squat in cavities or corridors; **displacement waves** (parked robots are static obstacles to others' searches; failed plans push idle robots toward storage, farthest-from-depot first); **claim cooldowns** (without them, the nearest robot re-claims the same unplannable cell forever and starves the robot that could do it — a livelock found by repair-in-a-crowd); best-effort holds with a `must_move` flag for robots whose cell an inbound lease crosses; and a **watchdog** that re-derives the build order if all robots idle with work pending, raising a hard error rather than spinning if that cannot help.

Two defects in this machinery were found *by scaling and by experiment* after all authored fixtures passed — we report them as evidence the methodology works, not as embarrassments. (1) At N=4, an aborting robot re-pinned its cell with a *strict* hold and crashed when another robot's lease already crossed that cell within the horizon; the fix (best-effort hold + displacement flag) matches the rest of the loop, and a regression test pins the interleaving. (2) The swarm-scale yield experiment exposed a repair race: robot A's post-repair re-sequence hit the planner's dirty-world refusal while robot B's defective block still stood, B's remove being one tick behind. The single-robot contract "remove before replanning" does not survive concurrency; the fix defers the re-sequence to the first clean tick — a bounded window, since every standing defect's detector already holds its scheduled REMOVE.

## 5. An adversarial benchmark: the trap suite

All coordination fixtures are **authored spec-first** — written and committed against the published failure classes and the API contract *before* the implementation they test exists — preserving test/implementation separation in time. Each fixture's docstring states the trap and why a naive coordinator falls into it; per-tick invariants (S1–S3) are asserted throughout every run.

**2D:** *pocket trap* (the blueprint's last block caps a cavity on the natural crossing route — placeable only when nobody is inside), *corridor standoff* (one crossing cell, two robots, opposite directions, overlapping windows — resolvable only by timing), *repair-in-a-crowd* (forced defect among three robots; repair must thread live reservations while others provably progress).

**3D:** *tomb trap* (four walls + roof around a non-blueprint cavity; sealing happens on the fourth wall — the connectivity gate must refuse placements whose permanent future strands anyone, yet the build must complete), *tunnel standoff* (a ceiling-high slab pierced by one tunnel cell; the MAPF 1-wide-corridor swap pathology [Stern et al. 2019] with no over-the-top route), *backfill trap* (a roofed dead-end channel that must be filled deepest-first by a robot standing inside it, backing out ahead of each placement — the TERMES single-path-additive interior constraint as choreography: the second robot must neither enter nor seal the mouth).

Beyond the authored six, a **fuzz harness** drives the full loop over randomized structures (solid/hollow boxes and walls), robot counts, defect rates, reach radii, and strides, with fixed seeds for determinism in CI and open-ended campaigns off it (a 40-case campaign runs clean). Failures print their scenario tuple for direct promotion into named fixtures.

The suite is usable as a benchmark by any planner that can consume the world/geometry contract: the fixtures encode the failure classes; passing them is a meaningful claim.

## 6. Results

All numbers regenerate from repository entry points (`python main.py <mode>`); charts referenced below are committed under `docs/`.

### 6.1 A negative result: 2D multi-robot assembly is topologically degenerate

Our first 2-robot demonstration was on the 2D wall from the single-robot slices. The measured outcome: **406 ticks for one robot, 406 for two.** The second robot starved — 117 kickbacks, zero placements. The cause is not an implementation defect but topology: early in a 2D wall build, the only walkable ground is a single-file corridor with the depot at one end; one robot working anywhere in it is a wall between the other robot and the depot — the known 1-wide-corridor pathology from the MAPF literature. Hand analysis of a two-depot variant fares no better: non-crossing robots force staircase-shaped intermediate structures growing beyond the blueprint footprint. Since no practical system targets 2D structures, we dropped 2D multi-robot work rather than optimizing it; the 2D simulator remains the regression base. We report this because the contrast is the cleanest evidence for the rest of §6: everything that follows changed *only the lattice*.

### 6.2 3D speedup and the congestion knee (`docs/speedup_3d.png`, `docs/knee_vs_size.png`)

On a hollow 4×4×3 box (40 voxels), the identical coordination stack delivers real parallelism: 472/278/211/176/168 ticks for N = 1…5 robots — ×1.70, ×2.24, ×2.68, ×2.81 — with zero collisions or deadlocks across all runs and the roof correctly sequenced last. Marginal speedup collapses to ×1.05 at N=5: the **congestion knee** at N≈4, where stance and depot contention binds.

The knee is a property of the structure, not the stack: on a 96-voxel hollow box the same swarm reaches **×4.07 at N=5** (1775/953/665/543/436 ticks) with marginal speedup still ×1.25 — no knee within tested range. Larger buildable surface admits more simultaneous non-conflicting work.

### 6.3 Motion capability as configuration (`docs/ticks_vs_reach.png`)

Re-running the 40-voxel build (2 robots) at snake-arm reach radii 1–4: 278/213/191/159 ticks — **×1.75 from configuration alone**, through an unchanged planner, gates, and reservation table. Kickbacks are non-monotonic across radii (3/0/4/11): extended reach trades travel for stance contention — robots work from fewer, better cells and sometimes want the same ones. Stride and coupled-pair configurations complete the same builds through the same code paths (suite-verified); we report reach as the headline because it is the capability most visible in current hardware.

### 6.4 Yield at swarm scale: correction is a liveness requirement (`docs/yield3d.png`)

Single-robot results reproduce the classical picture (2D: corrected yield 100% at the p = 0.08 spec point; blind baseline decaying toward ~83% at p = 0.15). At swarm scale the picture sharpens. Corrected: **100% yield at every tested point** (p ∈ {0, 0.04, 0.08, 0.12} × N ∈ {1, 2, 3} × 3 seeds), with repairs threaded through live reservations and — per the liveness assertions — other robots demonstrably progressing during repair windows.

The uncorrected baseline does **not** follow the (1−p) per-placement decay a single blind builder shows. It collapses far below it: 31% at p = 0.04, 15% at p = 0.08. The mechanism: a standing defective block occupies its blueprint cell, so the buildability gate — correctly — cannot prove the remaining order completable near it, and placements in the defect's neighborhood are refused; the coordinated build *jams* rather than degrades. We initially expected the (1−p) curve and consider the measured result the more important finding: **in a gated, safety-proving swarm, error correction is not a yield optimization but the mechanism that keeps the build alive.** (The (1−p) line is retained on the chart as the reference bound; closed-form yield models in this lineage trace to Cellucci & Cheung.)

### 6.5 Engineering scale

The exhaustive-search core is fast enough for the demonstrated regime after two semantics-preserving optimizations (one reachable-set BFS per world state replacing ~93k per-candidate searches; memoized buildability verdicts keyed on world state) — 512 s → 18 s wall-clock for the 2-robot 40-voxel build, byte-identical simulation. We state the limit honestly: exhaustive sequencing will not hold at 10³–10⁴ voxels; hierarchical decomposition is future work, and the gate/kickback architecture is agnostic to the sequencer behind it.

## 7. Interchange format and tooling

Every run emits one JSON log (versioned contract, documented in-source): world dimensions, legend, blueprint, and per-tick occupancy plus per-robot position/state/carry. Three versions are live (v1 single-robot 2D; v2 multi-robot; v3 3D), all consumed by the same zero-build Three.js replay viewer (drop a log in; play/scrub/orbit) and by the chart pipeline. The log is the integration point for higher-fidelity validation: a community-requested USD/Isaac Sim exporter (listed in CONTRIBUTING) would replay itineraries against articulated robot models — fidelity layered *above* the discrete source of truth, which is where relative-robot systems locate their ground truth (the lattice is the metrology).

## 8. Related work

**ARMADAS** [Gregg et al., *Science Robotics* 2024; planning supplementary NTRS 20250002467] is the closest published system and our provenance anchor: cooperative/multi-label A\* over time-expanded pose graphs with node reservations and storage-location deadlock-freedom. Our stack is a clean-room implementation from the published descriptions (no ARMADAS code exists to consult); lease/deed is our vocabulary for machinery their framework implies. **TERMES** [Werfel et al. 2014] compiles structures into local rules — search-free coordination purchased by restricting buildable structures to single-path additive orders; our gates + search remove that restriction at the cost of computation, and TERMES's documented failure classes ground two of our fixtures. **MAPF**: cooperative A\* descends from Silver's prioritized planning; optimal solvers (CBS/ECBS, e.g. libMultiRobotPlanning) are natural comparison baselines but do not natively address a *changing world* whose placements alter the graph, nor placement gating; our corridor fixtures instantiate Stern et al.'s benchmark pathologies. **Assembly planning**: the pb-construction lineage (sequence-and-motion planning for architectural assembly) is the strongest prior code and is unlicensed — the availability gap §1 documents. **Foundations**: digital materials [Cheung & Gershenfeld 2013]; Winfree's Tile Assembly Model as the theory backbone; the collective-robotic-construction review [Petersen et al. 2019] for the field map; Petersen et al.'s error taxonomy and Cellucci & Cheung's yield analysis for the defect lineage.

## 9. Limitations and future work

No physics: gravity, loads, and structural checks on partial builds are absent (the cubic abstraction is coordination-level; the cuboct voxel's mechanics are parts-level future work). Prioritized planning is incomplete relative to joint optimal planning — kickback plus displacement resolved every scenario in suite and fuzz, but we have not characterized where they cannot. Coupling is static configuration; dynamic rendezvous/attach/detach needs a task-layer protocol. Sequencing is exhaustive and will require hierarchy beyond ~10³ voxels. Single depot; multi-depot logistics untouched. And all results are simulation — hardware-in-the-loop validation via the run-log exporter path is the intended next step for external labs.

## 10. Reproducibility

Pure Python + numpy + matplotlib; no game engine, no ROS, no build step. `python -m unittest` runs the 98-test suite (traps, fuzz, invariants) in under a minute; each figure regenerates from a single `main.py` mode; the viewer is one HTML file. MIT-licensed deliberately: the field's strongest prior code is unlicensed, and a reference implementation must be legally buildable-upon to function as one.

---

*Acknowledgments: implementation developed with Claude (Anthropic) as coding agent under the author's direction; all design decisions, experimental directions, and this document's claims were reviewed and directed by the author.*

*Correspondence: issues and design discussion via the repository's GitHub tracker.*
