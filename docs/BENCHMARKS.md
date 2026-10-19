# Public benchmark suite

Twenty-one scenarios in [`benchmarks/tasks/`](../benchmarks/tasks/). The index is [`benchmarks/suite.json`](../benchmarks/suite.json). Each file is a `scenario` document of format `discrete-assembly` 1.0.0, described in [FORMAT.md](FORMAT.md).

The suite is the geometries this repository already builds: the 2D wall, the 3D wall, the hollow box, the cuboct block, pier bridges of span 4 through 8, cantilevers, the closed-form arm and tower, the on-lattice bridge, the 256-cell shelter stand-in, crane and inchworm scaffold cases, and two seeded fault tasks. `python -m sim.scenarios` regenerates the JSON from those constructors. The committed files are what a lab reads. Importing the simulator is not required to read a task.

```bash
python main.py bench                 # all 21 -> benchmarks/results.csv, benchmarks/results.md
python main.py bench smoke           # nine tasks -> benchmarks/results-partial.csv
python main.py bench bridge-span-4 tower
```

`results-partial.csv` is a local subset. It is not part of the suite. The full table is committed.

Every row is scored at `k = mass = g = 1` with `fitted` false. `k = 1` is not a fit. Sag is in units of `mass·g/k`. Joint utilization is demand divided by the finished structure's own peak component demand at that `k`. It is not a measured connector strength, and it is not the derived ARMADAS face compression of 1057.2365952 N. Fault rates are configuration. `assert_unfitted_suite` refuses a public row whose model says otherwise.

The runner calls the stock planner only: build-order search, the scaffold search, the three-role shelter schedule, and the 2D wall robot. A lab submits a planner by reading a scenario and writing an event log. This command does not load a second planner.

## Metrics

Columns in `benchmarks/results.csv`:

| column | definition |
| --- | --- |
| `status` | `complete` when the blueprint is finished, no scaffold remains, and the swarm (when there is one) reports done. `infeasible` when the search or the precheck rejects the blueprint. `incomplete` when the tick cap is hit with cells still empty. `undecided` when the node budget is exceeded. |
| `completion` | good voxels on structural and seed cells, divided by that count. A defect does not count. Scaffold does not count. |
| `makespan` | swarm ticks, or shelter seconds. The markdown table prints shelter time in hours. |
| `peak_sag` | peak voxel displacement during the build, robot weight off, `StructureLog` every step, `k = mass = g = 1`. Blank on the 2D wall: the sag model is the 3D spring network. Units `mass*g/k`. |
| `joint_utilization` | peak of demand / the finished structure's own peak component demand (compression, tension, shear; components at or below 1e-9 are omitted). The finished structure's worst joint is 1. A worse prefix can exceed 1. |
| `scaffold_count` | cells the search inserted, counted even after they are removed. Zero when the plain plan already works. |
| `faults` | fault events other than cascades. |
| `recoveries` | recovery events. |
| `cascades` | faults that open while a recovery is already open. |
| `unrecovered` | `faults + cascades − recoveries`, floored at 0. A good voxel closes every open fault on that cell. |
| `recovery_time` | median clock from an open fault to its recovery. Blank when there is no recovery. |
| `gate_accepts`, `gate_refusals` | gate events of those verdicts. |
| `k`, `fitted` | `1` and `false` on every public row. |
| `n_target` | structural cells plus seed cells. |
| `n_structural` | structural cells only. |

## Tasks

Motion on the stability-campaign bridges, walls, hollow box, cantilevers, and tower is the stepper: reach 1, stride 1, climb 1, one robot, gate `finished` (caps taken from the finished structure), scaffold `none`, `max_ticks` 8000. The crane used by the scaffold tasks is reach 4, climb 0. The inchworm gait is the on-lattice configuration in `experiments/on_lattice.py` (stride 1, span 2, place reach 1, top and sides, not inverted). The fault tower uses reach 2, climb 1, and a `JointCap` of axial compression 10 and shear 10, above self-weight so the cap does not bind. Only that task passes a joint cap.

| id | family | what it is |
| --- | --- | --- |
| `wall-2d` | wall | 6×4 square wall, 24 structural cells, no defects. |
| `wall-3d` | wall | cubic wall, gate at the finished caps. |
| `hollow-box` | hollow | 3×3×2 box. The roof is legal at the finished cantilever. |
| `hollow-roof-limit-0` | hollow | the same box at cantilever limit 0. The precheck rejects it. |
| `cuboct-block` | cuboct | 2×2×2 cuboctahedra on the cubic grid. Gate off. |
| `bridge-span-4` … `bridge-span-8` | bridge | pier column gap of that span. Stepper, one robot. |
| `cantilever-3`, `cantilever-4` | cantilever | free lengths 3 and 4 from the stability campaign. |
| `cantilever-arm` | cantilever | 3-voxel arm. Tip sag closed form `n(n+1)/2 = 6`. |
| `tower` | tower | axial stack. Tip shortening closed form 6, same `n`. |
| `onlattice-bridge` | bridge | cell robot, gate off. The inchworm on a deck is the scaffold task. |
| `armadas-shelter` | shelter | 256-cell stand-in, five-voxel seed, one feed, one robot per role. |
| `scaffold-span-4-crane` | scaffold | span 4, crane, minimal-scaffold search. |
| `scaffold-span-4-inchworm` | scaffold | span 4, inchworm, entry pad stored as seed. |
| `scaffold-span-8-crane` | scaffold | span 8, crane. |
| `fault-wall` | fault | the 2D wall at `defect_p` 0.12, `rng_seed` 7. |
| `fault-tower` | fault | two voxels, scripted fastener `["fastener", "fastener"]`. |

`n_target` on the arm and the tower counts seed. `n_structural` is the cells still to place (3 on each of those two). The inchworm span-4 target is 34 because the planted pad is seed; structural cells are the 7 of the span-4 deck.

## Stock results

`python main.py bench`, `k = mass = g = 1`. Source: [`benchmarks/results.csv`](../benchmarks/results.csv). Sag is `mass·g/k`. The shelter makespan is hours (154700 s).

| id | status | completion | makespan | peak sag | joint util. | scaffold | faults | recoveries |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| wall-2d | complete | 1 | 402 tick |  |  | 0 | 0 | 0 |
| wall-3d | complete | 1 | 115 tick | 3 | 1 | 0 | 0 | 0 |
| hollow-box | complete | 1 | 159 tick | 1.4 | 1 | 0 | 0 | 0 |
| hollow-roof-limit-0 | infeasible | 0 | 0 tick |  |  | 0 | 0 | 0 |
| cuboct-block | complete | 1 | 76 tick | 1 | 1 | 0 | 0 | 0 |
| bridge-span-4 | complete | 1 | 69 tick | 4.5 | 1 | 0 | 0 | 0 |
| bridge-span-5 | complete | 1 | 82 tick | 6 | 1 | 0 | 0 | 0 |
| bridge-span-6 | complete | 1 | 98 tick | 8 | 1 | 0 | 0 | 0 |
| bridge-span-7 | complete | 1 | 130 tick | 10 | 1 | 0 | 0 | 0 |
| bridge-span-8 | complete | 1 | 150 tick | 12.5 | 1 | 0 | 0 | 0 |
| cantilever-3 | complete | 1 | 40 tick | 10 | 1 | 0 | 0 | 0 |
| cantilever-4 | complete | 1 | 54 tick | 15 | 1 | 0 | 0 | 0 |
| cantilever-arm | complete | 1 | 19 tick | 6 | 1 | 0 | 0 | 0 |
| tower | complete | 1 | 14 tick | 6 | 1 | 0 | 0 | 0 |
| onlattice-bridge | complete | 1 | 37 tick | 3 | 1 | 0 | 0 | 0 |
| armadas-shelter | complete | 1 | 42.972 h | 48.7104 | 2.78677 | 0 | 0 | 0 |
| scaffold-span-4-crane | complete | 1 | 40 tick | 4.5 | 1 | 0 | 0 | 0 |
| scaffold-span-4-inchworm | complete | 1 | 169 tick | 4.5 | 1 | 3 | 0 | 0 |
| scaffold-span-8-crane | complete | 1 | 97 tick | 12.5 | 1 | 0 | 0 | 0 |
| fault-wall | complete | 1 | 527 tick |  |  | 0 | 7 | 7 |
| fault-tower | complete | 1 | 10 tick | 1 | 1 | 0 | 1 | 1 |

Cascades are 0 on every row. Unrecovered is 0. The fault wall's median recovery time is 18 ticks. The fault tower's single recovery is `re-place` at 5 ticks. The hollow roof at limit 0 records one stability refusal and does not start a search. The shelter records 502 gate accepts and 0 refusals.

Spans 7 and 8 finish. The stepper (reach 1, climb 1) starts beside the far pier. Nearest claim used to grow that side to the cantilever limit, then reclaim the nearer floating deck cells: a cooldown of 8 ticks expired before the scan reached the other pier, and the run sat at the 8000-tick cap (completion 0.5 and 0.454545). A cell the structural gate refuses now stays out of the claim window until a placement changes the prefix. The caps are unchanged: span 7 is still overhang 3 and sag 10, span 8 is overhang 4 and sag 12.5. With that deferral, span 7 finishes in 130 ticks (peak sag 10, 3 gate refusals) and span 8 in 150 ticks (peak sag 12.5, 4 gate refusals). Scaffold count on both is 0.

The crane (reach 4, climb 0) also finishes span 8 with scaffold count 0, in 97 ticks (`scaffold-span-8-crane`, peak sag 12.5). Span 4 with the crane inserts 0 scaffold cells and finishes in 40 ticks. The inchworm on span 4 inserts 3 scaffold cells, all removed by the end, and finishes in 169 ticks. The crane needs none. The inchworm needs three cells under the span-4 deck.

The cantilever arm and the tower peak at 6, the closed form `n(n+1)/2` for `n = 3` free voxels at `k = mass = g = 1`. Bridge span 4 peaks at 4.5. Cantilever length 3 peaks at 10 and length 4 at 15, the same peaks the stability campaign reports once the limit allows the arm.

The shelter is the 256-cell stand-in in `sim/armadas.py`, not the Fig. 5 coordinate list. One feed and one robot per role finishes in 154700 s, which is 42.972 h, the same 1×1 row as `docs/armadas_sweep.csv`. Peak sag is 48.7104 in units of `mass·g/k`. Joint utilization 2.78677 is the worst construction prefix against the finished structure's own peak demand, so it is above 1. Step times of 20 s are Park et al. 2023's "on the order of 20 seconds." Load, handoff, place, and fasten are configuration at that same 20 s. The published 304.8 mm voxel edge is metadata on the task. It is not the model pitch.

`k = 1` is not a fit. Stiffness has not been fit to a printed specimen.

## Submitting a planner

The comparison a lab is invited to make is an event log for these files, scored with the columns above, at the same unfitted `k`.

**NASA ARMADAS.** The shelter task is the three-role schedule (cargo, crane, fasten) from Gregg et al. 2024 (doi:10.1126/scirobotics.adi2746) as this repository already models it: one fasten step per face joint, one feed station, the 256-cell stand-in. A SOLL-E style submission uses `locomotion: "inchworm"` or the three-role schedule and writes placements, fastener faults, and recoveries in the event log. Report sag in `mass·g/k` with `fitted: false`. Do not put a calibrated modulus in `k` and compare that sag to 48.7104. The 1057.2365952 N face figure is a derivation from the published 3×3×3 strength. It is not the utilization denominator in this table.

**MIT CBA.** A BILL-E style submission is an inchworm document: two-foot stances, gait fields as configuration, feet starting on seed. The cuboct block is face-joined on the cubic grid. A planner for the 2013 vertex-connected octahedron needs a lattice this schema does not yet name; say so in the log note rather than relabeling `cuboctahedral`. The span-4 inchworm task is the one that records scaffold. The stock count is 3.

**TU Delft.** A scaffold-minimizing sequence is a log whose placements of type `scaffold` are the supports, later removed, with `scaffold_count` equal to how many distinct cells were inserted. The stock search is an exact enumeration of under-column supports (`sim/scaffold.py`). Crane spans in this suite insert 0. A shorter makespan at the same count, or a smaller count that still finishes inside the finished-structure caps, is the comparison. Caps for `gate: "finished"` are the finished blueprint's own overhang and its `worst_sag` at `k = mass = g = 1`.

Shared rules for all three:

- Read `benchmarks/tasks/<id>.json`. Write an `event_log` whose `scenario_id` is that id. Schema: [FORMAT.md](FORMAT.md).
- Keep `model.k = model.mass = model.g = 1`, `fitted: false`, displacement units `mass*g/k`, and a note that contains `not a fit`.
- Completion, makespan, peak sag, joint utilization, scaffold count, and the fault columns use the definitions in the table above.
- A fault rate, if you inject one, has `rate_source` `configuration`.
- The unit tests do not run the full bench. `python main.py bench smoke` covers nine tasks. The committed CSV is the full run.
