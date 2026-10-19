# Interchange format 1.0.0

A lab that already has a planner can read a scenario from this repository and write an event log back, without importing the simulator. The schema is [`schemas/discrete-assembly-1.0.0.json`](../schemas/discrete-assembly-1.0.0.json). The format name is `discrete-assembly`. The format version is `1.0.0`. The schema `$id` is `https://github.com/jdyar/discrete-assembly-sim/schemas/discrete-assembly-1.0.0.json`.

`sim/format.py` checks a document against that schema and against the cross-field rules the schema cannot state. The checker is a subset of JSON Schema draft 2020-12 (`type`, `enum`, `const`, `required`, `properties`, `additionalProperties` as a boolean or a schema, `items`, `minItems`, `maxItems`, `minimum`, `maximum`, `exclusiveMinimum`, `minLength`, `oneOf`, and document-local `$ref`). A boolean is not an integer. Dependencies stay NumPy and Matplotlib. There is no `jsonschema` package.

This file is the interchange. The replay viewer still reads the run log in `sim/metrics.py` (versions 1, 2, and 3). Those two JSON shapes are not the same document.

## Envelope

Every document is one JSON object. Four kinds, chosen by `kind`:

| kind | what it carries |
| --- | --- |
| `blueprint` | lattice, cells, seed, supports, spring model |
| `robots` | locomotion, roles, optional fault rates |
| `event_log` | placements, removals, faults, recoveries, gate verdicts |
| `scenario` | one blueprint, one robots document, and a task |

Required on every kind:

```json
{
  "format": "discrete-assembly",
  "format_version": "1.0.0",
  "kind": "blueprint"
}
```

`format` and `format_version` are constants. Unknown properties are rejected. Canonical text is `json.dumps` with indent 2 and a trailing newline (`sim.format.canonical`). `dump` validates and writes. `load` reads and validates. `validate` raises `FormatError`.

## Coordinates

An index is an array of integers.

- Square lattice: `[row, col]`. Row 0 is the top of the frame. `up_sign` is `-1`. The ground plane is the last row (`ground_plane = rows - 1`). The ground row is not a cell.
- Cubic and cuboctahedral lattices: `[level, row, col]`. Level 0 is the ground plane. Levels increase upward (`up_axis` 0, `up_sign` 1). The ground plane is not a cell.

`pitch` is the center spacing in model length. Every public task uses pitch `1`. A published millimeter edge, when a paper prints one, belongs in `meta`. It is not the pitch while `k = 1`.

`cuboctahedral` is the optional solid in `sim/cuboct.py`: the same six face-neighbors as the cubic grid, different vertices and square-face joint sites. It is not the 2013 vertex-connected octahedron (Cheung & Gershenfeld, Science, doi:10.1126/science.1240889). BILL-E walks that octahedral lattice. This format does not describe it.

## Cells, seed, and supports

`cells` is a list of `{index, voxel_type}`. The three types:

| voxel_type | meaning |
| --- | --- |
| `structural` | a blueprint cell still to place. It starts empty. |
| `seed` | a voxel already placed. It is not remaining work. |
| `scaffold` | a temporary support already in the start state. Occupancy `SCAFFOLD`. |

`seed` is exactly the indices whose type is `seed`, and `supports` is exactly the indices whose type is `scaffold`. Both lists exist so a planner can find the start structure without scanning types. Duplicate indices are rejected. A cell on the ground plane is rejected.

A planner that inserts support during a search does not edit `supports`. Those cells appear in the event log as placements of `voxel_type` `scaffold`, and their removal is a later event. The public inchworm span-4 task plants an entry pad as seed. That pad is not scaffold.

`world_from_blueprint` builds a `World` or `World3D`: structural cells become the blueprint mask, seed cells are placed voxels, support cells are scaffold. `blueprint_from_world` writes that occupancy back. Round trip is the same document.

## Spring model

Every blueprint carries a `model`:

| field | public suite |
| --- | --- |
| `k` | `1` |
| `mass` | `1` |
| `g` | `1` |
| `fitted` | `false` |
| `displacement_units` | `mass*g/k` |
| `force_units` | `mass*g` |
| `note` | must contain the words `not a fit` |

The note used by the suite is:

> k = 1 is not a fit. Displacement is in model units of mass\*g/k, not a measured sag. Stiffness has not been fit to a printed specimen.

`k = 1` is not a fit. Sag from the spring network is `mass·g/k`. Force at that setting is `mass·g`. Optional `k_axial` and `k_shear` may split the two springs; the public tasks leave them off and use one `k`.

A document with `fitted: true` must not report displacement in `mass*g/k`. `assert_unfitted_suite` refuses to score a public benchmark whose model is fitted or whose `k`, `mass`, or `g` is not 1. The derived ARMADAS 3×3×3 face compression of 1057.2365952 N is not this capacity and is not applied to a public row.

## Robots and roles

A robots document lists robots, a `roles` map, `defect_p`, `inspect`, and `rng_seed`.

Locomotion is `cell` or `inchworm`.

- A cell robot stands in an empty cell. It carries a `motion` object: `reach_radius`, `stride`, `climb`, `lattice_only`. Depot and start are indices. They may be the same cell. The 2D wall and the fault tower start at the depot. A cell pose must lie inside the grid, off the ground plane, and off every blueprint cell.
- An inchworm grips two voxel faces. It carries a `gait` and no cell `motion`. Depot and start are stances: `{feet: [{cell, normal}, {cell, normal}]}`. Each normal is one axis, components −1, 0, or 1. In a scenario, both feet start on seed voxels. `stance_document` and `stance_from_document` convert the Python stance node. Gait fields (stride, span, place reach, which faces are walkable) are configuration. They are not a joint-trajectory fit to BILL-E or SOLL-E.

The role name on a robot must exist in `roles`. A role has a `description` and may carry `mass`, `carried_mass`, and step times. Public cell robots use mass 0, so robot weight is off in the table. The crane snapshot has no feet and adds no point load.

`schedule: "three-role"` requires roles named `cargo`, `crane`, and `fasten`. That schedule is the shelter task. `robots_per_role` and `n_feeds` sit on the robots document and on the task. The public shelter uses one feed and one robot per role.

`defect_p` is the 2D Bernoulli bad-bond probability. It is configuration. Zero leaves that channel off. `inspect` is the inspect-after-place flag.

Optional `faults`:

```json
{
  "seed": 7,
  "rate_source": "configuration",
  "rates": {"fastener": 0.0},
  "script": ["fastener", "fastener"]
}
```

`rate_source` is the constant `configuration`. A rate is not a measurement. A rate of zero does not draw. `script` names channels in order: `capture`, `drop`, `stall`, `robot_fail`, `fastener`, `feed_outage`. The scripted fastener on the two-voxel tower is `["fastener", "fastener"]` because the first voxel has no partner face. Channel names match `sim.faults`.

## Event log

An event log names the `scenario_id`, a `clock` of `tick` or `second`, a `makespan`, and an `events` array. Makespan is the clock when the run stopped. A refusal before any step is 0. Swarm runs use ticks. The shelter schedule uses seconds.

Each event has `t` (a number, ≥ 0) and `type`:

| type | required fields | meaning |
| --- | --- | --- |
| `placement` | `cell`, `voxel_type` | a structural or scaffold voxel was placed |
| `removal` | `cell`, `voxel_type` | a voxel or a defect was removed |
| `fault` | `fault` | a channel fired |
| `recovery` | `action` | a recovery finished |
| `gate` | `gate`, `verdict` | a screen accepted or refused |

Fault channels: `capture`, `drop`, `stall`, `robot_fail`, `fastener`, `feed_outage`, `defect` (the 2D Bernoulli bond), `cascade`. A cascade is a fault that starts while another recovery is open. It is logged. It is not a second configured probability.

Recovery actions, the names in `sim.faults`: `re-place`, `re-route`, `reassign`, `scaffold`.

Gates: `connectivity`, `buildability`, `stability`, `sag`, `joint`, `coordination`. Verdicts: `accept`, `refuse`.

Optional fields: `robot`, `role`, `reason`, `joint` (two indices, the face the fastener failed on), `cascade`, `cause` (index of the open fault a cascade refers to).

A plan prefix may emit `stability` or `sag` verdicts at `t = 0` with `reason` `"prefix"`. A swarm kickback that does not name which inner gate failed is a `coordination` refuse. The stock extractor (`events_from_swarm`) skips motion nodes that are not cells. An inchworm stance is not a cell index.

`replay_completion` in `sim/bench.py` applies placements and removals onto the blueprint and returns the same completion a run reports after every bad bond has been replaced.

## What a lab sends back

1. Read `benchmarks/tasks/<id>.json`. It is a `scenario`.
2. Place every `structural` cell. Leave `seed` cells in place. If the task's `scaffold` is `minimal`, any support you insert is a placement of type `scaffold` and must be removed before the log ends. If `scaffold` is `none`, the finished log has no scaffold left.
3. Write an `event_log` with `scenario_id` equal to the task id. Clock `tick` unless you are scoring the shelter schedule, which is `second`.
4. Validate with `sim.format.load` or any draft 2020-12 checker against the schema file. Semantic rules (seed equals seed cells, inchworm feet on seed, unfitted note) are in `sim.format.validate`.
5. Score with the definitions in [BENCHMARKS.md](BENCHMARKS.md). `k`, `mass`, and `g` stay 1. `fitted` stays false. Do not replace the model block with a calibrated stiffness and then compare the sag column to this table.

The stock runner is `python main.py bench`. It calls the planners already in this repository. Submitting a planner means emitting an event log for these scenarios. This repository does not load an external planner.

## What this format does not claim

- `k = 1` is not a fit to a printed specimen. Displacement is not a measured sag.
- Pitch `1` is not 304.8 mm. The shelter note records that published edge as metadata.
- Cuboctahedral cells here are face-joined on the cubic grid. They are not ARMADAS hardware and not the 2013 vertex octahedron.
- Gait numbers and motion numbers are configuration.
- Fault probabilities are configuration. `rate_source` says so.
- Step times of 20 s on the shelter are Park et al. 2023's order of magnitude. Load, handoff, place, and fasten were not fitted to a clock.
- A submission is an event log for a scenario. Policy weights, prompts, and replication rules are outside this interchange.
