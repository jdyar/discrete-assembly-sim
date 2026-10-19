# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Versioned interchange for blueprints, robot roles, and event logs.

Documents are JSON Schema draft 2020-12, file
``schemas/discrete-assembly-1.0.0.json``, format version ``1.0.0``.
This module checks a document against that schema and against the
invariants the schema cannot state (a seed list is the seed cells, a
cell sits inside the grid, an unfitted model says so).

Four kinds:

* ``blueprint`` — lattice, pitch, cells with a voxel type, seed, supports
* ``robots`` — motion or gait, roles, and optional fault rates
* ``event_log`` — placements, removals, faults, recoveries, gate verdicts
* ``scenario`` — one blueprint, one robot document, and a task

The public suite uses ``k = mass = g = 1``. That stiffness is not a
fit. Displacement is in units of ``mass*g/k``. Import and export round
trip: a blueprint loaded into a world and written again is the same
document.

Fault channel names and recovery names are the ones in
:mod:`sim.faults`. A rate is configuration. ``rate_source`` is the
constant ``configuration``.
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from .inchworm import make_stance, stance_faces
from .world import SCAFFOLD, VOXEL, World
from .world3d import World3D

FORMAT = "discrete-assembly"
FORMAT_VERSION = "1.0.0"
SCHEMA_NAME = "discrete-assembly-1.0.0.json"

UNFITTED_NOTE = (
    "k = 1 is not a fit. Displacement is in model units of mass*g/k, "
    "not a measured sag. Stiffness has not been fit to a printed specimen."
)

VOXEL_TYPES = ("structural", "seed", "scaffold")
LATTICE_TYPES = ("square", "cubic", "cuboctahedral")
EVENT_TYPES = ("placement", "removal", "fault", "recovery", "gate")
FAULT_NAMES = (
    "capture",
    "drop",
    "stall",
    "robot_fail",
    "fastener",
    "feed_outage",
    "defect",
    "cascade",
)
RECOVERY_NAMES = ("re-place", "re-route", "reassign", "scaffold")
GATE_NAMES = (
    "connectivity",
    "buildability",
    "stability",
    "sag",
    "joint",
    "coordination",
)
VERDICTS = ("accept", "refuse")

_SCHEMA: dict | None = None


class FormatError(ValueError):
    """A document failed the schema or a cross-field invariant."""


def schema_path() -> Path:
    """The schema file next to the repository ``sim`` package."""
    return Path(__file__).resolve().parents[1] / "schemas" / SCHEMA_NAME


def schema() -> dict:
    """The parsed schema. Loaded once."""
    global _SCHEMA
    if _SCHEMA is None:
        path = schema_path()
        if not path.is_file():
            raise FormatError(f"schema file is missing: {path}")
        _SCHEMA = json.loads(path.read_text(encoding="utf-8"))
    return _SCHEMA


def canonical(document: dict) -> str:
    """Stable JSON text, insertion order, trailing newline."""
    return json.dumps(document, indent=2, ensure_ascii=False) + "\n"


def dump(document: dict, path: str | Path) -> Path:
    """Validate and write ``document``."""
    validate(document)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(canonical(document), encoding="utf-8")
    return path


def load(path: str | Path) -> dict:
    """Read JSON and validate it."""
    document = json.loads(Path(path).read_text(encoding="utf-8"))
    validate(document)
    return document


def unfitted_model(k: float = 1.0, mass: float = 1.0, g: float = 1.0) -> dict:
    """The public-suite spring. ``fitted`` is false. The note says so."""
    return {
        "k": float(k),
        "mass": float(mass),
        "g": float(g),
        "fitted": False,
        "displacement_units": "mass*g/k",
        "force_units": "mass*g",
        "note": UNFITTED_NOTE,
    }


def cubic_lattice(
    levels: int,
    rows: int,
    cols: int,
    *,
    kind: str = "cubic",
    pitch: float = 1.0,
) -> dict:
    if kind not in ("cubic", "cuboctahedral"):
        raise FormatError(f"cubic lattice kind must be cubic or cuboctahedral, got {kind}")
    return {
        "type": kind,
        "pitch": float(pitch),
        "dimensions": {"levels": int(levels), "rows": int(rows), "cols": int(cols)},
        "up_axis": 0,
        "up_sign": 1,
        "ground_plane": 0,
    }


def square_lattice(rows: int, cols: int, *, pitch: float = 1.0) -> dict:
    return {
        "type": "square",
        "pitch": float(pitch),
        "dimensions": {"rows": int(rows), "cols": int(cols)},
        "up_axis": 0,
        "up_sign": -1,
        "ground_plane": int(rows) - 1,
    }


def _index(cell) -> list[int]:
    return [int(v) for v in cell]


def _is_cell(node) -> bool:
    """True for a row/col or level/row/col index. Stances are not cells."""
    return (
        isinstance(node, tuple)
        and 2 <= len(node) <= 3
        and all(isinstance(value, int) and not isinstance(value, bool) for value in node)
    )


def make_blueprint(
    *,
    ident: str,
    lattice: dict,
    cells: list[tuple],
    title: str = "",
    model: dict | None = None,
    meta: dict | None = None,
) -> dict:
    """Build a blueprint document from ``(index, voxel_type)`` pairs.

    ``cells`` may be unsorted. The written lists are sorted by index.
    ``seed`` and ``supports`` are the cells of those types.
    """
    packed = []
    for index, voxel_type in cells:
        if voxel_type not in VOXEL_TYPES:
            raise FormatError(f"unknown voxel type {voxel_type!r}")
        packed.append({"index": _index(index), "voxel_type": voxel_type})
    packed.sort(key=lambda item: tuple(item["index"]))
    document: dict[str, Any] = {
        "format": FORMAT,
        "format_version": FORMAT_VERSION,
        "kind": "blueprint",
        "id": ident,
        "lattice": lattice,
        "cells": packed,
        "seed": [
            item["index"] for item in packed if item["voxel_type"] == "seed"
        ],
        "supports": [
            item["index"] for item in packed if item["voxel_type"] == "scaffold"
        ],
        "model": model if model is not None else unfitted_model(),
    }
    if title:
        document["title"] = title
    if meta:
        document["meta"] = meta
    validate(document)
    return document


def make_robots(
    *,
    ident: str,
    robots: list[dict],
    roles: dict,
    defect_p: float = 0.0,
    inspect: bool = True,
    rng_seed: int = 0,
    schedule: str | None = None,
    robots_per_role: int | None = None,
    n_feeds: int | None = None,
    faults: dict | None = None,
) -> dict:
    document: dict[str, Any] = {
        "format": FORMAT,
        "format_version": FORMAT_VERSION,
        "kind": "robots",
        "id": ident,
        "robots": robots,
        "roles": roles,
        "defect_p": float(defect_p),
        "inspect": bool(inspect),
        "rng_seed": int(rng_seed),
    }
    if schedule is not None:
        document["schedule"] = schedule
    if robots_per_role is not None:
        document["robots_per_role"] = int(robots_per_role)
    if n_feeds is not None:
        document["n_feeds"] = int(n_feeds)
    if faults is not None:
        document["faults"] = faults
    validate(document)
    return document


def make_scenario(
    *,
    ident: str,
    title: str,
    family: str,
    blueprint: dict,
    robots: dict,
    task: dict,
    notes: str = "",
    meta: dict | None = None,
) -> dict:
    document: dict[str, Any] = {
        "format": FORMAT,
        "format_version": FORMAT_VERSION,
        "kind": "scenario",
        "id": ident,
        "title": title,
        "family": family,
        "blueprint": blueprint,
        "robots": robots,
        "task": task,
    }
    if notes:
        document["notes"] = notes
    if meta:
        document["meta"] = meta
    validate(document)
    return document


def make_event_log(
    *,
    scenario_id: str,
    clock: str,
    makespan: float,
    events: list[dict],
) -> dict:
    document = {
        "format": FORMAT,
        "format_version": FORMAT_VERSION,
        "kind": "event_log",
        "scenario_id": scenario_id,
        "clock": clock,
        "makespan": float(makespan),
        "events": events,
    }
    validate(document)
    return document


def cell_robot(
    *,
    ident: str,
    role: str,
    depot,
    start,
    motion: dict,
    description: str,
) -> dict:
    return {
        "id": ident,
        "role": role,
        "locomotion": "cell",
        "motion": motion,
        "depot": _index(depot),
        "start": _index(start),
    }


def inchworm_robot(
    *,
    ident: str,
    role: str,
    depot_stance,
    start_stance,
    gait: dict,
) -> dict:
    return {
        "id": ident,
        "role": role,
        "locomotion": "inchworm",
        "gait": gait,
        "depot": stance_document(depot_stance),
        "start": stance_document(start_stance),
    }


def stance_document(stance) -> dict:
    """Canonical two-foot pose. Foot order is :func:`make_stance` order."""
    faces = stance_faces(stance)
    if faces is None:
        raise FormatError("not an inchworm stance")
    return {
        "feet": [
            {"cell": _index(cell), "normal": _index(normal)}
            for cell, normal in faces
        ]
    }


def stance_from_document(pose: dict):
    """The stance node a planner can pass to the inchworm geometry."""
    feet = pose["feet"]
    faces = [
        (tuple(int(v) for v in foot["cell"]), tuple(int(v) for v in foot["normal"]))
        for foot in feet
    ]
    return make_stance(faces[0], faces[1])


def pose_node(pose):
    """A cell tuple or a stance node."""
    if isinstance(pose, list):
        return tuple(int(v) for v in pose)
    return stance_from_document(pose)


def cells_from_world(world) -> list[tuple]:
    """Initial occupancy as ``(index, voxel_type)`` pairs.

    Blueprint cells are structural and must still be empty. Voxels
    already placed are seed. Scaffold already placed is support.
    """
    found = []
    for raw in zip(*world.blueprint.nonzero()):
        cell = tuple(int(v) for v in raw)
        if int(world.occupancy[cell]) != 0:
            raise FormatError(
                f"blueprint cell {cell} is already occupied; "
                "seed and support are not blueprint cells"
            )
        found.append((cell, "structural"))
    occupancy = world.occupancy
    solid = (occupancy == VOXEL) | (occupancy == SCAFFOLD)
    for raw in zip(*solid.nonzero()):
        cell = tuple(int(v) for v in raw)
        if bool(world.blueprint[cell]):
            raise FormatError(f"occupied blueprint cell {cell}")
        kind = "scaffold" if int(occupancy[cell]) == SCAFFOLD else "seed"
        found.append((cell, kind))
    return found


def blueprint_from_world(
    world,
    *,
    ident: str,
    lattice: dict,
    title: str = "",
    model: dict | None = None,
    meta: dict | None = None,
) -> dict:
    """Export ``world`` as a blueprint document."""
    return make_blueprint(
        ident=ident,
        lattice=lattice,
        cells=cells_from_world(world),
        title=title,
        model=model,
        meta=meta,
    )


def world_from_blueprint(document: dict):
    """A :class:`~sim.world.World` or :class:`~sim.world3d.World3D`.

    Structural cells are the blueprint. Seed cells are placed voxels.
    Support cells are scaffold. The ground plane is the lattice ground,
    not a cell in the document.
    """
    validate(document if document.get("kind") == "blueprint" else _as_blueprint(document))
    blueprint = document if document.get("kind") == "blueprint" else document
    lattice = blueprint["lattice"]
    dims = lattice["dimensions"]
    if lattice["type"] == "square":
        world: World | World3D = World(rows=int(dims["rows"]), cols=int(dims["cols"]))
    else:
        world = World3D(
            levels=int(dims["levels"]),
            rows=int(dims["rows"]),
            cols=int(dims["cols"]),
        )
    for item in blueprint["cells"]:
        cell = tuple(int(v) for v in item["index"])
        kind = item["voxel_type"]
        if kind == "structural":
            world.blueprint[cell] = True
        elif kind == "seed":
            world.place_voxel(cell)
        else:
            world.place_voxel(cell, scaffold=True)
    return world


def _as_blueprint(document: dict) -> dict:
    if document.get("kind") != "blueprint":
        raise FormatError("world_from_blueprint expects a blueprint document")
    return document


def lattice_for_world(world, *, kind: str = "cubic", pitch: float = 1.0) -> dict:
    """Lattice block matching ``world``'s array shape."""
    if world.occupancy.ndim == 2:
        return square_lattice(world.rows, world.cols, pitch=pitch)
    return cubic_lattice(
        world.levels, world.rows, world.cols, kind=kind, pitch=pitch
    )


def events_from_injector(injector) -> list[dict]:
    """Fault and recovery events from a :class:`~sim.faults.FaultInjector`.

    A fault that has been closed also contributes a recovery event at
    ``done_tick`` or ``done_time_s``. Cascades stay in the log with
    ``cascade`` true. The rate is not copied: the scenario already
    stores it, and a cascade's rate is zero.
    """
    events: list[dict] = []
    for index, fault in enumerate(injector.events):
        opened = fault.tick if fault.tick is not None else fault.time_s
        item: dict[str, Any] = {
            "t": 0 if opened is None else float(opened),
            "type": "fault",
            "fault": fault.kind,
            "cascade": bool(fault.cascade),
        }
        if fault.robot is not None:
            item["robot"] = f"r{fault.robot}"
        if fault.role:
            item["role"] = fault.role
        if fault.cell is not None:
            item["cell"] = _index(fault.cell)
        if fault.joint is not None:
            item["joint"] = [_index(side) for side in fault.joint]
        if fault.cause is not None:
            item["cause"] = int(fault.cause)
        item["reason"] = fault.recovery
        events.append(item)
        if not fault.done:
            continue
        closed = fault.done_tick if fault.done_tick is not None else fault.done_time_s
        recovery: dict[str, Any] = {
            "t": float(closed),
            "type": "recovery",
            "action": fault.recovery,
            "fault": fault.kind,
            "cause": index,
        }
        if "robot" in item:
            recovery["robot"] = item["robot"]
        if "role" in item:
            recovery["role"] = item["role"]
        if "cell" in item:
            recovery["cell"] = item["cell"]
        events.append(recovery)
    return events


def events_from_swarm(swarm) -> list[dict]:
    """Placements, scaffold removals, defect removals, and kickbacks.

    A kickback is a coordination-gate refusal. The swarm does not record
    which of the connectivity, buildability, or stability checks inside
    the choreographer returned it.
    """
    events = []
    for tick, robot_id, kind, node in swarm.events:
        if kind not in ("place", "scaffold_removed", "defect_removed", "kickback"):
            continue
        robot = f"r{robot_id}"
        cell = _index(node) if _is_cell(node) else None
        if cell is None:
            if kind != "kickback":
                continue
        if kind == "place":
            voxel = "scaffold" if node in swarm.scaffold_cells else "structural"
            events.append(
                {
                    "t": int(tick),
                    "type": "placement",
                    "cell": cell,
                    "robot": robot,
                    "voxel_type": voxel,
                }
            )
        elif kind == "scaffold_removed":
            events.append(
                {
                    "t": int(tick),
                    "type": "removal",
                    "cell": cell,
                    "robot": robot,
                    "voxel_type": "scaffold",
                    "reason": "scaffold",
                }
            )
        elif kind == "defect_removed":
            events.append(
                {
                    "t": int(tick),
                    "type": "removal",
                    "cell": cell,
                    "robot": robot,
                    "voxel_type": "structural",
                    "reason": "defect",
                }
            )
        elif kind == "kickback":
            item = {
                "t": int(tick),
                "type": "gate",
                "gate": "coordination",
                "verdict": "refuse",
                "robot": robot,
                "reason": "kickback",
            }
            if cell is not None:
                item["cell"] = cell
            events.append(item)
    return events


def gate_event(
    *,
    t: float,
    gate: str,
    verdict: str,
    cell=None,
    reason: str = "",
    robot: str | None = None,
) -> dict:
    item: dict[str, Any] = {
        "t": float(t),
        "type": "gate",
        "gate": gate,
        "verdict": verdict,
    }
    if cell is not None:
        item["cell"] = _index(cell)
    if reason:
        item["reason"] = reason
    if robot is not None:
        item["robot"] = robot
    return item


def assert_unfitted_suite(blueprint: dict) -> None:
    """Public benchmarks score ``k = mass = g = 1`` with ``fitted`` false."""
    model = blueprint["model"]
    if model.get("fitted"):
        raise FormatError("a public benchmark model is not a fit")
    if float(model["k"]) != 1.0 or float(model["mass"]) != 1.0 or float(model["g"]) != 1.0:
        raise FormatError(
            "public benchmarks score k = mass = g = 1, which is not a fit"
        )
    if model.get("displacement_units") != "mass*g/k":
        raise FormatError("public benchmark displacement is mass*g/k")


# --- schema subset ----------------------------------------------------------


def _resolve(root: dict, ref: str) -> dict:
    if not isinstance(ref, str) or not ref.startswith("#/"):
        raise FormatError(f"{ref}: only document-local refs are supported")
    node: Any = root
    for part in ref[2:].split("/"):
        part = part.replace("~1", "/").replace("~0", "~")
        if not isinstance(node, dict) or part not in node:
            raise FormatError(f"unresolved schema ref {ref}")
        node = node[part]
    if not isinstance(node, dict):
        raise FormatError(f"schema ref {ref} is not an object")
    return node


def _fail(path: str, message: str) -> None:
    raise FormatError(f"{path}: {message}")


def _type_ok(value: Any, expected: str) -> bool:
    if expected == "object":
        return isinstance(value, dict)
    if expected == "array":
        return isinstance(value, list)
    if expected == "string":
        return isinstance(value, str)
    if expected == "boolean":
        return isinstance(value, bool)
    if expected == "integer":
        return isinstance(value, int) and not isinstance(value, bool)
    if expected == "number":
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    if expected == "null":
        return value is None
    _fail("$schema", f"unsupported type {expected}")
    return False


def _check(instance: Any, node: dict, root: dict, path: str) -> None:
    if "$ref" in node:
        _check(instance, _resolve(root, node["$ref"]), root, path)
        return
    if "oneOf" in node:
        matched = 0
        for branch in node["oneOf"]:
            try:
                _check(instance, branch, root, path)
            except FormatError:
                continue
            matched += 1
        if matched != 1:
            _fail(path, f"oneOf matched {matched} branches, expected 1")
        return
    if "const" in node and instance != node["const"]:
        _fail(path, f"expected {node['const']!r}")
    if "enum" in node and instance not in node["enum"]:
        _fail(path, f"{instance!r} is not one of {node['enum']!r}")
    if "type" in node and not _type_ok(instance, node["type"]):
        _fail(path, f"expected type {node['type']}")
    if "minLength" in node and isinstance(instance, str) and len(instance) < node["minLength"]:
        _fail(path, "string is shorter than minLength")
    if "minimum" in node and isinstance(instance, (int, float)) and not isinstance(instance, bool):
        if instance < node["minimum"]:
            _fail(path, f"below minimum {node['minimum']}")
    if "maximum" in node and isinstance(instance, (int, float)) and not isinstance(instance, bool):
        if instance > node["maximum"]:
            _fail(path, f"above maximum {node['maximum']}")
    if (
        "exclusiveMinimum" in node
        and isinstance(instance, (int, float))
        and not isinstance(instance, bool)
        and instance <= node["exclusiveMinimum"]
    ):
        _fail(path, f"must be greater than {node['exclusiveMinimum']}")
    if "minItems" in node and isinstance(instance, list) and len(instance) < node["minItems"]:
        _fail(path, "fewer items than minItems")
    if "maxItems" in node and isinstance(instance, list) and len(instance) > node["maxItems"]:
        _fail(path, "more items than maxItems")
    if "items" in node and isinstance(instance, list):
        for index, item in enumerate(instance):
            _check(item, node["items"], root, f"{path}[{index}]")
    if "properties" in node and isinstance(instance, dict):
        properties = node["properties"]
        for key, value in instance.items():
            if key in properties:
                _check(value, properties[key], root, f"{path}.{key}")
        if "required" in node:
            for key in node["required"]:
                if key not in instance:
                    _fail(path, f"missing required property {key!r}")
        additional = node.get("additionalProperties", True)
        if additional is False:
            extra = [key for key in instance if key not in properties]
            if extra:
                _fail(path, f"unexpected properties {extra}")
        elif isinstance(additional, dict):
            for key, value in instance.items():
                if key not in properties:
                    _check(value, additional, root, f"{path}.{key}")


def validate(document: dict) -> None:
    """Raise :class:`FormatError` if ``document`` is not a 1.0.0 document."""
    if not isinstance(document, dict):
        _fail("$", "document must be an object")
    kind = document.get("kind")
    if kind not in ("blueprint", "robots", "event_log", "scenario"):
        _fail("$.kind", f"unknown kind {kind!r}")
    root = schema()
    _check(document, {"$ref": f"#/$defs/{kind}"}, root, "$")
    _semantic(document)


def _semantic(document: dict) -> None:
    kind = document["kind"]
    if document.get("format") != FORMAT or document.get("format_version") != FORMAT_VERSION:
        _fail("$", f"format must be {FORMAT} {FORMAT_VERSION}")
    if kind == "blueprint":
        _semantic_blueprint(document)
    elif kind == "robots":
        _semantic_robots(document)
    elif kind == "event_log":
        _semantic_events(document)
    elif kind == "scenario":
        _semantic_blueprint(document["blueprint"])
        _semantic_robots(document["robots"])
        _semantic_scenario(document)


def _typed_indices(cells: list[dict], voxel_type: str) -> list[tuple]:
    return sorted(
        tuple(item["index"])
        for item in cells
        if item["voxel_type"] == voxel_type
    )


def _in_bounds(index: tuple, lattice: dict) -> bool:
    dims = lattice["dimensions"]
    if lattice["type"] == "square":
        if len(index) != 2:
            return False
        row, col = index
        return 0 <= row < int(dims["rows"]) and 0 <= col < int(dims["cols"])
    if len(index) != 3 or "levels" not in dims:
        return False
    level, row, col = index
    return (
        0 <= level < int(dims["levels"])
        and 0 <= row < int(dims["rows"])
        and 0 <= col < int(dims["cols"])
    )


def _on_ground(index: tuple, lattice: dict) -> bool:
    plane = int(lattice["ground_plane"])
    if lattice["type"] == "square":
        return len(index) == 2 and index[0] == plane
    return len(index) == 3 and index[0] == plane


def _semantic_blueprint(document: dict) -> None:
    lattice = document["lattice"]
    if lattice["type"] == "square" and "levels" in lattice["dimensions"]:
        _fail("$.lattice.dimensions", "a square lattice has no levels")
    if lattice["type"] != "square" and "levels" not in lattice["dimensions"]:
        _fail("$.lattice.dimensions", "a 3D lattice needs levels")
    cells = document["cells"]
    seen = []
    for item in cells:
        index = tuple(item["index"])
        if index in seen:
            _fail("$.cells", f"duplicate index {list(index)}")
        seen.append(index)
        if not _in_bounds(index, lattice):
            _fail("$.cells", f"index {list(index)} is outside the lattice")
        if _on_ground(index, lattice):
            _fail("$.cells", f"index {list(index)} is on the ground plane")
    seed = sorted(tuple(item) for item in document["seed"])
    supports = sorted(tuple(item) for item in document["supports"])
    if seed != _typed_indices(cells, "seed"):
        _fail("$.seed", "seed must be exactly the cells of voxel_type seed")
    if supports != _typed_indices(cells, "scaffold"):
        _fail("$.supports", "supports must be exactly the cells of voxel_type scaffold")
    model = document["model"]
    if model["fitted"] is False:
        if "not a fit" not in model["note"]:
            _fail("$.model.note", "an unfitted model note must say it is not a fit")
        if model["displacement_units"] != "mass*g/k":
            _fail("$.model.displacement_units", "an unfitted displacement unit is mass*g/k")
        if model["force_units"] != "mass*g":
            _fail("$.model.force_units", "an unfitted force unit is mass*g")
    elif model["displacement_units"] == "mass*g/k":
        _fail("$.model", "a fitted model does not report mass*g/k")


def _unit_normal(normal: list) -> bool:
    if len(normal) != 3 or any(component not in (-1, 0, 1) for component in normal):
        return False
    return sum(abs(component) for component in normal) == 1


def _semantic_robots(document: dict) -> None:
    roles = document["roles"]
    seen = set()
    for robot in document["robots"]:
        if robot["id"] in seen:
            _fail("$.robots", f"duplicate robot id {robot['id']!r}")
        seen.add(robot["id"])
        if robot["role"] not in roles:
            _fail("$.robots", f"role {robot['role']!r} is not defined")
        locomotion = robot["locomotion"]
        if locomotion == "cell":
            if "motion" not in robot:
                _fail("$.robots", f"{robot['id']} needs a motion model")
            if "gait" in robot:
                _fail("$.robots", f"{robot['id']} is a cell robot and has a gait")
            for key in ("depot", "start"):
                if not isinstance(robot[key], list):
                    _fail(f"$.robots.{key}", "a cell robot pose is an index")
        elif locomotion == "inchworm":
            if "gait" not in robot:
                _fail("$.robots", f"{robot['id']} needs a gait")
            for key in ("depot", "start"):
                pose = robot[key]
                if not isinstance(pose, dict):
                    _fail(f"$.robots.{key}", "an inchworm pose is a stance")
                for foot in pose["feet"]:
                    if not _unit_normal(foot["normal"]):
                        _fail("$.robots", "a foot normal is one axis, components -1, 0, or 1")
    if document.get("schedule") == "three-role":
        missing = [name for name in ("cargo", "crane", "fasten") if name not in roles]
        if missing:
            _fail("$.roles", f"three-role schedule needs {missing}")
    faults = document.get("faults")
    if faults is not None and faults.get("rate_source") != "configuration":
        _fail("$.faults.rate_source", "fault rates are configuration, not a measurement")


def _semantic_events(document: dict) -> None:
    for index, event in enumerate(document["events"]):
        path = f"$.events[{index}]"
        kind = event["type"]
        if kind in ("placement", "removal"):
            if "cell" not in event or "voxel_type" not in event:
                _fail(path, f"{kind} needs a cell and a voxel_type")
        elif kind == "fault":
            if "fault" not in event:
                _fail(path, "a fault event needs a fault channel")
        elif kind == "recovery":
            if "action" not in event:
                _fail(path, "a recovery event needs an action")
        elif kind == "gate":
            if "gate" not in event or "verdict" not in event:
                _fail(path, "a gate event needs a gate and a verdict")
        if "joint" in event:
            for side in event["joint"]:
                if not isinstance(side, list):
                    _fail(path, "a joint side is an index")


def _occupied(blueprint: dict) -> set[tuple]:
    return {tuple(item["index"]) for item in blueprint["cells"]}


def _semantic_scenario(document: dict) -> None:
    blueprint = document["blueprint"]
    robots = document["robots"]
    task = document["task"]
    lattice = blueprint["lattice"]
    occupied = _occupied(blueprint)
    seeds = {tuple(item) for item in blueprint["seed"]}
    if task["runner"] == "armadas":
        if robots.get("schedule") != "three-role":
            _fail("$.task", "the armadas runner needs a three-role schedule")
    elif task["runner"] == "wall2d":
        if lattice["type"] != "square":
            _fail("$.task", "wall2d runs on the square lattice")
    elif task["runner"] in ("swarm", "scaffold", "precheck"):
        if lattice["type"] == "square":
            _fail("$.task", f"{task['runner']} runs on a 3D lattice")
    if task["runner"] != "armadas":
        n_robots = int(task.get("n_robots", len(robots["robots"])))
        if n_robots != len(robots["robots"]):
            _fail("$.task.n_robots", "n_robots must match the robot list")
    if task["scaffold"] == "minimal" and task["runner"] != "scaffold":
        _fail("$.task", "minimal scaffold is the scaffold runner")
    if task["scaffold"] == "none" and task["runner"] == "scaffold":
        _fail("$.task", "the scaffold runner searches minimal support")
    for robot in robots["robots"]:
        if robot["locomotion"] == "cell":
            for key in ("depot", "start"):
                cell = tuple(robot[key])
                if not _in_bounds(cell, lattice):
                    _fail(f"$.robots.{key}", f"{list(cell)} is outside the blueprint")
                if cell in occupied:
                    _fail(f"$.robots.{key}", f"{list(cell)} is a blueprint cell")
                if _on_ground(cell, lattice):
                    _fail(f"$.robots.{key}", f"{list(cell)} is on the ground plane")
        else:
            for key in ("depot", "start"):
                for foot in robot[key]["feet"]:
                    cell = tuple(foot["cell"])
                    if cell not in seeds:
                        _fail(
                            f"$.robots.{key}",
                            f"inchworm foot {list(cell)} must start on a seed voxel",
                        )
