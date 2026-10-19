# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""Interchange 1.0.0: schema, validator, import, export, round trip.

The public scenarios keep k = mass = g = 1. That stiffness is not a fit.
"""

from __future__ import annotations

import copy
import json
import tempfile
import unittest
from pathlib import Path

from sim.format import (
    FORMAT,
    FORMAT_VERSION,
    UNFITTED_NOTE,
    FormatError,
    blueprint_from_world,
    canonical,
    cubic_lattice,
    dump,
    load,
    make_blueprint,
    make_event_log,
    make_robots,
    make_scenario,
    schema,
    schema_path,
    square_lattice,
    validate,
    world_from_blueprint,
)
from sim.scenarios import SUITE, catalog
from sim.world import SCAFFOLD, VOXEL

_DOCUMENTS = None


def documents() -> dict:
    global _DOCUMENTS
    if _DOCUMENTS is None:
        _DOCUMENTS = catalog()
    return _DOCUMENTS


def _minimal() -> dict:
    return make_blueprint(
        ident="min",
        lattice=cubic_lattice(3, 3, 3),
        cells=[((1, 1, 1), "structural")],
    )


def _robots() -> dict:
    return make_robots(
        ident="min.robots",
        robots=[
            {
                "id": "r0",
                "role": "builder",
                "locomotion": "cell",
                "motion": {
                    "reach_radius": 1,
                    "stride": 1,
                    "climb": 1,
                    "lattice_only": False,
                },
                "depot": [1, 0, 0],
                "start": [1, 0, 1],
            }
        ],
        roles={"builder": {"description": "test stepper"}},
    )


class SchemaDocument(unittest.TestCase):
    def test_schema_file_is_draft_2020_12(self):
        raw = json.loads(schema_path().read_text(encoding="utf-8"))
        self.assertEqual(
            raw["$schema"], "https://json-schema.org/draft/2020-12/schema"
        )

    def test_schema_id_names_the_version(self):
        self.assertIn(FORMAT_VERSION, schema()["$id"])
        self.assertIn("discrete-assembly-1.0.0.json", schema()["$id"])

    def test_every_document_const_is_this_version(self):
        text = schema_path().read_text(encoding="utf-8")
        self.assertIn('"const": "1.0.0"', text)
        self.assertNotIn('"const": "0.', text)
        self.assertEqual(FORMAT, "discrete-assembly")

    def test_schema_defines_the_four_kinds(self):
        defs = schema()["$defs"]
        for name in ("blueprint", "robots", "event_log", "scenario"):
            self.assertEqual(defs[name]["properties"]["kind"]["const"], name)

    def test_voxel_types_are_structural_seed_and_scaffold(self):
        kinds = schema()["$defs"]["cell"]["properties"]["voxel_type"]["enum"]
        self.assertEqual(kinds, ["structural", "seed", "scaffold"])

    def test_recovery_names_match_the_fault_module(self):
        actions = schema()["$defs"]["event"]["properties"]["action"]["enum"]
        self.assertEqual(actions, ["re-place", "re-route", "reassign", "scaffold"])

    def test_fault_channels_include_the_six_configured_ones(self):
        faults = schema()["$defs"]["event"]["properties"]["fault"]["enum"]
        for name in (
            "capture",
            "drop",
            "stall",
            "robot_fail",
            "fastener",
            "feed_outage",
        ):
            self.assertIn(name, faults)

    def test_gate_verdicts_are_accept_and_refuse(self):
        verdicts = schema()["$defs"]["event"]["properties"]["verdict"]["enum"]
        self.assertEqual(verdicts, ["accept", "refuse"])


class RejectDocuments(unittest.TestCase):
    def test_rejects_a_non_object(self):
        with self.assertRaises(FormatError):
            validate([])  # type: ignore[arg-type]

    def test_rejects_an_empty_object(self):
        with self.assertRaises(FormatError):
            validate({})

    def test_rejects_an_unknown_kind(self):
        with self.assertRaises(FormatError):
            validate({"kind": "planner"})

    def test_rejects_an_extra_property(self):
        document = _minimal()
        document["extra"] = 1
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_different_format_version(self):
        document = _minimal()
        document["format_version"] = "0.9.0"
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_zero_pitch(self):
        document = _minimal()
        document["lattice"]["pitch"] = 0
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_negative_pitch(self):
        document = _minimal()
        document["lattice"]["pitch"] = -1
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_boolean_index(self):
        document = _minimal()
        document["cells"][0]["index"] = [True, 0, 0]
        document["seed"] = []
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_rank_1_index(self):
        document = _minimal()
        document["cells"][0]["index"] = [1]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_rank_4_index(self):
        document = _minimal()
        document["cells"][0]["index"] = [1, 1, 1, 1]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_an_unknown_voxel_type(self):
        document = _minimal()
        document["cells"][0]["voxel_type"] = "strut"
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_duplicate_index(self):
        document = _minimal()
        document["cells"].append({"index": [1, 1, 1], "voxel_type": "seed"})
        document["seed"] = [[1, 1, 1]]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_an_index_outside_the_grid(self):
        document = _minimal()
        document["cells"][0]["index"] = [1, 1, 9]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_cell_on_the_ground_plane(self):
        document = _minimal()
        document["cells"][0]["index"] = [0, 1, 1]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_levels_on_a_square_lattice(self):
        document = make_blueprint(
            ident="flat",
            lattice=square_lattice(4, 4),
            cells=[((2, 1), "structural")],
        )
        document["lattice"]["dimensions"]["levels"] = 2
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_cubic_lattice_without_levels(self):
        document = _minimal()
        del document["lattice"]["dimensions"]["levels"]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_square_index_on_a_cubic_lattice(self):
        document = _minimal()
        document["cells"][0]["index"] = [1, 1]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_seed_list_that_does_not_match(self):
        document = _minimal()
        document["seed"] = [[1, 1, 1]]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_support_list_that_does_not_match(self):
        document = _minimal()
        document["supports"] = [[1, 0, 1]]
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_an_unfitted_note_that_does_not_say_so(self):
        document = _minimal()
        document["model"]["note"] = "calibrated yesterday"
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_meter_units_on_an_unfitted_model(self):
        document = _minimal()
        document["model"]["displacement_units"] = "m"
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_mass_units_on_a_fitted_model(self):
        document = _minimal()
        document["model"]["fitted"] = True
        document["model"]["note"] = "A caller supplied a fit."
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_gate_event_without_a_verdict(self):
        with self.assertRaises(FormatError):
            make_event_log(
                scenario_id="min",
                clock="tick",
                makespan=0,
                events=[{"t": 0, "type": "gate", "gate": "stability"}],
            )

    def test_rejects_a_placement_without_a_cell(self):
        with self.assertRaises(FormatError):
            make_event_log(
                scenario_id="min",
                clock="tick",
                makespan=1,
                events=[{"t": 1, "type": "placement", "voxel_type": "structural"}],
            )

    def test_rejects_an_unknown_event_type(self):
        with self.assertRaises(FormatError):
            make_event_log(
                scenario_id="min",
                clock="tick",
                makespan=0,
                events=[{"t": 0, "type": "thought"}],
            )

    def test_rejects_an_unknown_fault_channel(self):
        with self.assertRaises(FormatError):
            make_event_log(
                scenario_id="min",
                clock="tick",
                makespan=1,
                events=[{"t": 1, "type": "fault", "fault": "panic"}],
            )

    def test_rejects_an_unknown_recovery_action(self):
        with self.assertRaises(FormatError):
            make_event_log(
                scenario_id="min",
                clock="tick",
                makespan=1,
                events=[{"t": 1, "type": "recovery", "action": "improvise"}],
            )

    def test_rejects_a_robot_whose_role_is_missing(self):
        document = _robots()
        document["robots"][0]["role"] = "pilot"
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_an_inchworm_without_a_gait(self):
        document = _robots()
        document["robots"][0]["locomotion"] = "inchworm"
        document["robots"][0]["depot"] = {
            "feet": [
                {"cell": [1, 1, 1], "normal": [1, 0, 0]},
                {"cell": [1, 0, 1], "normal": [1, 0, 0]},
            ]
        }
        document["robots"][0]["start"] = {
            "feet": [
                {"cell": [1, 1, 1], "normal": [1, 0, 0]},
                {"cell": [1, 1, 0], "normal": [1, 0, 0]},
            ]
        }
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_cell_robot_whose_pose_is_a_stance(self):
        document = _robots()
        document["robots"][0]["depot"] = {
            "feet": [
                {"cell": [1, 1, 1], "normal": [1, 0, 0]},
                {"cell": [1, 0, 1], "normal": [1, 0, 0]},
            ]
        }
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_three_role_schedule_missing_fasten(self):
        document = _robots()
        document["robots"] = []
        document["schedule"] = "three-role"
        document["roles"] = {
            "cargo": {"description": "carry"},
            "crane": {"description": "place"},
        }
        with self.assertRaises(FormatError):
            validate(document)

    def test_rejects_a_measured_fault_rate_source(self):
        document = _robots()
        document["faults"] = {"seed": 0, "rate_source": "measured"}
        with self.assertRaises(FormatError):
            validate(document)


class RoundTrip(unittest.TestCase):
    def test_minimal_blueprint_round_trips_through_a_world(self):
        document = _minimal()
        world = world_from_blueprint(document)
        self.assertEqual(world.blueprint_count, 1)
        self.assertEqual(int(world.occupancy[1, 1, 1]), 0)
        again = blueprint_from_world(
            world,
            ident=document["id"],
            lattice=document["lattice"],
            model=document["model"],
        )
        self.assertEqual(again["cells"], document["cells"])
        self.assertEqual(again["seed"], [])
        self.assertEqual(again["supports"], [])

    def test_seed_and_scaffold_round_trip_as_occupancy(self):
        document = make_blueprint(
            ident="supported",
            lattice=cubic_lattice(4, 4, 4),
            cells=[
                ((1, 1, 1), "seed"),
                ((2, 1, 1), "structural"),
                ((1, 1, 2), "scaffold"),
            ],
        )
        world = world_from_blueprint(document)
        self.assertEqual(int(world.occupancy[1, 1, 1]), VOXEL)
        self.assertEqual(int(world.occupancy[1, 1, 2]), SCAFFOLD)
        self.assertTrue(bool(world.blueprint[2, 1, 1]))
        self.assertFalse(bool(world.blueprint[1, 1, 1]))
        again = blueprint_from_world(
            world,
            ident="supported",
            lattice=document["lattice"],
            model=document["model"],
        )
        self.assertEqual(again, document)

    def test_square_wall_round_trips(self):
        document = make_blueprint(
            ident="flat",
            lattice=square_lattice(5, 5),
            cells=[((3, 2), "structural"), ((3, 1), "seed")],
        )
        world = world_from_blueprint(document)
        self.assertEqual(world.occupancy.shape, (5, 5))
        self.assertEqual(int(world.occupancy[3, 1]), VOXEL)
        again = blueprint_from_world(
            world, ident="flat", lattice=document["lattice"], model=document["model"]
        )
        self.assertEqual(again["cells"], document["cells"])

    def test_event_log_round_trips_through_a_file(self):
        document = make_event_log(
            scenario_id="min",
            clock="tick",
            makespan=4,
            events=[
                {
                    "t": 1,
                    "type": "placement",
                    "cell": [1, 1, 1],
                    "robot": "r0",
                    "voxel_type": "structural",
                },
                {
                    "t": 2,
                    "type": "fault",
                    "fault": "fastener",
                    "cell": [1, 1, 1],
                    "robot": "r0",
                    "cascade": False,
                },
                {
                    "t": 3,
                    "type": "recovery",
                    "action": "re-place",
                    "fault": "fastener",
                    "cell": [1, 1, 1],
                },
                {
                    "t": 0,
                    "type": "gate",
                    "gate": "stability",
                    "verdict": "refuse",
                    "reason": "cantilever",
                },
            ],
        )
        with tempfile.TemporaryDirectory() as tmp:
            path = dump(document, Path(tmp) / "log.json")
            loaded = load(path)
        self.assertEqual(loaded, document)
        self.assertEqual(canonical(loaded), canonical(document))

    def test_dump_refuses_an_invalid_document(self):
        with tempfile.TemporaryDirectory() as tmp:
            with self.assertRaises(FormatError):
                dump({"kind": "blueprint"}, Path(tmp) / "no.json")

    def test_unfitted_note_is_on_the_minimal_model(self):
        self.assertIn("not a fit", _minimal()["model"]["note"])
        self.assertIn("not a fit", UNFITTED_NOTE)
        self.assertFalse(_minimal()["model"]["fitted"])
        self.assertEqual(_minimal()["model"]["k"], 1.0)


class CatalogFiles(unittest.TestCase):
    def test_committed_tasks_match_the_generator(self):
        root = Path("benchmarks/tasks")
        for ident, document in documents().items():
            path = root / f"{ident}.json"
            self.assertTrue(path.is_file(), ident)
            self.assertEqual(path.read_text(encoding="utf-8"), canonical(document))

    def test_suite_manifest_lists_every_task_in_order(self):
        suite = json.loads(Path("benchmarks/suite.json").read_text(encoding="utf-8"))
        self.assertEqual(suite["format_version"], FORMAT_VERSION)
        self.assertEqual(suite["tasks"], list(SUITE))
        self.assertIn("not a fit", suite["description"])

    def test_every_public_model_is_unfitted_k_1(self):
        for ident, document in documents().items():
            model = document["blueprint"]["model"]
            self.assertFalse(model["fitted"], ident)
            self.assertEqual(model["k"], 1.0, ident)
            self.assertEqual(model["mass"], 1.0, ident)
            self.assertEqual(model["g"], 1.0, ident)
            self.assertEqual(model["displacement_units"], "mass*g/k", ident)
            self.assertIn("not a fit", model["note"], ident)
            self.assertEqual(document["blueprint"]["lattice"]["pitch"], 1.0, ident)

    def test_shelter_is_256_cells_with_a_five_voxel_seed(self):
        document = documents()["armadas-shelter"]
        cells = document["blueprint"]["cells"]
        self.assertEqual(len(cells), 256)
        self.assertEqual(len(document["blueprint"]["seed"]), 5)
        self.assertEqual(document["blueprint"]["supports"], [])
        self.assertEqual(document["robots"]["schedule"], "three-role")
        self.assertEqual(document["robots"]["faults"] if "faults" in document["robots"] else None, None)
        self.assertIn("304.8", document["blueprint"]["meta"]["note"])
        self.assertIn("not a fit", document["blueprint"]["meta"]["note"])

    def test_inchworm_scaffold_starts_on_the_pad_and_has_no_supports(self):
        document = documents()["scaffold-span-4-inchworm"]
        self.assertEqual(document["blueprint"]["supports"], [])
        self.assertGreater(len(document["blueprint"]["seed"]), 0)
        robot = document["robots"]["robots"][0]
        self.assertEqual(robot["locomotion"], "inchworm")
        self.assertEqual(robot["gait"]["stride"], 1)
        self.assertEqual(robot["gait"]["span"], 2.0)
        for foot in robot["depot"]["feet"]:
            self.assertIn(foot["cell"], document["blueprint"]["seed"])

    def test_bridge_spans_4_through_8_are_present(self):
        for span in range(4, 9):
            document = documents()[f"bridge-span-{span}"]
            self.assertEqual(document["task"]["span"], span)
            self.assertEqual(document["family"], "bridge")
            self.assertEqual(document["task"]["scaffold"], "none")

    def test_scenario_bundle_validates_as_one_document(self):
        document = make_scenario(
            ident="min",
            title="Minimal",
            family="wall",
            blueprint=_minimal(),
            robots=_robots(),
            task={"runner": "swarm", "gate": "off", "scaffold": "none", "n_robots": 1},
            notes="k = 1 is not a fit.",
        )
        self.assertEqual(document["kind"], "scenario")
        validate(document)


def _add_round_trips() -> None:
    def make(ident: str):
        def test(self):
            document = documents()[ident]
            world = world_from_blueprint(document["blueprint"])
            again = blueprint_from_world(
                world,
                ident=document["blueprint"]["id"],
                lattice=document["blueprint"]["lattice"],
                title=document["blueprint"].get("title", ""),
                model=document["blueprint"]["model"],
                meta=document["blueprint"].get("meta"),
            )
            self.assertEqual(again["cells"], document["blueprint"]["cells"])
            self.assertEqual(again["seed"], document["blueprint"]["seed"])
            self.assertEqual(again["supports"], document["blueprint"]["supports"])
            self.assertEqual(again["model"]["k"], 1.0)

        test.__name__ = f"test_round_trip_{ident.replace('-', '_')}"
        return test

    for ident in SUITE:
        setattr(RoundTrip, f"test_round_trip_{ident.replace('-', '_')}", make(ident))


_add_round_trips()


class CopyDoesNotMutate(unittest.TestCase):
    def test_deepcopy_then_edit_does_not_change_the_catalog(self):
        original = documents()["tower"]
        edited = copy.deepcopy(original)
        edited["blueprint"]["model"]["k"] = 4
        self.assertEqual(documents()["tower"]["blueprint"]["model"]["k"], 1.0)
        self.assertEqual(original["blueprint"]["model"]["k"], 1.0)


if __name__ == "__main__":
    unittest.main()
