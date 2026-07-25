# Copyright (c) 2026 Joshua Dyar. MIT License (see LICENSE).
"""The examples/ directory is a public promise — keep it green.

Each example is run as a subprocess from the repo root (exactly how a
researcher runs it). A failing example is a failing build: these files
are the adoption path, not decoration.
"""

import subprocess
import sys
import unittest
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent


def run_example(name: str) -> subprocess.CompletedProcess:
    return subprocess.run(
        [sys.executable, str(REPO_ROOT / "examples" / name)],
        cwd=REPO_ROOT,
        capture_output=True,
        text=True,
        timeout=600,
    )


class TestExamples(unittest.TestCase):
    def test_hex_lattice_example(self):
        result = run_example("hex_lattice.py")
        self.assertEqual(result.returncode, 0, result.stderr[-2000:])
        self.assertIn("UNCHANGED", result.stdout)

    def test_my_robot_example(self):
        result = run_example("my_robot.py")
        self.assertEqual(result.returncode, 0, result.stderr[-2000:])
        self.assertIn("coupled pair", result.stdout)


if __name__ == "__main__":
    unittest.main()
