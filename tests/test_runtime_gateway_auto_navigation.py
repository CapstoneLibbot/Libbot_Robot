from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from runtime_gateway import RobotRuntimeGateway


class RuntimeGatewayAutoNavigationTests(unittest.TestCase):
    def test_saved_map_grid_is_validated_and_normalized(self):
        grid = RobotRuntimeGateway._grid_from_saved_map({
            "width": 2,
            "height": 2,
            "data": {
                "format": "occupancy-grid-v1",
                "resolution": 0.05,
                "origin": [-1.0, 2.0, 0.25],
                "timestamp": "2026-10-24T00:00:00Z",
                "cells": [0, 100, -1, 50],
            },
        })

        self.assertEqual(grid["width"], 2)
        self.assertEqual(grid["height"], 2)
        self.assertEqual(grid["resolution"], 0.05)
        self.assertEqual(grid["origin_x"], -1.0)
        self.assertEqual(grid["origin_y"], 2.0)
        self.assertEqual(grid["origin_yaw"], 0.25)
        self.assertEqual(grid["data"], [0, 100, -1, 50])

    def test_saved_map_grid_rejects_invalid_cell_count(self):
        with self.assertRaises(ValueError):
            RobotRuntimeGateway._grid_from_saved_map({
                "width": 2,
                "height": 2,
                "data": {
                    "format": "occupancy-grid-v1",
                    "resolution": 0.05,
                    "origin": [0.0, 0.0, 0.0],
                    "cells": [0],
                },
            })

    def test_grid_writer_preserves_map_metadata_and_flips_rows_for_pgm(self):
        grid = {
            "width": 2,
            "height": 2,
            "resolution": 0.05,
            "origin_x": -1.0,
            "origin_y": 2.0,
            "origin_yaw": 0.25,
            "data": [0, 100, -1, 50],
        }
        with tempfile.TemporaryDirectory() as directory:
            yaml_path, pgm_path = RobotRuntimeGateway._write_grid_files(
                grid, Path(directory) / "backend_test"
            )

            yaml_text = yaml_path.read_text(encoding="utf-8")
            self.assertIn("resolution: 0.05", yaml_text)
            self.assertIn("origin: [-1.0, 2.0, 0.25]", yaml_text)
            pgm = pgm_path.read_bytes()
            self.assertTrue(pgm.startswith(b"P5\n2 2\n255\n"))
            self.assertEqual(list(pgm[-4:]), [205, 127, 254, 0])


if __name__ == "__main__":
    unittest.main()
