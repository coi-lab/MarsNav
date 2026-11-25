"""Check the terrain math, planner, and real Jezero experiment end to end."""

import json
import tempfile
import unittest
from math import sqrt
from pathlib import Path

import numpy as np

from run_experiment import ROOT, compare, main, plan_legs
from terrain import astar, block_mean, route_metrics, route_separation, slope_degrees


class TerrainChecks(unittest.TestCase):
    def test_plane_has_same_slope_after_aggregation(self):
        plane = (np.indices((20, 20))[0] + np.indices((20, 20))[1]) / sqrt(2)
        np.testing.assert_allclose(slope_degrees(plane, 1), 45)
        np.testing.assert_allclose(slope_degrees(block_mean(plane, 5), 5), 45)

    def test_planner_uses_shortest_detour_around_blocked_cell(self):
        slope = np.zeros((3, 3))
        slope[1, 1] = 20
        path, cost = astar(slope, 1, (1, 0), (1, 2), limit=20, weight=2)
        self.assertNotIn((1, 1), path)
        self.assertEqual(cost, 4)
        slope[:, 1] = 20
        self.assertIsNone(astar(slope, 1, (1, 0), (1, 2), limit=20, weight=2))

    def test_slope_penalty_changes_route(self):
        slope = np.zeros((3, 3))
        slope[1, 1] = 19
        unweighted, _ = astar(slope, 1, (1, 0), (1, 2), limit=20, weight=0)
        weighted, _ = astar(slope, 1, (1, 0), (1, 2), limit=20, weight=4)
        self.assertIn((1, 1), unweighted)
        self.assertNotIn((1, 1), weighted)

    def test_reference_audit_measures_distance_inside_a_hazard(self):
        hazards = np.zeros((3, 3), dtype=bool)
        hazards[1, 1] = True
        length, hazardous_length = route_metrics([(0.5, 0.5), (2.5, 2.5)], hazards, 1)
        self.assertAlmostEqual(length, 2 * sqrt(2))
        self.assertAlmostEqual(hazardous_length, sqrt(2))

    def test_required_stops_are_visited_in_order(self):
        slope = np.zeros((5, 5))
        stops = [(0.5, 0.5), (0.8, 4.3), (4.2, 4.5), (4.5, 0.5)]
        points, _, failed_leg = plan_legs(slope, 1, stops, 20, 2)
        self.assertIsNone(failed_leg)
        positions = [points.index(stop) for stop in stops]
        self.assertTrue(all(left < right for left, right in zip(positions, positions[1:])))
        slope[4, 4] = 20
        self.assertEqual(plan_legs(slope, 1, stops, 20, 2), ([], None, 2))

    def test_route_separation_measures_deviation_from_line(self):
        direct = [(0, 0), (8, 0)]
        detour = [(0, 0), (4, 3), (8, 0)]
        self.assertEqual(route_separation(direct, detour), 3)
        self.assertEqual(route_separation(direct, direct), 0)

    def test_keep_in_blocks_an_unusable_coarse_cell(self):
        elevation = np.zeros((20, 20))
        allowed = np.zeros_like(elevation, dtype=bool)
        allowed[8:12, :] = True
        results = compare(elevation, (1.5, 9.5), (18.5, 9.5), 20, 2, corridor=allowed)
        runs = {report["grid_spacing_m"]: report for _, report in results}
        self.assertEqual(runs[1]["route_outside_keep_in_m"], 0)
        self.assertEqual(runs[2]["route_outside_keep_in_m"], 0)
        self.assertEqual(runs[5]["failed_leg"], 1)
        self.assertEqual(runs[10]["failed_leg"], 1)
        control = compare(elevation, (1.5, 9.5), (18.5, 9.5), 20, 2,
                          corridor=allowed, fixed_planning_grid=True)
        for _, report in control:
            self.assertEqual(report["planner_spacing_m"], 1)
            self.assertEqual(report["route_outside_keep_in_m"], 0)
            self.assertEqual(report["route_length_m"], 17)
            self.assertEqual(report["grid_center_path_length_m"], 17)
            self.assertEqual(report["waypoint_connector_length_m"], 0)

    def test_report_uses_the_real_crop_and_shared_endpoints(self):
        with tempfile.TemporaryDirectory() as directory:
            main(["--dem", str(ROOT / "data/jezero-c-crop.tif"), "--output", directory])
            output = Path(directory)
            report = json.loads((output / "comparison.json").read_text(encoding="utf-8"))
            with (output / "routes.png").open("rb") as map_file:
                self.assertEqual(map_file.read(8), b"\x89PNG\r\n\x1a\n")
            self.assertEqual(report["crop_shape_rows_cols"], [300, 300])
            self.assertEqual(report["source"]["source_tags"]["source_window_col_row_width_height"],
                             "3380,6672,300,300")
            self.assertEqual([run["grid_spacing_m"] for run in report["runs"]], [1, 2, 5, 10])
            for run in report["runs"]:
                self.assertEqual(run["route_xy_m"][0], report["start_xy_m"])
                self.assertEqual(run["route_xy_m"][-1], report["goal_xy_m"])
                self.assertGreaterEqual(run["route_length_m"], 240)
                self.assertAlmostEqual(run["grid_center_path_length_m"]
                                       + run["waypoint_connector_length_m"], run["route_length_m"])
            native = report["runs"][0]
            self.assertGreater(native["hazard_area_pct"], 0)
            self.assertEqual(native["missed_native_hazard_pct"], 0)
            self.assertEqual(native["route_over_native_hazards_m"], 0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
