"""Run the Jezero resolution comparison and write a report and route map."""

import argparse
import json
from math import dist
from pathlib import Path

import matplotlib
import numpy as np
import rasterio
from rasterio.features import rasterize
from rasterio.windows import Window

from terrain import astar, block_mean, route_metrics, route_separation, slope_degrees

matplotlib.use("Agg")
from matplotlib import pyplot as plt


FACTORS = (1, 2, 5, 10)
ROOT = Path(__file__).resolve().parent


def read_elevation(path, window):
    with rasterio.open(path) as source:
        if window is None:
            window = Window(0, 0, source.width, source.height)
        x, y, width, height = window.flatten()
        if min(x, y) < 0 or x + width > source.width or y + height > source.height:
            raise ValueError("The requested crop must lie inside the source raster.")
        if min(width, height) < 20 or width % 10 or height % 10:
            raise ValueError("Crop width and height must be multiples of 10 and at least 20 pixels.")
        if (not source.crs or not source.crs.is_projected
                or source.crs.linear_units_factor[1] != 1.0):
            raise ValueError("Use a projected elevation raster with horizontal units in meters.")
        transform = source.window_transform(window)
        if not np.allclose((transform.a, transform.b, transform.d, transform.e), (1, 0, 0, -1)):
            raise ValueError("Use an unrotated, north-up raster with 1 m square cells.")
        elevation = source.read(1, window=window, masked=True).astype(float).filled(np.nan)
        metadata = {
            "file": path.name,
            "crs": source.crs.to_string(),
            "transform": list(transform)[:6],
            "window_col_row_width_height": list(window.flatten()),
            "source_tags": source.tags(),
        }
    if not np.isfinite(elevation).all():
        raise ValueError("The crop contains missing elevations. Choose a fully covered window.")
    return elevation, metadata


def plan_legs(slope, spacing, stops, limit, weight):
    points = [stops[0]]
    total_cost = 0.0
    for leg, (start, goal) in enumerate(zip(stops, stops[1:]), 1):
        start_cell = (int(start[1] // spacing), int(start[0] // spacing))
        goal_cell = (int(goal[1] // spacing), int(goal[0] // spacing))
        solution = astar(slope, spacing, start_cell, goal_cell, limit, weight)
        if solution is None:
            return [], None, leg
        cells, cost = solution
        centers = [((col + 0.5) * spacing, (row + 0.5) * spacing) for row, col in cells]
        points.extend([*centers, goal])
        total_cost += cost
    return points, float(total_cost), None


def compare(elevation, start, goal, limit, weight, waypoints=(), corridor=None,
            fixed_planning_grid=False):
    native_slope = slope_degrees(elevation, 1)
    native_hazards = native_slope >= limit
    stops = [start, *waypoints, goal]
    rows, cols = elevation.shape
    if any(not (0 <= x < cols and 0 <= y < rows) for x, y in stops):
        raise ValueError("Start, goal, and science stops must lie inside the crop.")
    experiments = []
    native_route = []
    for spacing in FACTORS:
        slope = native_slope if spacing == 1 else slope_degrees(block_mean(elevation, spacing), spacing)
        hazards = slope >= limit
        upsampled = hazards.repeat(spacing, axis=0).repeat(spacing, axis=1)
        planner_spacing = 1 if fixed_planning_grid else spacing
        planning_slope = slope
        if fixed_planning_grid and spacing > 1:
            planning_slope = slope.repeat(spacing, axis=0).repeat(spacing, axis=1)
        if corridor is not None:
            allowed = block_mean(corridor, planner_spacing) == 1
            planning_slope = np.where(allowed, planning_slope, np.inf)
        points, cost, failed_leg = plan_legs(planning_slope, planner_spacing, stops, limit, weight)
        if spacing == 1:
            native_route = points
        result = {
            "grid_spacing_m": spacing,
            "planner_spacing_m": planner_spacing,
            "hazard_area_pct": 100 * float(hazards.mean()),
            "missed_native_hazard_pct": (
                100 * int(np.count_nonzero(native_hazards & ~upsampled))
                / int(np.count_nonzero(native_hazards)) if native_hazards.any() else None
            ),
            "route_length_m": None,
            "grid_center_path_length_m": None,
            "waypoint_connector_length_m": None,
            "route_over_native_hazards_m": None,
            "grid_route_cost": cost,
            "route_xy_m": points,
            "failed_leg": failed_leg,
            "max_separation_from_native_route_m": None,
        }
        if corridor is not None:
            result.update(
                keep_in_native_hazard_cells=int(np.count_nonzero(native_hazards & corridor)),
                keep_in_flagged_cells_on_native_grid=int(np.count_nonzero(upsampled & corridor)),
                keep_in_missed_native_hazard_cells=int(
                    np.count_nonzero(native_hazards & ~upsampled & corridor)
                ),
            )
        if points:
            length, hazard_length = route_metrics(points, native_hazards, 1)
            offsets = []
            for point in stops:
                center = tuple((int(axis // planner_spacing) + 0.5) * planner_spacing
                               for axis in point)
                offsets.append(dist(point, center))
            connectors = offsets[0] + 2 * sum(offsets[1:-1]) + offsets[-1]
            result.update(
                route_length_m=length,
                route_over_native_hazards_m=hazard_length,
                grid_center_path_length_m=max(0.0, length - connectors),
                waypoint_connector_length_m=connectors,
            )
            if corridor is not None:
                _, outside = route_metrics(points, ~corridor, 1)
                result["route_outside_keep_in_m"] = outside
            if native_route:
                result["max_separation_from_native_route_m"] = (
                    0.0 if spacing == 1 else route_separation(points, native_route)
                )
        experiments.append((slope, result))
    return experiments


def save_map(experiments, width, height, start, goal, limit, output, waypoints=(), polygons=()):
    fig, axes = plt.subplots(2, 2, figsize=(9, 8), layout="constrained")
    for ax, (slope, result) in zip(axes.flat, experiments):
        image = ax.imshow(slope, origin="upper", extent=(0, width, height, 0),
                          vmin=0, vmax=limit, cmap="magma", interpolation="nearest")
        for polygon in polygons:
            ax.plot(*zip(*polygon), color="#80ea9e", linewidth=0.8, alpha=0.75)
        points = result["route_xy_m"]
        if points:
            ax.plot(*zip(*points), color="#54e7ff", linewidth=1.5)
            subtitle = f'{result["route_length_m"]:.1f} m route'
        else:
            subtitle = f'No route on leg {result["failed_leg"]}'
        ax.scatter(*start, c="#54e7ff", s=25, marker="o", edgecolors="black", zorder=3)
        ax.scatter(*goal, c="white", s=34, marker="X", edgecolors="black", zorder=3)
        for number, (x, y) in enumerate(waypoints, 1):
            ax.scatter(x, y, c="#ffe46a", s=65 if polygons else 30, marker="D",
                       edgecolors="black", zorder=3)
            if polygons:
                ax.annotate(str(number), (x, y), ha="center", va="center",
                            fontsize=6, color="black", zorder=4)
            else:
                ax.annotate(f"S{number}", (x, y), xytext=(5, 5), textcoords="offset points",
                            color="black", bbox={"facecolor": "#ffe46a", "edgecolor": "none", "pad": 1})
        ax.set(title=f'{result["grid_spacing_m"]} m grid · {subtitle}',
               xlabel="East from crop edge (m)", ylabel="South from crop edge (m)")
    title = "Jezero Crater slope grids and A* routes"
    if experiments[-1][1]["planner_spacing_m"] == 1:
        title += "\nA* held on a 1 m grid"
    fig.suptitle(title)
    fig.colorbar(image, ax=axes, label="Slope angle (°)", extend="max", shrink=0.85)
    fig.savefig(output, dpi=170)
    plt.close(fig)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dem", type=Path, default=ROOT / "data/jezero-c-crop.tif")
    parser.add_argument("--window", type=int, nargs=4, metavar=("COL", "ROW", "WIDTH", "HEIGHT"))
    parser.add_argument("--start", type=float, nargs=2, default=(30.5, 150.5), metavar=("X", "Y"))
    parser.add_argument("--goal", type=float, nargs=2, default=(270.5, 150.5), metavar=("X", "Y"))
    parser.add_argument("--via", type=float, nargs=2, action="append", default=[], metavar=("X", "Y"),
                        help="Required waypoint. Repeat to specify waypoints in order.")
    parser.add_argument("--keep-in", type=Path,
                        help="JSON file with closed polygon rings in crop-relative meters.")
    parser.add_argument("--fixed-planning-grid", action="store_true",
                        help="Coarsen terrain only. Plan each route on the original 1 m grid.")
    parser.add_argument("--limit", type=float, default=20, help="Blocked slope angle in degrees.")
    parser.add_argument("--weight", type=float, default=2, help="Penalty for traveling on slopes.")
    parser.add_argument("--output", type=Path, default=ROOT / "results")
    args = parser.parse_args(argv)
    if not (0 < args.limit < 90 and np.isfinite(args.weight) and args.weight >= 0):
        parser.error("Use a slope limit between 0 and 90 degrees and a finite nonnegative weight.")

    window = Window(*args.window) if args.window else None
    try:
        elevation, source = read_elevation(args.dem, window)
        start, goal = tuple(args.start), tuple(args.goal)
        waypoints = [tuple(stop) for stop in args.via]
        corridor_data = None
        polygons = []
        allowed = None
        if args.keep_in:
            corridor_data = json.loads(args.keep_in.read_text(encoding="utf-8"))
            polygons = corridor_data["polygons_xy_m"]
            if not polygons or any(len(ring) < 4 or ring[0] != ring[-1] for ring in polygons):
                raise ValueError("Keep-in data must contain one or more closed polygon rings.")
            allowed = rasterize(
                [({"type": "Polygon", "coordinates": [ring]}, 1) for ring in polygons],
                out_shape=elevation.shape, dtype="uint8",
            ).astype(bool)
            if not allowed.any():
                raise ValueError("The keep-in polygons do not intersect this crop.")
        experiments = compare(elevation, start, goal, args.limit, args.weight, waypoints, allowed,
                              fixed_planning_grid=args.fixed_planning_grid)
    except (ValueError, KeyError, OSError, rasterio.errors.RasterioIOError) as error:
        parser.error(str(error))

    args.output.mkdir(parents=True, exist_ok=True)
    rows, cols = elevation.shape
    report = {
        "source": source,
        "crop_shape_rows_cols": [rows, cols],
        "start_xy_m": start,
        "goal_xy_m": goal,
        "science_stops_xy_m": waypoints,
        "straight_waypoint_chain_length_m": sum(
            dist(a, b) for a, b in zip([start, *waypoints], [*waypoints, goal])
        ),
        "fixed_planning_grid": args.fixed_planning_grid,
        "keep_in": corridor_data,
        "slope_limit_degrees": args.limit,
        "slope_weight": args.weight,
        "runs": [result for _, result in experiments],
    }
    (args.output / "comparison.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    save_map(experiments, cols, rows, start, goal, args.limit, args.output / "routes.png",
             waypoints, polygons)
    for _, result in experiments:
        distance = result["route_length_m"]
        outcome = f'no route on leg {result["failed_leg"]}'
        if distance is not None:
            outcome = f'{distance:.1f} m route, {result["route_over_native_hazards_m"]:.2f} m over reference hazards'
        print(f'{result["grid_spacing_m"]:2} m grid | {result["hazard_area_pct"]:.3f}% flagged | {outcome}')


if __name__ == "__main__":
    main()
