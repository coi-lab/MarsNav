"""Compare raster resolutions on the approximate Dec. 10 keep-in corridor."""

import json

from run_experiment import ROOT, main


if __name__ == "__main__":
    config = ROOT / "data" / "sol1709-corridor.json"
    settings = json.loads(config.read_text(encoding="utf-8"))
    args = ["--dem", str(ROOT / "data" / "jezero-sol1709.tif"),
            "--keep-in", str(config)]
    args.extend(["--start", *map(str, settings["start_xy_m"])])
    args.extend(["--goal", *map(str, settings["goal_xy_m"])])
    for waypoint in settings["navigation_waypoints_xy_m"]:
        args.extend(["--via", *map(str, waypoint)])
    output = ROOT / "results" / "sol1709"
    main([*args, "--output", str(output)])
    print("\nCoarsen terrain only, with A* held at 1 m")
    main([*args, "--fixed-planning-grid", "--output", str(output / "fixed_grid")])
