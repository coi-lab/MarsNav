"""Compare direct and science-stop routes across three Jezero windows."""

import csv
import json

from run_experiment import ROOT, main, read_elevation
from terrain import slope_degrees
from matplotlib import pyplot as plt


SITES = {
    "original": "jezero-c-crop.tif",
    "flat": "jezero-c-flat.tif",
    "steep": "jezero-c-steep.tif",
}
SCENARIOS = {
    "direct": [],
    "science_stops": ["--via", "90.5", "65.5", "--via", "205.5", "235.5"],
}
METRICS = (
    "grid_spacing_m",
    "hazard_area_pct",
    "missed_native_hazard_pct",
    "route_length_m",
    "route_over_native_hazards_m",
    "max_separation_from_native_route_m",
    "failed_leg",
)


def save_overview(output, reports):
    fig, axes = plt.subplots(len(SITES), len(SCENARIOS), figsize=(8, 11), layout="constrained")
    for row, (site, filename) in enumerate(SITES.items()):
        elevation, _ = read_elevation(ROOT / "data" / filename, None)
        slope = slope_degrees(elevation, 1)
        height, width = slope.shape
        for column, scenario in enumerate(SCENARIOS):
            report = reports[site, scenario]
            ax = axes[row, column]
            image = ax.imshow(slope, extent=(0, width, height, 0), cmap="Greys",
                              vmin=0, vmax=report["slope_limit_degrees"])
            for run, color, style in ((report["runs"][0], "#005f9e", "-"),
                                      (report["runs"][-1], "#c43a06", "--")):
                if run["route_xy_m"]:
                    ax.plot(*zip(*run["route_xy_m"]), color=color, linestyle=style,
                            label=f'{run["grid_spacing_m"]} m route', linewidth=1.8)
            for marker, stops in (("o", [report["start_xy_m"]]), ("X", [report["goal_xy_m"]]),
                                  ("D", report["science_stops_xy_m"])):
                for x, y in stops:
                    ax.scatter(x, y, marker=marker, color="#ffe46a", edgecolors="black",
                               s=28, zorder=3)
            gap = report["runs"][-1]["max_separation_from_native_route_m"]
            label = "Direct" if scenario == "direct" else "Two science stops"
            gap_label = "Incomplete route pair" if gap is None else f"Maximum route gap {gap:.1f} m"
            ax.set(title=f"{site.title()} | {label}\n{gap_label}",
                   xticks=[0, 100, 200, 300], yticks=[0, 100, 200, 300])
            ax.tick_params(labelbottom=row == len(SITES) - 1, labelleft=column == 0)
    axes[0, 0].legend(loc="lower left", fontsize=8)
    fig.suptitle("Jezero Crater routes on the original 1 m slope grid")
    fig.supxlabel("East from window edge (m)")
    fig.supylabel("South from window edge (m)")
    fig.colorbar(image, ax=axes, shrink=0.6, extend="max", label="1 m reference slope (°)")
    fig.savefig(output / "route_overview.png", dpi=155)
    plt.close(fig)


def run_missions(output):
    rows = []
    reports = {}
    for site, filename in SITES.items():
        for scenario, stops in SCENARIOS.items():
            folder = output / site / scenario
            print(f"\n{site} | {scenario}")
            main(["--dem", str(ROOT / "data" / filename), "--output", str(folder), *stops])
            report = json.loads((folder / "comparison.json").read_text(encoding="utf-8"))
            reports[site, scenario] = report
            rows.extend({"site": site, "scenario": scenario, **{k: run[k] for k in METRICS}}
                        for run in report["runs"])
    summary = output / "summary.csv"
    with summary.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=("site", "scenario", *METRICS))
        writer.writeheader()
        writer.writerows(rows)
    save_overview(output, reports)
    print(f"\nSummary written to {summary}")


if __name__ == "__main__":
    run_missions(ROOT / "results" / "missions")
