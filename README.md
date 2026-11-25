# Jezero Crater rover route planning

This class proof of concept tests how coarsening an orbital elevation map changes slope-based hazard detection and A* routes in Jezero Crater. It runs the same experiment at 1, 2, 5, and 10 meters per grid cell.

## Data and provenance

The input is the **Jezero C edited 1 m digital terrain model**, derived from the HiRISE stereo images ESP_046060_1985 and ESP_045994_1985. It comes from the USGS Mars 2020 Terrain Relative Navigation dataset. The band contains elevation in meters relative to the MOLA geoid. [USGS dataset description](https://astrogeology.usgs.gov/search/map/mars_2020_terrain_relative_navigation_hirise_dtm_mosaic) · [Original GeoTIFF](https://planetarymaps.usgs.gov/mosaic/mars2020_trn/HiRISE/DTM_MOLAtopography_DeltaGeoid_Jezero_C_Edited_affine_1m_Eqc_latTs0_lon0.tif)

The included `data/jezero-c-crop.tif` copies a 300 × 300 pixel window at original column **3380** and row **6672** (zero-based). Elevations, georeferencing, and the Mars 2000 Sphere equirectangular projection are preserved. The crop was chosen from a small set of fully covered windows to demonstrate terrain with some steep cells. It represents one site in Jezero. The included crop runs offline. The original file SHA-256 is `0c681fa0c2fa72ec2253d09ed8643d5794ea0c3575f5fa953d470cc0beeb1751`.

The proof of concept uses the orbital DTM as its remote sensing input. NASA's recorded Perseverance traverse can provide geographic context for a later comparison once its overlap with the chosen DTM is checked. [NASA traverse map](https://science.nasa.gov/mission/mars-2020-perseverance/location-map/)

## From elevation to route

1. Keep the original 1 m grid. Average adjacent 2 × 2, 5 × 5, and 10 × 10 blocks of elevations to simulate lower spatial resolution across the same footprint.
2. On each elevation grid, subtract neighboring elevations and divide by their horizontal separation. For an interior cell, the east–west derivative is `(z_east - z_west) / (2 * spacing)`. Compute the equivalent north–south derivative and combine them as `slope_deg = degrees(atan(hypot(dz_dx, dz_dy)))`. Grid edges use the single available neighbor per axis. The calculation uses the product's projected meter spacing, so slopes and distances inherit projection distortion. At Jezero's latitude, map east–west distances are about 5% larger than surface distances.
3. A* searches the eight neighboring cells, prohibits diagonal corner cuts through blocked cells, and blocks slopes at or above **20°** by default. Each move costs `distance * (1 + weight * mean_endpoint_slope / limit)` with default weight **2**. Euclidean distance is a lower bound for remaining cost. The cheapest route balances short distance with low slope under these model settings.
4. Each grid gets the same projected endpoints. A* searches between the centers of the containing cells. The exported line connects those centers to the exact endpoints so start and goal match in the figure. Total plotted length includes the two endpoint connectors. The reported grid cost covers A* moves.
5. Compare the percentage of area flagged on each grid, the percentage of native hazard cells missed by coarser grids, route length, and meters traveled through cells flagged by the 1 m grid. The 1 m result is the best-resolution orbital reference available here. The model covers slope-based hazards. It leaves out rocks, sand, rover dynamics, and onboard sensing.

## Run the experiment and checks

PowerShell with Python 3.14, the version used for this run.

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe check.py
.\.venv\Scripts\python.exe run_experiment.py
```

`check.py` verifies the slope calculation on a known plane, the planner on known paths, the native hazard crossing calculation, and the full experiment on real Jezero elevation. The run saves the graphic to `results/routes.png` and records the source metadata, settings, metrics, and route coordinates in `results/comparison.json`.

Try a different slope threshold or endpoint by passing `--limit 15` or `--start 30.5 120.5 --goal 270.5 180.5`. Coordinates are projected meters east and south from the crop's upper-left corner. To reproduce the run directly from the original file, use `--dem data/jezero-c-full.tif --window 3380 6672 300 300`.

## Multiple sites and required science stops

```powershell
.\.venv\Scripts\python.exe run_missions.py
```

The batch compares the original 300 m window with two nonoverlapping windows cut from the same USGS Jezero C DTM. The additional windows were chosen from a preview grid for contrasting native slope coverage before running the routes.

| Window | Source column and row | Approximate window center | Native area at or above 20° |
|---|---|---|---:|
| Original | 3380, 6672 | 18.4355° N, 77.4422° E | 0.288% |
| Flat | 4100, 3450 | 18.4899° N, 77.4543° E | 0.000% |
| Steep | 2900, 10450 | 18.3718° N, 77.4341° E | 4.086% |

Each window runs two paired scenarios. The direct route goes from `(30.5, 150.5)` to `(270.5, 150.5)`. The science-stop route requires the same endpoints plus stops at `(90.5, 65.5)` and `(205.5, 235.5)` in that order. These are hypothetical observation sites at fixed crop-relative coordinates. NASA uses navigation waypoints and chooses science locations during the real mission. The cases here use the elevation model and hypothetical targets. [NASA annotated Dec. 10 drive](https://science.nasa.gov/photojournal/mapping-perseverances-route-with-ai/) · [How Perseverance drives](https://science.nasa.gov/resource/how-perseverance-drives-on-mars/)

A* plans each leg in sequence on a given grid. The exported route passes through the exact stops and the report identifies the first failed leg if any stop is unreachable. To specify your own stops in a single run, add `--via 90.5 65.5 --via 205.5 235.5` to `run_experiment.py`.

The batch writes `results/missions/summary.csv`, an overlay in `results/missions/route_overview.png`, and the four-resolution maps and JSON for each case. Maximum route gap measures the farthest separation between a coarser route and the corresponding 1 m route. It samples both lines at intervals of at most 1 m and measures each sample to the other line. This captures changes in where the rover goes even when total distances are similar.

## Comparing with the Dec. 10, 2025 Perseverance drive

NASA's sol 1709 illustration shows the route, planned waypoints, and green keep-in polygons. The public waypoint record puts the drive endpoint at **18.42771922° N, 77.23659073° E**, west of the Jezero C source raster. The included `data/jezero-sol1709.tif` copies a fully covered 500 × 500 pixel window at original column **10423**, row **9197** from the 1 m USGS Mars Sample Return HiRISE mosaic. [NASA annotated map](https://science.nasa.gov/photojournal/mapping-perseverances-route-with-ai/) · [NASA waypoint data](https://mars.nasa.gov/mmgis-maps/M20/Layers/json/M20_waypoints.json) · [USGS western mosaic](https://www.usgs.gov/data/mars-sample-return-hirise-mosaics-terrain-relative-navigation)

`data/sol1709-corridor.json` records the eight manually traced polygons and five planned waypoint centers before the recorded endpoint. The approximate image registration uses the visible start and finish and NASA's sol 1707 and sol 1709 end-of-drive localizations to seed a similarity transform. It then refines the transform by comparing image texture with directional slopes from the USGS raster. The held-out texture correlation was 0.51. Refinement moved the image anchors by about 3 m and 5 m. Those offsets describe the fit adjustment. Absolute accuracy of the polygon boundaries remains unknown.

```powershell
.\.venv\Scripts\python.exe run_sol1709.py
```

Every run uses the same reconstructed polygons and ordered waypoint centers. The planner rasterizes the polygon union on the 1 m grid using cell centers. A coarser cell is usable if all of its underlying 1 m cells lie in the rasterized corridor and its coarse slope is below 20°. The harness also measures how much of the resulting line lies outside the 1 m corridor mask. The model uses the union of the traced zones and represents the rover as a point. Reported path lengths include connections to grid cell centers at the endpoints and waypoints.

The script runs two paired comparisons on the same terrain and writes maps and JSON under `results/sol1709/` and `results/sol1709/fixed_grid/`. In the first comparison, the planner uses the same grid as the terrain. In the second, terrain is coarsened in the same way, then copied back onto the 1 m planning grid. The second comparison holds route moves, waypoint connectors, and corridor rasterization fixed so that only the terrain information changes.

| Terrain spacing | Length with matching planner grid (m) | Of which waypoint connectors (m) | Length with planner fixed at 1 m (m) |
|---|---:|---:|---:|
| 1 m | 271.8 | 4.5 | 271.8 |
| 2 m | 272.5 | 8.5 | 268.4 |
| 5 m | 278.8 | 16.2 | 267.8 |
| 10 m | 306.5 | 51.0 | 267.8 |

The straight-line sum between successive fixed waypoints is 251.3 projected meters. With a matching planner grid, the grid-center portion fell from 267.3 to 255.6 m. The reported total rose because the longer waypoint connectors outweighed that decrease. Every route stayed inside the rasterized corridor. The native keep-in corridor contains zero 1 m cells at or above 20°, so this drive measures changes in route choice at that threshold. A hazard-detection benchmark also needs eligible corridors with native steep terrain.
