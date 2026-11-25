# Jezero rover navigation experiment

Use the research question and method in README.md when changing the experiment.
Keep raster I/O in run_experiment.py and terrain math and A* in terrain.py.

1. Identify the change to the research question, model, or output. Record any new data source, units, assumptions, and expected behavior.
2. Implement the smallest complete change through the command-line experiment. Preserve a shared crop, physical endpoints, and route cost settings across grid resolutions unless the experiment intentionally varies one of them.
3. Run `python check.py` from the project root using the project virtual environment. Add a focused fixture when changing slope math, route costs, or reference scores in a way the existing checks would miss.
4. Run `python run_experiment.py` for changes to the experiment. Inspect the JSON and route map together. Explain changes to the route and the hazard metrics using the actual outputs.
5. For changes affecting multiple sites or required science stops, run `python run_missions.py`. In a resolution comparison, preserve each scenario's stop coordinates and visit order. Check the summary and route overview together.

Keep original elevation values and provenance. Describe slope thresholds as model choices and the 1 m grid as a reference. Cite the DEM separately from any imagery or rover traverse added to the project.
