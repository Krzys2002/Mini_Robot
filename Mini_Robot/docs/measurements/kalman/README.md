# Pose filter comparison — 2026-11-07

These recorded software measurements use the deterministic synthetic metric-pose
replays in `tools/tracking/compare_pose_filters.cpp`. They are not physical camera
or robot measurements. Every scenario has 600 samples at 50 Hz; the first 25
samples are excluded from RMSE and lag calculations. See [settings](configuration.txt)
and [recorded metrics](metrics.csv). Position RMSE is Euclidean x/y error in metres;
heading RMSE uses wrapped angular error in radians. Position lag is the best
least-squares fit against a delayed true trajectory over 0–20 samples, reported
as mean elapsed time for the selected shift. This metric is approximate and
cannot distinguish all smoothing distortions; stationary lag is unidentifiable
(the tool selects zero on tied errors).

| Scenario | Exponential position RMSE | Kalman position RMSE | Exponential lag | Kalman lag |
| --- | ---: | ---: | ---: | ---: |
| Stationary | 2.918 mm | 4.860 mm | N/A | N/A |
| Constant motion | 8.512 mm | 4.860 mm | 80 ms | 0 ms |
| Turning | 15.698 mm | 5.146 mm | 80 ms | 0 ms |

The exponential filter suppresses this stationary noise more effectively. The
Kalman constant-velocity model tracks the moving cases with less error and less
lag at these settings. That benefit justifies retaining it as an optional
research estimator, not changing the default. Neither filter has been tuned on
held-out physical measurements; sinusoidal deterministic noise does not establish
general statistical superiority. Acceleration changes, occlusion, calibration
error and real detector outliers need separate evaluation. The default stays
exponential with alpha 1 until an experiment selects and validates smoothing.

Reproduce from repository root:

```sh
cmake --preset core
cmake --build --preset core --parallel 3
build/core/Mini_Robot/src/micro_robot_filter_comparison build/kalman-comparison
# Replay one generated metric-pose CSV through the same tracker again:
build/core/Mini_Robot/src/micro_robot_filter_comparison build/kalman-replay build/kalman-comparison/turning-input.csv
ctest --preset core -R 'KalmanPoseFilterTest|FilterComparison'
```

The tool writes configuration, metrics, ground-truth/input samples and both
filtered output streams. Only aggregate metrics/settings are versioned here;
full samples are reproducible build artifacts. An external replay must have a
header followed by `t,x,y,heading,true_x,true_y,true_heading` numeric CSV rows,
with increasing monotonic elapsed seconds (0–86400), metres and radians. At least
50 samples are required. Ground truth is required for this comparison, not for
the live tracker. The CSV regression checks identical outputs after round-trip
replay and rejects a truncated input. Real video can use the ordinary metric
batch pipeline with `filter=kalman`; quantitative comparison requires independently
measured ground truth associated with each captured frame.

Validation on macOS Apple Silicon: root core and macOS configure/build/test
passed (292 and 464 tests respectively). No physical live-camera, robot, GUI
inspection, Linux or Windows execution was performed for this estimator.
