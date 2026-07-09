# Trajectory-Based Extrinsic Calibration via ArUco

`align_trajectories_to_aruco.py` calibrates two cameras that ran
**independent monocular SLAM sessions with no shared visual overlap** (e.g.
front vs. left, front vs. back), using a shared ArUco marker as the common
reference frame instead of feature matching between the two maps.

Run it after `estimate_aruco_slam_scale.py` (see `README_scale_recovery.md`)
and after re-exporting the raw keyframe CSVs with the pose-carrying
`atlas_export_observations` build (adds `qw,qx,qy,qz` columns).

## What It Does

One tool, three stages, run together (each reuses what the previous one
computes -- no need to invoke them separately):

1. **Alignment.** Picks the marker id best seen by both cameras, picks each
   camera's best-observed keyframe of that marker, runs `cv2.solvePnP`
   against the marker's real geometry to get that keyframe's pose in the
   marker frame, and combines it with that same keyframe's own SLAM-frame
   pose (read from the raw CSV) to get one similarity transform (scale,
   rotation, translation) per camera. That transform is applied to every
   keyframe, giving each camera's full trajectory in the shared, metric,
   marker-centered frame. Plots both trajectories.

2. **Extrinsic calibration**, reusing stage 1's trajectories. Since the two
   cameras share no visual content, keyframes are matched by timestamp
   instead: for every camera2 keyframe, camera1's pose is interpolated
   (SLERP + linear) to that exact timestamp, and the relative pose is
   computed at every match. Matches are filtered by distance from each
   camera's own anchor keyframe (accuracy degrades with distance -- see
   Notes), and the survivors are robustly averaged (SVD chordal mean for
   rotation, per-axis median for translation) into the "averaged" extrinsic.

3. **Joint optimization refinement**, reusing stage 2's surviving matches, in
   two variants:

   - **3a (per-component, `scipy.optimize.least_squares`).** Fits a single
     (R, t) minimizing one combined robust (Huber by default) loss over
     rotation and translation residuals *together*, across all matches at
     once -- seeded from stage 2's result. The robust loss is applied to
     each of the 6 residual numbers per match (3 rotation + 3 translation)
     independently, because that's what `least_squares`'s API supports.
   - **3b (grouped, `scipy.optimize.minimize`, L-BFGS-B).** Same idea, but
     the robust loss is applied once to each match's *combined* 6D residual
     norm, so a match with a large combined error is downweighted or
     rejected as a whole rather than having each of its 6 numbers judged
     independently. This is the thing a generic scalar minimizer (the
     Python analog of MATLAB's `fminunc`) can do that `least_squares`'s
     per-component loss structurally cannot.

   Outlier matches are smoothly downweighted instead of only relying on the
   hard distance/time cutoffs. All three results (averaged, optimized,
   optimized-grouped) are printed and saved side by side (as
   `T_<camera1>_<camera2>`, `T_<camera1>_<camera2>_optimized`, and
   `T_<camera1>_<camera2>_optimized_grouped`) so they can be compared run to
   run -- neither optimizer is assumed to be better a priori.

## Inputs Required

Per camera:

```text
results_agilex/<run_id>/aruco_scale/<camera>/<camera>_aruco_scale_points.csv   # from estimate_aruco_slam_scale.py
results_agilex/<run_id>/<camera>_raw_keyframe_observations.csv                 # must include qw,qx,qy,qz columns
config/sim/agilex_<camera>_defished_cam.yaml                                   # camera intrinsics
```

If a camera's raw CSV predates the pose export fix, regenerate it:

```bash
./build/calib/atlas_export_observations \
  Vocabulary/ORBvoc.txt \
  results_agilex/<run_id>/config/<camera>_controlled_observation_export_load.yaml \
  "Camera 1" \
  results_agilex/<run_id>/<camera>_raw_keyframe_observations.csv
```

(`"Camera 1"` for camera1, `"Camera 2"` for camera2 -- must match the atlas
prefix used when the run was recorded.)

## Usage

```bash
python3 tools/align_trajectories_to_aruco.py \
  --camera1-name front \
  --camera1-points-csv results_agilex/<run_id>/aruco_scale/front/front_aruco_scale_points.csv \
  --camera1-raw-csv results_agilex/<run_id>/front_raw_keyframe_observations.csv \
  --camera1-config config/sim/agilex_front_defished_cam.yaml \
  --camera2-name left \
  --camera2-points-csv results_agilex/<run_id>/aruco_scale/left/left_aruco_scale_points.csv \
  --camera2-raw-csv results_agilex/<run_id>/left_raw_keyframe_observations.csv \
  --camera2-config config/sim/agilex_left_defished_cam.yaml \
  --max-distance-from-anchor-m 1.0
```

Run once *without* `--max-distance-from-anchor-m` first: the tool prints a
sensitivity table (matches kept / rotation deviation / translation deviation
at several cutoffs) so you can pick a cutoff deliberately instead of
guessing. Then rerun with your chosen cutoff for the final saved extrinsic.

Useful options:

```text
--marker-id 3                     # force a marker instead of auto-selecting the best-shared one
--keyframe-candidates 5            # try this many top-by-point-count keyframes, keep lowest PnP RMS
--marker-length-m 0.182            # marker side length, for the plot's marker outline only
--camera1-scale 5.864969842432022  # camera1's SLAM map scale (m per SLAM unit), e.g. from
                                    # estimate_aruco_ground_plane_scale.py or a MATLAB ground-plane
                                    # fit. If omitted, falls back to extracting it from marker point
                                    # pairs at the anchor keyframe (see Notes).
--camera2-scale 4.0349409367671125 # same as --camera1-scale, for camera2
--max-bracket-width-s 0.5          # drop matches where camera1's bracketing keyframes are farther
                                    # apart in time than this (usually matters little, see Notes)
--max-distance-from-anchor-m 1.0   # drop matches farther than this from either camera's own anchor
                                    # keyframe (the main quality knob, see Notes)
--show                             # also display the plot interactively (it is always saved)
--output-plot PATH                 # override the default save location (see below)
--output-alignment-json PATH       # override the default save location (see below)
--output-extrinsic-yaml PATH       # override the default save location (see below)
--optimize-loss huber              # robust loss for stage 3: linear/huber/soft_l1/cauchy (default: huber)
--optimize-f-scale-m 0.02          # stage 3's robust-loss transition scale (meters-equivalent). Default:
                                    # 1.5x the median translation deviation from the stage-2 average.
```

## Outputs

Always saved (no flags required), all under `<run-dir>/aruco_alignment/`,
where `<run-dir>` is `--camera1-raw-csv`'s parent directory:

```text
results_agilex/<run_id>/aruco_alignment/<camera1>_<camera2>_trajectories.png   # stage 1: 3D plot of both trajectories
results_agilex/<run_id>/aruco_alignment/<camera1>_<camera2>_alignment.json     # stage 1: marker/keyframes/scale/Sim3 per camera
results_agilex/<run_id>/aruco_alignment/<camera1>_<camera2>_extrinsic.yaml     # stage 2 + 3: both T_ blocks, plus diagnostics
```

The extrinsic YAML contains **two** blocks, both in the format used in
`Agilex Recordings/Intrinsic_ground_truths/robot_relative_extrinsics.yaml`
exactly -- `from_frame`/`to_frame`, `translation_xyz`, `euler_zyx_deg`
(`roll`/`pitch`/`yaw`, ZYX convention: `R = Rz(yaw) @ Ry(pitch) @ Rx(roll)`),
and the full 4x4 homogeneous `matrix`:

- `T_<camera1>_<camera2>` -- stage 2's averaged result.
- `T_<camera1>_<camera2>_optimized` -- stage 3a's per-component joint-optimization result.
- `T_<camera1>_<camera2>_optimized_grouped` -- stage 3b's grouped joint-optimization result.

Any of the three can be dropped straight into the ground-truth file or
diffed against it. A `calibration_diagnostics` section (match counts,
cutoffs used, rotation/translation deviation stats, and `optimization` /
`optimization_grouped` sub-sections -- loss/robust scale used, cost
before/after, how far each stage-3 variant moved from stage 2) is appended
below all three blocks, outside them.

Also printed to stdout: the marker-selection table, per-camera PnP/scale
diagnostics, the bracket-width and distance-from-anchor sensitivity tables,
and all three final `T_<camera1>_<camera2>*` blocks.

## Notes

- **Marker coverage matters.** Pick (or let auto-selection pick) a marker
  actually seen by both cameras -- check the printed per-marker keyframe
  counts. A marker only one camera saw is useless for this.
- **Distance from the anchor keyframe, not timestamp gap, is the dominant
  accuracy driver.** Each camera's transform is exact at its own anchor
  keyframe and degrades slowly with distance from it (monocular SLAM has no
  other metric constraint over a long loop). Always check the
  `--max-distance-from-anchor-m` sensitivity table rather than assuming a
  default cutoff is right for a new dataset.
- **Validate against any ground truth you have.** Comparing to a known
  extrinsic caught a real sign bug in an earlier version of
  `relative_extrinsic()` (camera center vs. `tcw` translation vector -- they
  are related by `t = -R @ C`, not interchangeable) that a purely internal
  consistency check (spread across matches) did not reveal, since the bug
  was a systematic bias, not noise.
- **Stage 3 only differs from stage 2 when it matters.** On clean data (no
  outlier matches) the Huber-loss joint fit and the chordal-mean/median
  average land on essentially the same answer -- that's expected, not a bug;
  Huber loss is designed to behave like ordinary least squares near the
  bulk of the data and only downweight far-out residuals. Where it should
  visibly help is exactly the case the hard `--max-distance-from-anchor-m`
  cutoff exists for: a few still-drifted matches slipping past the cutoff.
  If stage 3 moves far from stage 2 on a "clean" run, that itself is a
  signal worth investigating (e.g. the cutoff is too loose), not something
  to blindly trust just because it came from an optimizer.
- By default, scale is extracted internally by comparing real distances
  between marker points (from the marker's known physical size) to their
  reconstructed SLAM distances, at the anchor keyframe -- see the point-pair
  method in `estimate_aruco_slam_scale.py` / `README_scale_recovery.md`. Pass
  `--camera1-scale`/`--camera2-scale` to override this per camera with a
  scale from elsewhere instead (e.g. `recommended_scale_m_per_slam_unit`
  from `estimate_aruco_ground_plane_scale.py`'s summary JSON, or a MATLAB
  `estimate_ground_plane_from_atlas_csv.m` ground-plane-fit +
  known-camera-height result) -- useful since the marker-point-pair method
  can be noisy for small markers viewed from a distance, while ground-plane
  fitting has a much larger effective baseline. The alignment JSON records
  which source was used per camera (`scale_source`: `"external"` or
  `"anchor_marker_points"`).
