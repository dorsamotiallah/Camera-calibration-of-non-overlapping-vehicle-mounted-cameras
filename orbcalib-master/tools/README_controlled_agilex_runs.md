# Controlled Agilex SLAM and Calibration Runs

These commands are intended to run inside the `orbcalib-noetic` Docker
container, where the repo is mounted at `/ws/src/orbcalib-master`.

## Build

After code changes, rebuild the calibration executable:

```bash
cd /ws/src/orbcalib-master
source /opt/ros/noetic/setup.bash
cmake --build build --target calib -j$(nproc)
```

## 1. Build Atlases With Controlled SLAM

The controlled SLAM script:

- starts `roscore`
- generates per-run config files
- runs `build/calib/calib` in SLAM mode
- publishes paired PNG frames with ACK pacing
- stops ORB-SLAM with `SIGINT` so atlases are saved
- exports raw keyframe observations for later scale estimation

General form:

```bash
cd /ws/src/orbcalib-master
tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/T7/Agilex outdoor data 8.6.2026/LobbywithTags_defished_fov125_diag" \
  --camera1 left \
  --camera2 right \
  --camera1-config config/sim/agilex_left_defished_cam.yaml \
  --camera2-config config/sim/agilex_right_defished_cam.yaml \
  --run-id agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --no-viewer
```

Front/back example:

```bash
tools/run_agilex_controlled_slam.sh \
  --dataset-root "/ws/src/T7/Agilex outdoor data 8.6.2026/LobbywithTags_defished_fov125" \
  --camera1 front \
  --camera2 back \
  --camera1-config config/sim/agilex_front_defished_cam.yaml \
  --camera2-config config/sim/agilex_back_defished_cam.yaml \
  --run-id agilex_8_6_2026_T7_LobbywithTags_front_back_defished_fov125 \
  --no-viewer
```

Useful options:

```bash
--pairing nearest              # pair frames by nearest timestamp
--max-skew-sec 0.05            # max timestamp skew for nearest pairing
--start-index 100              # start from selected pair index
--max-pairs 1000               # process only this many selected pairs
--viewer                       # enable Pangolin viewer
--viewer-warmup-sec 5          # wait before publishing first frame
--pause-before-playback        # wait for Enter before frame publishing
--skip-bad-images              # skip unreadable PNG pairs
```

Main outputs:

```text
results_agilex/<run_id>/manifest.txt
results_agilex/<run_id>/slam.log
results_agilex/<run_id>/<camera1>_atlasCamera 1.osa
results_agilex/<run_id>/<camera2>_atlasCamera 2.osa
results_agilex/<run_id>/<camera1>_raw_keyframe_observations.csv
results_agilex/<run_id>/<camera2>_raw_keyframe_observations.csv
```

## 2. Run Calibration Without Metric Scale Correction

This loads the atlases from an existing run folder and runs camera-to-camera
calibration in map units.

```bash
cd /ws/src/orbcalib-master
tools/run_agilex_controlled_calib.sh \
  --run-id agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera1 left \
  --camera2 right \
  --camera1-config config/sim/agilex_left_defished_cam.yaml \
  --camera2-config config/sim/agilex_right_defished_cam.yaml \
  --no-viewer \
  --no-global-map-scales
```

Outputs:

```text
results_agilex/<run_id>/calib.log
results_agilex/<run_id>/calib_keyframe_matches.csv
results_agilex/<run_id>/roscore_calib.log
```

## 3. Run Calibration With Metric Map Scales

Use this after estimating one scale per map. The scale values are in
meters per SLAM unit.

```bash
tools/run_agilex_controlled_calib.sh \
  --run-id agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera1 left \
  --camera2 right \
  --camera1-config config/sim/agilex_left_defished_cam.yaml \
  --camera2-config config/sim/agilex_right_defished_cam.yaml \
  --no-viewer \
  --use-global-map-scales \
  --camera1-global-scale 5.18248537263193 \
  --camera2-global-scale 7.030844632273152 \
  --free-scale-after-global-scaling
```

Outputs:

```text
results_agilex/<run_id>/ground_scale/calib_scaled.log
results_agilex/<run_id>/ground_scale/calib_keyframe_matches_scaled.csv
results_agilex/<run_id>/ground_scale/calib_agilex_scaled.yaml
```

The script currently writes scaled calibration outputs under `ground_scale/`
for any scale source. If comparing point-pair and ground-plane scales, archive
or rename that output folder between runs, for example:

```bash
mv results_agilex/<run_id>/ground_scale results_agilex/<run_id>/PointPair_Calib
```

Then rerun calibration with the second scale pair.

## Notes

- `camera1` and `camera2` order matters. The resulting transform should be
  compared against the matching ground-truth direction.
- `--fix-scale-after-global-scaling` fixes the final Sim3 scale to 1 after map
  scaling. `--free-scale-after-global-scaling` lets the calibration still
  optimize residual Sim3 scale.
- The calibration logs can contain multiple candidate poses. For summary
  tables, use the optimized block with nonzero `inliers size`.
