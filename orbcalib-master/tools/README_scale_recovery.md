# ArUco Scale Recovery Tools

These tools estimate monocular ORB-SLAM map scale from ArUco observations.
Run them after `tools/run_agilex_controlled_slam.sh`, because that script
creates the raw observation CSVs used here.

The output scale convention is:

```text
metric meters = SLAM units * scale_m_per_slam_unit
```

## Inputs Produced by Controlled SLAM

Expected files:

```text
results_agilex/<run_id>/manifest.txt
results_agilex/<run_id>/<camera>_raw_keyframe_observations.csv
results_agilex/<run_id>/frame_pairs.csv
```

The tools use `manifest.txt` to find camera names, image folders, and raw
observation CSVs. You can override image folders with `--camera1-dir` and
`--camera2-dir` if the original container paths are not valid.

## Point-Pair ArUco Scale

Use `estimate_aruco_slam_scale.py` when marker side length is known. It detects
ArUco markers in keyframe images, keeps map points that land inside marker
polygons, compares pairwise metric distances on the marker with SLAM distances,
and reports a robust map scale.

Left camera example:

```bash
cd /ws/src/orbcalib-master
python3 tools/estimate_aruco_slam_scale.py \
  --run-dir results_agilex/agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera left \
  --marker-length-m 0.182 \
  --dictionary-size 50
```

Right camera example:

```bash
python3 tools/estimate_aruco_slam_scale.py \
  --run-dir results_agilex/agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera right \
  --marker-length-m 0.182 \
  --dictionary-size 50
```

Useful options:

```bash
--marker-id 0 --marker-id 1    # keep only selected marker IDs
--margin-px 2                  # include nearby points outside marker polygon
--min-points 4                 # minimum selected points per marker observation
--max-keyframes 100            # quick test on first N keyframes
--no-debug-images              # skip overlay image output
--outlier-mad 3.5              # robust outlier cutoff
```

Outputs:

```text
results_agilex/<run_id>/aruco_scale/<camera>/<camera>_aruco_scale_summary.json
results_agilex/<run_id>/aruco_scale/<camera>/<camera>_aruco_scale_points.csv
results_agilex/<run_id>/aruco_scale/<camera>/<camera>_aruco_scale_pairs.csv
results_agilex/<run_id>/aruco_scale/<camera>/debug_images/
```

Use `recommended_scale_m_per_slam_unit` from the summary JSON for calibration.
The scale statistics also include raw median/mean/std and kept median/mean/std
after MAD filtering.

## Ground-Plane Height Scale

Use `estimate_aruco_ground_plane_scale.py` when the markers lie on a ground
plane and camera optical-center height is known. It fits a plane to marker map
points, measures camera-center distance to that plane in SLAM units, and
converts known camera height into map scale.

Two-camera example:

```bash
cd /ws/src/orbcalib-master
python3 tools/estimate_aruco_ground_plane_scale.py \
  --run-dir results_agilex/agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera left \
  --camera right \
  --camera-height left=0.536276885562841 \
  --camera-height right=0.5447200312305958 \
  --dictionary-size 50 \
  --marker-length-m 0.182
```

Single shared height example:

```bash
python3 tools/estimate_aruco_ground_plane_scale.py \
  --run-dir results_agilex/<run_id> \
  --camera front \
  --camera back \
  --camera-height-m 0.54 \
  --dictionary-size 50
```

Useful options:

```bash
--marker-id 0 --marker-id 1    # use selected markers only
--min-points 12                # minimum unique marker map points per camera
--ransac-threshold 0.03        # plane threshold in SLAM units
--ransac-iterations 3000       # plane RANSAC iterations
--max-keyframes 100            # quick test
--outlier-mad 3.5              # camera-distance scale outlier cutoff
```

Outputs:

```text
results_agilex/<run_id>/aruco_ground_scale/aruco_ground_scale_summary.json
results_agilex/<run_id>/aruco_ground_scale/<camera>_aruco_ground_points.csv
results_agilex/<run_id>/aruco_ground_scale/<camera>_camera_plane_distances.csv
```

Use each camera's `recommended_scale_m_per_slam_unit` from
`aruco_ground_scale_summary.json` for scaled calibration.

## Applying Recovered Scales in Calibration

Point-pair example:

```bash
tools/run_agilex_controlled_calib.sh \
  --run-id agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera1 left \
  --camera2 right \
  --camera1-config config/sim/agilex_left_defished_cam.yaml \
  --camera2-config config/sim/agilex_right_defished_cam.yaml \
  --use-global-map-scales \
  --camera1-global-scale 5.222149432044507 \
  --camera2-global-scale 6.946616506752742 \
  --free-scale-after-global-scaling \
  --no-viewer
```

Ground-plane example:

```bash
tools/run_agilex_controlled_calib.sh \
  --run-id agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera1 left \
  --camera2 right \
  --camera1-config config/sim/agilex_left_defished_cam.yaml \
  --camera2-config config/sim/agilex_right_defished_cam.yaml \
  --use-global-map-scales \
  --camera1-global-scale 5.18248537263193 \
  --camera2-global-scale 7.030844632273152 \
  --free-scale-after-global-scaling \
  --no-viewer
```

No-scale baseline:

```bash
tools/run_agilex_controlled_calib.sh \
  --run-id agilex_8_6_2026_T7_LobbywithTags_left_right_defished_fov125_diag \
  --camera1 left \
  --camera2 right \
  --camera1-config config/sim/agilex_left_defished_cam.yaml \
  --camera2-config config/sim/agilex_right_defished_cam.yaml \
  --no-global-map-scales \
  --no-viewer
```

## Quick Quality Checks

- Point-pair scale should have a reasonably tight kept standard deviation.
- Ground-plane scale should have a high `plane_inlier_fraction`.
- If scaled calibration translation gets worse, compare against the no-scale
  baseline and check camera order, GT transform direction, and marker-plane
  height assumptions.
