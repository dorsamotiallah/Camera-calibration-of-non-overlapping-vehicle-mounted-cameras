# Calibration Approach Summary

Ground truth front-to-back optical-frame calibration:

```text
rotation:    180 deg around Y
translation: [0, 0.26, -0.76]
```

Translation difference is reported as:

```text
estimate - ground_truth = [dx, dy, dz]
```

Rotation error is computed from the printed ORB-SLAM Euler angles by reconstructing
`R = Rz * Ry * Rx` and comparing against the equivalent ground-truth Euler representation
`[180, 0, 180]`.

This summary intentionally focuses on the primary **first-pose optimization** result. The previous
final-pose optimization results were removed because they consistently behaved as a drift-sensitive
consistency check rather than the best calibration estimate.

## Primary Results Across Datasets

| Dataset | Approach | Translation | Difference `[dx, dy, dz]` | Rotation Error | Inliers | Source |
|---|---|---:|---:|---:|---:|---|
| Ground truth | Ground truth | `[0.0000, 0.2600, -0.7600]` | `[0.0000, 0.0000, 0.0000]` | `0.000 deg` | - | Gazebo camera pose |
| Good recording | CamMap baseline | `[0.0354, 0.2543, -0.7506]` | `[+0.0354, -0.0057, +0.0094]` | `0.849 deg` | `3815` | `orbcalib-master/good_results_2/calibration.txt` |
| Good recording | Current NMC3D: frame-to-frame equal-width distance-balanced CamMap | `[0.0460, 0.2529, -0.7615]` | `[+0.0460, -0.0071, -0.0015]` | `0.925 deg` | `1664` | `NMC3D/good_results_2/NMC3D_calibration.txt` |
| Multi-depth recording | CamMap baseline | `[-0.0186, 0.2692, -0.7278]` | `[-0.0186, +0.0092, +0.0322]` | `0.436 deg` | `478` | `orbcalib-master/results_multi_depth_2/CamMap_result.txt` |
| Multi-depth recording | Current NMC3D: frame-to-frame equal-width distance-balanced CamMap | `[-0.0179, 0.2685, -0.7620]` | `[-0.0179, +0.0085, -0.0020]` | `0.640 deg` | `176` | `NMC3D/results_multi_depth_2/NMC3D_result.txt` |

## Good Recording: All Tested Variants

Dataset:

```text
orbcalib-master/good_results_2
```

Current-code NMC3D result:

```text
NMC3D/good_results_2/NMC3D_calibration.txt
```

Archived NMC3D variants:

```text
NMC3D/calibration_versions
```

| Approach | Method | Translation | Difference `[dx, dy, dz]` | Rotation Error | Inliers |
|---|---|---:|---:|---:|---:|
| CamMap baseline | Original CamMap matching and global Sim3 calibration | `[0.0354, 0.2543, -0.7506]` | `[+0.0354, -0.0057, +0.0094]` | `0.849 deg` | `3815` |
| NMC3D global depth tertiles + 8x8 selected optimization | CamMap matches first, then global depth-balanced selected matches are used for optimization | `[0.0354, 0.2543, -0.7506]` | `[+0.0354, -0.0057, +0.0094]` | `0.849 deg` | `3815` |
| NMC3D global thresholds for scoring, all-depth optimization | Depth-balanced subsets score/select keyframe pairs; all-depth matches are used in optimization | `[0.0294, 0.2523, -0.7475]` | `[+0.0294, -0.0077, +0.0125]` | `0.834 deg` | `3305` |
| NMC3D frame-to-frame Euclidean tertiles + 8x8 grid | Each accepted keyframe pair is locally balanced by Euclidean 3D distance before optimization | `[0.0355, 0.2542, -0.7507]` | `[+0.0355, -0.0058, +0.0093]` | `0.850 deg` | `3815` |
| Current NMC3D: frame-to-frame equal-width distance-balanced CamMap | Each accepted keyframe pair is locally balanced using equal-width Euclidean distance bins and 8x8 image-grid selection | `[0.0460, 0.2529, -0.7615]` | `[+0.0460, -0.0071, -0.0015]` | `0.925 deg` | `1664` |

## Multi-Depth Recording

CamMap output:

```text
orbcalib-master/results_multi_depth_2/CamMap_result.txt
```

NMC3D output:

```text
NMC3D/results_multi_depth_2/NMC3D_result.txt
```

| Approach | Method | Translation | Difference `[dx, dy, dz]` | Rotation Error | Inliers |
|---|---|---:|---:|---:|---:|
| CamMap baseline | Original CamMap matching and global Sim3 calibration | `[-0.0186, 0.2692, -0.7278]` | `[-0.0186, +0.0092, +0.0322]` | `0.436 deg` | `478` |
| Current NMC3D: frame-to-frame equal-width distance-balanced CamMap | Each accepted keyframe pair is locally balanced using equal-width Euclidean distance bins and 8x8 image-grid selection | `[-0.0179, 0.2685, -0.7620]` | `[-0.0179, +0.0085, -0.0020]` | `0.640 deg` | `176` |

## Methodology of Each Approach

### 1. CamMap Baseline

CamMap performs the original calibration pipeline:

1. Load front and back ORB-SLAM3 atlases.
2. For each source keyframe, query target keyframes using BoW/place recognition.
3. Use covisibility groups and BoW matches to initialize a Sim3.
4. Use projection matching and Sim3 optimization to validate common regions.
5. Collect matched keyframe pairs and matched map points.
6. Run global bidirectional reprojection optimization to estimate the camera-to-camera Sim3.

There is no explicit depth balancing. The optimizer receives whatever matches the CamMap pipeline found.

### 2. Global Depth Tertiles + 8x8 Selected Optimization

Archived as:

```text
NMC3D/calibration_versions/calib_cammap_matching_global_depth_tertiles_8x8_grid_selected_optimization.cpp
```

Method:

1. Run CamMap matching normally.
2. Collect all accepted keyframe-pair matches.
3. Compute source-camera depth using `Pc1.z()`.
4. Sort all matches globally by depth.
5. Split the full global set into three equal-count tertiles: near, middle, far.
6. Within each tertile, distribute selection over an 8x8 source-image grid.
7. Use only the selected near/middle/far-balanced matches in global optimization.

This is a global post-filter: matching is unchanged, but optimization receives a balanced subset.

### 3. Global Depth Thresholds + Local Random Uniform Selection

Archived as:

```text
NMC3D/calibration_versions/calib_global_depth_thresholds_local_random_uniform_selection_final_optimization_on_selected_features.cpp
```

Method:

1. Compute global near/far depth thresholds from all source atlas map-point depths.
2. During each local keyframe-pair match, classify matches as near, middle, or far using those global thresholds.
3. Choose an equal number from each bin using local random/uniform selection.
4. Use the selected depth-balanced matches for final optimization.

No saved numeric output file was present for this archived variant, so it is documented as an
implementation path rather than a result-table row.

### 4. Global Thresholds for Keyframe Scoring, All-Depth Optimization

Archived as:

```text
NMC3D/calibration_versions/calib_global_depth_thresholds_selected_scoring_all_depth_first_and_final_optimization.cpp
```

Method:

1. Compute global depth thresholds from source atlas map points.
2. For each candidate keyframe pair, create a locally balanced match subset.
3. Use the balanced subset only for candidate scoring / keyframe-pair selection.
4. Store both selected matches and all original all-depth matches.
5. Run final global optimization using all-depth matches.

This was a compromise: depth balancing influences which keyframe pairs are trusted, but the optimizer
still receives all available geometric constraints.

### 5. Frame-to-Frame Euclidean Tertiles + 8x8 Grid

Archived as:

```text
NMC3D/calibration_versions/calib_frame_to_frame_euclidean_distance_tertiles_8x8_grid_selected_global_bidirectional_optimization.cpp
```

Method:

1. Run CamMap candidate detection normally.
2. After projection matching and geometric validation, process each accepted keyframe pair independently.
3. For each matched pair of map points, compute:

```text
distance = 0.5 * (||Pc1|| + ||Pc2||)
```

4. Sort that keyframe pair's matches by Euclidean camera distance.
5. Split the local match set into near, middle, and far tertiles.
6. Select an equal number from the three groups.
7. Within each group, distribute selection over an 8x8 source-image grid.
8. Use the selected matches in global bidirectional optimization.

### 6. Current NMC3D: Frame-to-Frame Equal-Width Distance-Balanced CamMap

This is the current / final version represented by:

```text
NMC3D/calib/calib.cpp
```

A more descriptive name is:

```text
Frame-to-frame equal-width distance-balanced CamMap with 8x8 image-grid selection
```

Method:

1. Run the original CamMap candidate detection, Sim3 initialization, projection matching, and geometric validation.
2. For each accepted keyframe pair, inspect only the matches already found by CamMap.
3. For each matched map-point pair, compute the average Euclidean camera distance:

```text
distance = 0.5 * (||Pc1|| + ||Pc2||)
```

4. Build equal-width distance bins from the local minimum and maximum distance:

```text
near:   min <= d < min + range / 3
middle:      d < min + 2 * range / 3
far:         d >= min + 2 * range / 3
```

5. Select an equal number of matches from near, middle, and far, limited by the smallest bin.
6. Within each bin, use an 8x8 image grid so selected points are spread across the source keyframe image.
7. Run the global calibration optimization using these selected frame-to-frame balanced matches.

This does not create new correspondences. It changes which already-found CamMap correspondences are
allowed to influence the final optimization.

## Final Conclusion on the Current Code

The current NMC3D method is best described as:

```text
Frame-to-frame equal-width distance-balanced CamMap with 8x8 image-grid selection
```

Compared with CamMap, it is more selective and uses fewer optimization inliers:

| Dataset | CamMap Inliers | Current NMC3D Inliers |
|---|---:|---:|
| Good recording | `3815` | `1664` |
| Multi-depth recording | `478` | `176` |

On the good recording, CamMap is already excellent. Current NMC3D makes the `z` component almost exact
but increases the `x` offset:

```text
CamMap difference:       [+0.0354, -0.0057, +0.0094]
Current NMC3D difference:[+0.0460, -0.0071, -0.0015]
```

On the multi-depth recording, current NMC3D gives the best translation vector overall, mainly by fixing
the depth-axis component:

```text
CamMap difference:       [-0.0186, +0.0092, +0.0322]
Current NMC3D difference:[-0.0179, +0.0085, -0.0020]
```

So the final version is most valuable when the scene contains meaningful depth diversity. It reduces
depth-axis bias by preventing the optimizer from being dominated by whichever depth range produced the
most raw CamMap matches.

