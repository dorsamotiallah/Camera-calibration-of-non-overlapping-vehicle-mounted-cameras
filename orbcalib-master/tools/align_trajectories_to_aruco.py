#!/usr/bin/env python3
"""Align two independent camera trajectories via ArUco, then calibrate their extrinsic.

Two stages, run together since the second reuses everything the first computes:

STAGE 1 -- alignment. Reads the per-point CSVs written by
estimate_aruco_slam_scale.py for two cameras, picks the marker id best
observed by both maps, and for each camera picks the keyframe with the most
marker points. At that keyframe it runs cv2.solvePnP once, against the real
marker geometry (object points = marker_x_m/marker_y_m/0, image points =
that keyframe's pixel detections), giving the camera's pose in the marker's
metric frame. The raw observations CSV (as written by the updated
atlas_export_observations) also carries every keyframe's own SLAM-frame pose
(camera_x/y/z plus a qw/qx/qy/qz world-to-camera rotation quaternion,
straight from ORB-SLAM3). Composing that anchor keyframe's SLAM-frame pose
with its marker-frame PnP pose gives the similarity transform (scale,
rotation, translation) from that camera's SLAM frame into the marker frame.
Scale (meters per SLAM unit) is normally extracted from marker point pairs
at the anchor keyframe, but can be supplied directly per camera instead via
--camera1-scale/--camera2-scale (e.g. from a ground-plane-fit scale
estimate), which skips that extraction. That single per-camera transform is
applied to every keyframe's full pose (position and orientation). Both
cameras' transformed trajectories are plotted together and saved to a
summary JSON.

STAGE 2 -- extrinsic calibration, reusing stage 1's two trajectories. Since
the two cameras share no visual content, keyframes can't be matched by
content -- only by time. For every camera2 keyframe timestamp, camera1's
pose is interpolated (linear position, SLERP rotation) between its two
bracketing keyframes. Every interpolated match gives one estimate of the
camera2->camera1 extrinsic; since the rig is rigid these should all agree,
so they are combined with a robust rotation/translation average. Distance
from each camera's own anchor keyframe is the dominant quality signal
(monocular SLAM scale/pose drift grows with distance from it), so matches
are filtered by that, and a sensitivity table is always printed so the
cutoff can be chosen deliberately rather than guessed.

STAGE 3 -- joint optimization refinement, reusing stage 2's surviving
matches. The averaged (chordal-mean rotation, median translation) result
above is a "solve each match independently, then reduce" estimate. This
stage instead fits a single (R, t) directly against every surviving match's
raw pose pair at once (a joint nonlinear least-squares over SE(3), seeded
from the averaged result), with a robust (Huber) loss so remaining outlier
matches are automatically downweighted rather than relying solely on the
hard anchor-distance/time cutoffs. Both the averaged and the optimized
result are printed and saved side by side so they can be compared run to
run -- the optimizer is not assumed to be better a priori.

Example:

  python3 tools/align_trajectories_to_aruco.py \\
    --camera1-name front \\
    --camera1-points-csv results_agilex/<run>/aruco_scale/front/front_aruco_scale_points.csv \\
    --camera1-raw-csv results_agilex/<run>/front_raw_keyframe_observations.csv \\
    --camera1-config config/sim/agilex_front_defished_cam.yaml \\
    --camera2-name left \\
    --camera2-points-csv results_agilex/<run>/aruco_scale/left/left_aruco_scale_points.csv \\
    --camera2-raw-csv results_agilex/<run>/left_raw_keyframe_observations.csv \\
    --camera2-config config/sim/agilex_left_defished_cam.yaml \\
    --max-distance-from-anchor-m 1.0
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
from scipy.optimize import least_squares, minimize

from estimate_checkerboard_camera_height import CameraCalibration, read_calibration


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--camera1-name", required=True)
    parser.add_argument("--camera1-points-csv", required=True, type=Path, help="<camera>_aruco_scale_points.csv")
    parser.add_argument("--camera1-raw-csv", required=True, type=Path, help="<camera>_raw_keyframe_observations.csv")
    parser.add_argument("--camera1-config", required=True, type=Path, help="ORB-SLAM style camera intrinsics yaml")
    parser.add_argument(
        "--camera1-scale",
        type=float,
        help="camera1's SLAM map scale in meters per SLAM unit (e.g. from "
        "estimate_aruco_ground_plane_scale.py or a MATLAB ground-plane fit). If omitted, "
        "falls back to extracting it from marker point pairs at the anchor keyframe.",
    )

    parser.add_argument("--camera2-name", required=True)
    parser.add_argument("--camera2-points-csv", required=True, type=Path)
    parser.add_argument("--camera2-raw-csv", required=True, type=Path)
    parser.add_argument("--camera2-config", required=True, type=Path)
    parser.add_argument(
        "--camera2-scale",
        type=float,
        help="camera2's SLAM map scale in meters per SLAM unit. Same fallback as --camera1-scale.",
    )

    parser.add_argument("--marker-id", type=int, help="Force this marker id instead of auto-selecting.")
    parser.add_argument(
        "--keyframe-candidates",
        type=int,
        default=5,
        help="Try this many top by-point-count keyframes per camera and keep the one with the lowest marker PnP reprojection RMS.",
    )
    parser.add_argument("--marker-length-m", type=float, default=0.182, help="Marker side length, for the plot outline only.")

    parser.add_argument(
        "--max-bracket-width-s",
        type=float,
        help="Drop matches where camera1's bracketing keyframes are farther apart than this "
        "many seconds (i.e. camera2's timestamp falls in a sparse stretch of camera1's "
        "trajectory). Default: no filtering, but a sensitivity table is always printed.",
    )
    parser.add_argument(
        "--max-distance-from-anchor-m",
        type=float,
        help="Drop matches where either camera's point is farther than this many meters (Euclidean, "
        "in the marker frame) from that camera's own anchor keyframe. Each camera's Sim3 is only "
        "well-constrained near its marker anchor -- monocular SLAM scale/pose drift grows with "
        "distance from it. Default: no filtering, but a sensitivity table is always printed. "
        "Euclidean distance is an imperfect proxy for accumulated drift (e.g. a keyframe can "
        "revisit the anchor's physical location much later, after more drift, and still look "
        "close by this measure) -- see also --max-time-from-anchor-s.",
    )
    parser.add_argument(
        "--max-time-from-anchor-s",
        type=float,
        help="Drop matches where either camera's keyframe is farther than this many seconds from "
        "that camera's own anchor keyframe, independent of --max-distance-from-anchor-m. SLAM "
        "drift accumulates with trajectory distance, not physical proximity, so a keyframe can be "
        "Euclidean-close to the anchor while being far away in time (e.g. a later revisit of the "
        "same spot) and still carry much more accumulated drift than the distance filter alone "
        "would catch. Default: no filtering, but a sensitivity table is always printed.",
    )

    parser.add_argument(
        "--output-plot",
        type=Path,
        help="Path to save the 3D trajectory plot (PNG). Defaults to "
        "<run-dir>/aruco_alignment/<camera1>_<camera2>_trajectories.png, where <run-dir> is "
        "--camera1-raw-csv's parent directory.",
    )
    parser.add_argument(
        "--output-alignment-json",
        type=Path,
        help="Path to save the stage-1 alignment summary JSON. Defaults to "
        "<run-dir>/aruco_alignment/<camera1>_<camera2>_alignment.json.",
    )
    parser.add_argument(
        "--output-extrinsic-yaml",
        type=Path,
        help="Path to save the stage-2 extrinsic result (T_<camera1>_<camera2>, matching the "
        "robot_relative_extrinsics.yaml ground-truth format). Defaults to "
        "<run-dir>/aruco_alignment/<camera1>_<camera2>_extrinsic.yaml.",
    )
    parser.add_argument("--show", action="store_true", help="Also show the plot interactively (it is always saved).")

    parser.add_argument(
        "--optimize-loss",
        default="huber",
        choices=["linear", "huber", "soft_l1", "cauchy"],
        help="Robust loss for the stage-3 joint SE(3) optimization refinement. 'linear' is a "
        "plain (non-robust) least-squares fit. Default: huber.",
    )
    parser.add_argument(
        "--optimize-f-scale-m",
        type=float,
        help="Robust loss transition scale (meters-equivalent residual magnitude) for stage 3 -- "
        "residuals below this are treated as inliers (quadratic cost), larger ones are "
        "downweighted. Default: 1.5x the median translation deviation from the averaged result.",
    )
    return parser.parse_args()


# --------------------------------------------------------------------------------------
# Stage 1: alignment
# --------------------------------------------------------------------------------------


def read_points_csv(path: Path) -> List[Dict[str, float]]:
    rows: List[Dict[str, float]] = []
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"marker_id", "kf_id", "u", "v", "marker_x_m", "marker_y_m", "mp_x", "mp_y", "mp_z"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(f"{path} is missing columns: {', '.join(sorted(missing))}")
        for row in reader:
            rows.append(
                {
                    "marker_id": int(float(row["marker_id"])),
                    "kf_id": int(float(row["kf_id"])),
                    "u": float(row["u"]),
                    "v": float(row["v"]),
                    "marker_x_m": float(row["marker_x_m"]),
                    "marker_y_m": float(row["marker_y_m"]),
                    "mp_x": float(row["mp_x"]),
                    "mp_y": float(row["mp_y"]),
                    "mp_z": float(row["mp_z"]),
                }
            )
    return rows


def keyframe_counts_by_marker(rows: List[Dict[str, float]]) -> Dict[int, int]:
    marker_to_kfs: Dict[int, set] = {}
    for row in rows:
        marker_to_kfs.setdefault(row["marker_id"], set()).add(row["kf_id"])
    return {marker_id: len(kfs) for marker_id, kfs in marker_to_kfs.items()}


def choose_marker_id(
    rows1: List[Dict[str, float]],
    rows2: List[Dict[str, float]],
    name1: str,
    name2: str,
    forced_marker_id: Optional[int],
) -> int:
    counts1 = keyframe_counts_by_marker(rows1)
    counts2 = keyframe_counts_by_marker(rows2)
    all_markers = sorted(set(counts1) | set(counts2))

    print(f"{'marker_id':>9}  {name1 + '_kfs':>12}  {name2 + '_kfs':>12}")
    for marker_id in all_markers:
        print(f"{marker_id:9d}  {counts1.get(marker_id, 0):12d}  {counts2.get(marker_id, 0):12d}")

    if forced_marker_id is not None:
        if not counts1.get(forced_marker_id) or not counts2.get(forced_marker_id):
            raise SystemExit(f"marker_id {forced_marker_id} is not observed by both {name1} and {name2}")
        return forced_marker_id

    shared = [m for m in all_markers if counts1.get(m, 0) > 0 and counts2.get(m, 0) > 0]
    if not shared:
        raise SystemExit(f"No marker id is observed by both {name1} and {name2}")
    best = max(shared, key=lambda m: counts1[m] + counts2[m])
    print(f"Selected marker_id={best} ({name1}: {counts1[best]} kfs, {name2}: {counts2[best]} kfs)")
    return best


def solve_pnp(
    object_points: np.ndarray,
    image_points: np.ndarray,
    K: np.ndarray,
    D: np.ndarray,
    prefer_planar: bool,
) -> Tuple[np.ndarray, np.ndarray, float]:
    object_points = np.ascontiguousarray(object_points, dtype=np.float64).reshape(-1, 1, 3)
    image_points = np.ascontiguousarray(image_points, dtype=np.float64).reshape(-1, 1, 2)
    flags_to_try = [cv2.SOLVEPNP_IPPE, cv2.SOLVEPNP_EPNP] if prefer_planar else [cv2.SOLVEPNP_EPNP]

    last_error: Optional[Exception] = None
    for flags in flags_to_try:
        try:
            ok, rvec, tvec = cv2.solvePnP(object_points, image_points, K, D, flags=flags)
        except cv2.error as exc:  # some flags reject degenerate point configs
            last_error = exc
            continue
        if not ok:
            continue
        try:
            rvec, tvec = cv2.solvePnPRefineLM(object_points, image_points, K, D, rvec, tvec)
        except cv2.error:
            pass
        R, _ = cv2.Rodrigues(rvec)
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, K, D)
        residual = projected.reshape(-1, 2) - image_points.reshape(-1, 2)
        rms_px = float(np.sqrt(np.mean(np.sum(residual**2, axis=1))))
        return R, tvec.reshape(3), rms_px

    raise RuntimeError(f"solvePnP failed for all flags tried (last error: {last_error})")


def select_best_keyframe(
    name: str,
    rows: List[Dict[str, float]],
    marker_id: int,
    K: np.ndarray,
    D: np.ndarray,
    top_k: int,
) -> Tuple[int, int]:
    """Take the top-k keyframes by marker point count, run the marker-only PnP
    for each, and keep whichever gives the lowest reprojection RMS."""
    counts = Counter(row["kf_id"] for row in rows if row["marker_id"] == marker_id)
    if not counts:
        raise ValueError(f"No keyframes observed marker_id={marker_id}")

    candidates = counts.most_common(top_k)
    print(f"{name}: evaluating {len(candidates)} keyframe candidate(s) for marker {marker_id}")

    best: Optional[Tuple[int, int, float]] = None
    for kf_id, num_points in candidates:
        marker_rows = [row for row in rows if row["marker_id"] == marker_id and row["kf_id"] == kf_id]
        real_points = np.array([[r["marker_x_m"], r["marker_y_m"], 0.0] for r in marker_rows])
        image_points = np.array([[r["u"], r["v"]] for r in marker_rows])
        try:
            _, _, rms_px = solve_pnp(real_points, image_points, K, D, prefer_planar=True)
        except RuntimeError as exc:
            print(f"  kf{kf_id}: {num_points} pts, PnP failed ({exc})")
            continue
        print(f"  kf{kf_id}: {num_points} pts, marker PnP RMS = {rms_px:.3f}px")
        if best is None or rms_px < best[2]:
            best = (kf_id, num_points, rms_px)

    if best is None:
        raise ValueError(f"All keyframe candidates for marker {marker_id} failed PnP")

    kf_id, num_points, rms_px = best
    print(f"{name}: selected kf_id={kf_id} ({num_points} pts, RMS={rms_px:.3f}px)")
    return kf_id, num_points


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = q / np.linalg.norm(q)
    return np.array(
        [
            [1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)],
        ],
        dtype=np.float64,
    )


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    m = R
    tr = m[0, 0] + m[1, 1] + m[2, 2]
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (m[2, 1] - m[1, 2]) / S
        qy = (m[0, 2] - m[2, 0]) / S
        qz = (m[1, 0] - m[0, 1]) / S
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        S = np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2]) * 2
        qw = (m[2, 1] - m[1, 2]) / S
        qx = 0.25 * S
        qy = (m[0, 1] + m[1, 0]) / S
        qz = (m[0, 2] + m[2, 0]) / S
    elif m[1, 1] > m[2, 2]:
        S = np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2]) * 2
        qw = (m[0, 2] - m[2, 0]) / S
        qx = (m[0, 1] + m[1, 0]) / S
        qy = 0.25 * S
        qz = (m[1, 2] + m[2, 1]) / S
    else:
        S = np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1]) * 2
        qw = (m[1, 0] - m[0, 1]) / S
        qx = (m[0, 2] + m[2, 0]) / S
        qy = (m[1, 2] + m[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qw, qx, qy, qz])
    return q / np.linalg.norm(q)


def load_trajectory(path: Path) -> Dict[int, Tuple[float, np.ndarray, np.ndarray]]:
    """Single pass over the raw observations CSV: kf_id -> (timestamp, camera_center, R_cw).

    R_cw is the keyframe's own SLAM-frame world-to-camera rotation
    (p_cam = R_cw @ p_world + t_cw), straight from ORB-SLAM3's KeyFrame::GetRotation()
    via the qw/qx/qy/qz columns written by atlas_export_observations.
    """
    trajectory: Dict[int, Tuple[float, np.ndarray, np.ndarray]] = {}
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        required = {"kf_id", "timestamp", "camera_x", "camera_y", "camera_z", "qw", "qx", "qy", "qz"}
        missing = required - set(reader.fieldnames or [])
        if missing:
            raise ValueError(
                f"{path} is missing columns: {', '.join(sorted(missing))}. "
                "Re-export it with the updated atlas_export_observations."
            )
        for row in reader:
            kf_id = int(row["kf_id"])
            if kf_id in trajectory:
                continue
            center = np.array([float(row["camera_x"]), float(row["camera_y"]), float(row["camera_z"])])
            q = np.array([float(row["qw"]), float(row["qx"]), float(row["qy"]), float(row["qz"])])
            trajectory[kf_id] = (float(row["timestamp"]), center, quaternion_to_matrix(q))
    return trajectory


def estimate_scale(real_points: np.ndarray, slam_points: np.ndarray) -> float:
    n = len(real_points)
    ratios: List[float] = []
    for i in range(n):
        for j in range(i + 1, n):
            real_dist = float(np.linalg.norm(real_points[i] - real_points[j]))
            slam_dist = float(np.linalg.norm(slam_points[i] - slam_points[j]))
            if real_dist > 1e-4 and slam_dist > 1e-7:
                ratios.append(real_dist / slam_dist)
    if not ratios:
        raise ValueError("Could not estimate scale: no usable point pairs")
    return float(np.median(ratios))


def align_camera(
    name: str,
    rows: List[Dict[str, float]],
    marker_id: int,
    kf_id: int,
    raw_csv_path: Path,
    calib: CameraCalibration,
    external_scale: Optional[float] = None,
) -> Dict[str, object]:
    K, D = calib.K, calib.D

    marker_rows = [row for row in rows if row["marker_id"] == marker_id and row["kf_id"] == kf_id]
    real_points = np.array([[r["marker_x_m"], r["marker_y_m"], 0.0] for r in marker_rows])
    slam_points_marker = np.array([[r["mp_x"], r["mp_y"], r["mp_z"]] for r in marker_rows])
    image_points_marker = np.array([[r["u"], r["v"]] for r in marker_rows])

    R_marker, t_marker, err_marker_px = solve_pnp(real_points, image_points_marker, K, D, prefer_planar=True)
    print(f"{name}: marker PnP reprojection RMS = {err_marker_px:.3f}px ({len(marker_rows)} pts)")

    trajectory = load_trajectory(raw_csv_path)
    if kf_id not in trajectory:
        raise ValueError(f"{name}: kf_id={kf_id} not found in {raw_csv_path}")

    if external_scale is not None:
        scale = external_scale
        print(f"{name}: using provided scale = {scale:.6f} m per SLAM unit (not extracted from anchor keyframe)")
    else:
        scale = estimate_scale(real_points, slam_points_marker)
        print(f"{name}: recovered scale = {scale:.6f} m per SLAM unit (from marker point pairs at anchor keyframe)")

    anchor_timestamp, camera_center_slam_anchor, R_slam_anchor = trajectory[kf_id]

    # p_cam = R_marker @ p_marker + t_marker  and  p_cam = R_slam_anchor @ p_slam + t_slam_anchor
    # (both world-to-camera rotations for the same physical camera pose at kf_id)
    R_slam_to_marker = R_marker.T @ R_slam_anchor
    camera_center_marker = -R_marker.T @ t_marker
    t_slam_to_marker = camera_center_marker - scale * (R_slam_to_marker @ camera_center_slam_anchor)

    items = sorted(trajectory.items(), key=lambda kv: kv[1][0])
    kf_ids_sorted = np.array([k for k, _ in items], dtype=np.int64)
    timestamps = np.array([v[0] for _, v in items], dtype=np.float64)
    centers_slam = np.array([v[1] for _, v in items], dtype=np.float64)
    rotations_slam = np.array([v[2] for _, v in items], dtype=np.float64)

    positions_marker = scale * (centers_slam @ R_slam_to_marker.T) + t_slam_to_marker
    # Rcw_marker[n] = Rcw_slam[n] @ R_slam_to_marker.T (verified against the anchor: at
    # kf_id this reduces exactly back to R_marker).
    rotations_marker = rotations_slam @ R_slam_to_marker.T
    anchor_check = scale * (R_slam_to_marker @ camera_center_slam_anchor) + t_slam_to_marker

    return {
        "name": name,
        "marker_id": marker_id,
        "kf_id": kf_id,
        "scale": scale,
        "scale_source": "external" if external_scale is not None else "anchor_marker_points",
        "R_slam_to_marker": R_slam_to_marker,
        "t_slam_to_marker": t_slam_to_marker,
        "trajectory_kf_ids": kf_ids_sorted,
        "trajectory_timestamps": timestamps,
        "trajectory": positions_marker,
        "trajectory_rotations": rotations_marker,
        "anchor_point": anchor_check,
        "anchor_timestamp": anchor_timestamp,
        "num_marker_points": len(marker_rows),
        "marker_pnp_rms_px": err_marker_px,
    }


def plot_trajectories(results: List[Dict[str, object]], marker_id: int, marker_length_m: float, output_plot: Path, show: bool) -> None:
    import matplotlib

    if not show:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt  # noqa: E402

    fig = plt.figure(figsize=(9, 8))
    ax = fig.add_subplot(111, projection="3d")

    half = marker_length_m * 0.5
    square = np.array(
        [[-half, half, 0.0], [half, half, 0.0], [half, -half, 0.0], [-half, -half, 0.0], [-half, half, 0.0]]
    )
    ax.plot(square[:, 0], square[:, 1], square[:, 2], color="black", linewidth=2, label=f"marker {marker_id}")

    colors = ["tab:blue", "tab:orange", "tab:green", "tab:red"]
    all_points = [square]
    for i, res in enumerate(results):
        color = colors[i % len(colors)]
        traj = res["trajectory"]
        ax.plot(traj[:, 0], traj[:, 1], traj[:, 2], color=color, linewidth=1.5, label=f"{res['name']} trajectory")
        anchor = res["anchor_point"]
        ax.scatter(anchor[0], anchor[1], anchor[2], color=color, marker="*", s=220, edgecolor="black",
                    label=f"{res['name']} kf{res['kf_id']} anchor")
        all_points.append(traj)

    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.set_title(f"Camera trajectories in ArUco marker {marker_id} frame")
    ax.legend(loc="upper left", fontsize=8)

    stacked = np.vstack(all_points)
    center = stacked.mean(axis=0)
    radius = float(np.max(np.linalg.norm(stacked - center, axis=1))) or 1.0
    ax.set_xlim(center[0] - radius, center[0] + radius)
    ax.set_ylim(center[1] - radius, center[1] + radius)
    ax.set_zlim(center[2] - radius, center[2] + radius)
    if hasattr(ax, "set_box_aspect"):
        ax.set_box_aspect((1, 1, 1))

    output_plot.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output_plot, dpi=150, bbox_inches="tight")
    print(f"Saved plot: {output_plot}")
    if show:
        plt.show()


# --------------------------------------------------------------------------------------
# Stage 2: extrinsic calibration (reuses stage 1's align_camera() results)
# --------------------------------------------------------------------------------------


def slerp(q0: np.ndarray, q1: np.ndarray, t: float) -> np.ndarray:
    q0 = q0 / np.linalg.norm(q0)
    q1 = q1 / np.linalg.norm(q1)
    dot = float(np.dot(q0, q1))
    if dot < 0:
        q1 = -q1
        dot = -dot
    dot = min(max(dot, -1.0), 1.0)
    if dot > 0.9995:
        result = q0 + t * (q1 - q0)
        return result / np.linalg.norm(result)
    theta0 = np.arccos(dot)
    theta = theta0 * t
    q2 = q1 - q0 * dot
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


def interpolate_pose(
    timestamps: np.ndarray, positions: np.ndarray, rotations: np.ndarray, query_ts: float
) -> Optional[Tuple[np.ndarray, np.ndarray, float]]:
    """Returns (position, rotation, bracket_width_s) interpolated at query_ts, or
    None if query_ts falls outside [timestamps[0], timestamps[-1]]."""
    if query_ts < timestamps[0] or query_ts > timestamps[-1]:
        return None
    i = int(np.searchsorted(timestamps, query_ts))
    if i == 0:
        i = 1
    t0, t1 = timestamps[i - 1], timestamps[i]
    alpha = 0.0 if t1 == t0 else (query_ts - t0) / (t1 - t0)
    pos = positions[i - 1] * (1 - alpha) + positions[i] * alpha
    q0 = matrix_to_quaternion(rotations[i - 1])
    q1 = matrix_to_quaternion(rotations[i])
    R = quaternion_to_matrix(slerp(q0, q1, alpha))
    return pos, R, float(t1 - t0)


def relative_extrinsic(R_a: np.ndarray, C_a: np.ndarray, R_b: np.ndarray, C_b: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """p_a_cam = R_b_to_a @ p_b_cam + t_b_to_a, given two world-to-camera rotations
    (R_a, R_b) and CAMERA CENTERS (C_a, C_b, i.e. positions, not the tcw translation
    vector) expressed in the same world (marker) frame.

    p_x_cam = R_x @ (p_world - C_x), so substituting p_world = R_b.T @ p_b_cam + C_b
    into camera a's equation gives R_b_to_a = R_a @ R_b.T and
    t_b_to_a = R_a @ (C_b - C_a) -- NOT "C_a - R_b_to_a @ C_b", which would be correct
    only if C_a/C_b were already the tcw vectors (t = -R @ C) rather than centers.
    """
    R_b_to_a = R_a @ R_b.T
    t_b_to_a = R_a @ (C_b - C_a)
    return R_b_to_a, t_b_to_a


def rotation_chordal_mean(rotations: np.ndarray) -> np.ndarray:
    M = rotations.mean(axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R


def rotation_angle_deg(R_a: np.ndarray, R_b: np.ndarray) -> float:
    R_diff = R_a @ R_b.T
    cos_angle = (np.trace(R_diff) - 1) / 2
    cos_angle = min(max(cos_angle, -1.0), 1.0)
    return float(np.degrees(np.arccos(cos_angle)))


def summarize(rotations: np.ndarray, translations: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    R_mean = rotation_chordal_mean(rotations)
    t_median = np.median(translations, axis=0)
    angle_devs = np.array([rotation_angle_deg(R, R_mean) for R in rotations])
    trans_devs = np.linalg.norm(translations - t_median, axis=1)
    return R_mean, t_median, angle_devs, trans_devs


def optimize_extrinsic(
    rotations: np.ndarray,
    translations: np.ndarray,
    R_init: np.ndarray,
    t_init: np.ndarray,
    loss: str = "huber",
    f_scale_m: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    """Stage 3: joint robust refinement over SE(3). Each match's relative_extrinsic()
    result (rotations[i], translations[i]) is already a complete, closed-form estimate of
    the same camera2->camera1 extrinsic -- there are no free unknowns entangled in raw
    poses left to solve jointly (unlike e.g. optim.txt's multi-tag reprojection problem,
    where the offsets genuinely couple through a nonlinear reprojection function).
    summarize() above instead combines the N noisy per-match estimates with two
    independent, mismatched rules: an SVD chordal mean for rotation and a per-axis median
    for translation. This replaces that with a single (R, t) minimizing one combined
    robust (default Huber) loss over rotation and translation residuals *together*, per
    match -- a joint M-estimator on SE(3) instead of two separately-reduced statistics,
    smoothly downweighting outlier matches instead of relying only on the hard
    anchor-distance/time cutoffs upstream. Parameterized as rvec (axis-angle,
    unconstrained) + t, same pattern as optim.txt's packT/unpackT via
    rotm2axang/axang2rotm.
    """
    n = len(rotations)

    # Rotation residuals are naturally in radians and translation residuals in meters --
    # not comparable units. Scale rotation residuals by the ratio of their typical (median)
    # magnitude to translation's, at the initial guess, so neither term dominates the
    # objective purely because of unit choice rather than actual fit quality.
    rot_res0_deg = np.array([rotation_angle_deg(R_init, rotations[i]) for i in range(n)])
    trans_res0 = np.linalg.norm(translations - t_init, axis=1)
    rot_scale = float(np.median(rot_res0_deg)) * np.pi / 180.0
    trans_scale = float(np.median(trans_res0))
    rot_weight = trans_scale / rot_scale if rot_scale > 1e-9 else 1.0

    if f_scale_m is None:
        f_scale_m = max(1.5 * trans_scale, 1e-4)

    def residuals(x: np.ndarray) -> np.ndarray:
        R, _ = cv2.Rodrigues(x[:3])
        t = x[3:]
        res = np.empty((n, 6))
        for i in range(n):
            rot_err_vec, _ = cv2.Rodrigues(R.T @ rotations[i])
            res[i, :3] = rot_weight * rot_err_vec.flatten()
            res[i, 3:] = t - translations[i]
        return res.flatten()

    rvec_init, _ = cv2.Rodrigues(R_init)
    x0 = np.concatenate([rvec_init.flatten(), t_init])

    initial_cost = float(0.5 * np.sum(residuals(x0) ** 2))
    result = least_squares(residuals, x0, loss=loss, f_scale=f_scale_m, method="trf")

    R_opt, _ = cv2.Rodrigues(result.x[:3])
    t_opt = result.x[3:]

    diagnostics = {
        "loss": loss,
        "f_scale_m": float(f_scale_m),
        "rot_weight": float(rot_weight),
        "success": bool(result.success),
        "nfev": int(result.nfev),
        "initial_cost": initial_cost,
        "final_cost": float(result.cost),
        "rotation_change_from_average_deg": rotation_angle_deg(R_opt, R_init),
        "translation_change_from_average_m": float(np.linalg.norm(t_opt - t_init)),
    }
    return R_opt, t_opt, diagnostics


def _robust_rho(loss: str, z: np.ndarray) -> np.ndarray:
    """Same rho(z) family scipy.optimize.least_squares uses internally (z = (r/f_scale)^2),
    exposed here so optimize_extrinsic_grouped() can apply it to a *combined* per-match
    residual norm instead of per-component, while staying directly comparable (same loss
    shape, same f_scale) to optimize_extrinsic()'s result."""
    if loss == "linear":
        return z
    if loss == "huber":
        return np.where(z <= 1, z, 2 * np.sqrt(z) - 1)
    if loss == "soft_l1":
        return 2 * (np.sqrt(1 + z) - 1)
    if loss == "cauchy":
        return np.log1p(z)
    if loss == "arctan":
        return np.arctan(z)
    raise ValueError(f"Unknown loss: {loss}")


def optimize_extrinsic_grouped(
    rotations: np.ndarray,
    translations: np.ndarray,
    R_init: np.ndarray,
    t_init: np.ndarray,
    loss: str = "huber",
    f_scale_m: Optional[float] = None,
) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
    """Stage 3b: joint SE(3) refinement with a *grouped* per-match robust loss, solved
    with a generic scalar minimizer (scipy.optimize.minimize, BFGS -- the Python analog
    of MATLAB's fminunc) instead of scipy.optimize.least_squares.

    optimize_extrinsic() above applies the robust loss to each of the 6 residual
    *components* (3 rotation + 3 translation) independently, because that is what
    least_squares's API supports -- a match could have one component judged an inlier
    and another an outlier. This instead computes ONE combined 6D residual norm per
    match and applies the robust loss to that single number, so a match is downweighted
    or effectively rejected as a whole -- exactly the thing a generic minimizer can do
    that least_squares's per-component loss structurally cannot. Same rho(z) family and
    f_scale as optimize_extrinsic() so the two are directly comparable.
    """
    n = len(rotations)
    rot_res0_deg = np.array([rotation_angle_deg(R_init, rotations[i]) for i in range(n)])
    trans_res0 = np.linalg.norm(translations - t_init, axis=1)
    rot_scale = float(np.median(rot_res0_deg)) * np.pi / 180.0
    trans_scale = float(np.median(trans_res0))
    rot_weight = trans_scale / rot_scale if rot_scale > 1e-9 else 1.0

    if f_scale_m is None:
        f_scale_m = max(1.5 * trans_scale, 1e-4)

    def match_residual_norms(x: np.ndarray) -> np.ndarray:
        R, _ = cv2.Rodrigues(x[:3])
        t = x[3:]
        norms = np.empty(n)
        for i in range(n):
            rot_err_vec, _ = cv2.Rodrigues(R.T @ rotations[i])
            res6 = np.concatenate([rot_weight * rot_err_vec.flatten(), t - translations[i]])
            norms[i] = np.linalg.norm(res6)
        return norms

    def cost(x: np.ndarray) -> float:
        norms = match_residual_norms(x)
        z = (norms / f_scale_m) ** 2
        return float(0.5 * f_scale_m**2 * np.sum(_robust_rho(loss, z)))

    rvec_init, _ = cv2.Rodrigues(R_init)
    x0 = np.concatenate([rvec_init.flatten(), t_init])
    initial_cost = cost(x0)

    # L-BFGS-B (unconstrained here, no bounds given) rather than plain BFGS: with a
    # numerically-differentiated gradient (cost() has no analytical jac), BFGS's line
    # search reports "precision loss" / success=False right at the true optimum once the
    # finite-difference gradient noise floor is reached, even though the solution is
    # correct (verified: same cost/solution as BFGS, but with a clean convergence flag).
    result = minimize(cost, x0, method="L-BFGS-B", options={"maxiter": 1000, "ftol": 1e-15, "gtol": 1e-12})

    R_opt, _ = cv2.Rodrigues(result.x[:3])
    t_opt = result.x[3:]

    diagnostics = {
        "loss": loss,
        "f_scale_m": float(f_scale_m),
        "rot_weight": float(rot_weight),
        "success": bool(result.success),
        "nfev": int(result.nfev),
        "initial_cost": initial_cost,
        "final_cost": float(result.fun),
        "rotation_change_from_average_deg": rotation_angle_deg(R_opt, R_init),
        "translation_change_from_average_m": float(np.linalg.norm(t_opt - t_init)),
    }
    return R_opt, t_opt, diagnostics


def rotation_matrix_to_euler_zyx_deg(R: np.ndarray) -> Tuple[float, float, float]:
    """Inverse of R = Rz(yaw) @ Ry(pitch) @ Rx(roll) -- the roll/pitch/yaw convention
    used by Agilex Recordings/Intrinsic_ground_truths/robot_relative_extrinsics.yaml
    (verified against its T_front_left entry to float precision)."""
    pitch = -np.arcsin(np.clip(R[2, 0], -1.0, 1.0))
    roll = np.arctan2(R[2, 1], R[2, 2])
    yaw = np.arctan2(R[1, 0], R[0, 0])
    return float(np.degrees(roll)), float(np.degrees(pitch)), float(np.degrees(yaw))


def format_extrinsic_yaml(camera1: str, camera2: str, R: np.ndarray, t: np.ndarray, suffix: str = "") -> str:
    """Formats camera2->camera1 as T_<camera1>_<camera2>, matching
    robot_relative_extrinsics.yaml's layout exactly (from_frame/to_frame/
    translation_xyz/euler_zyx_deg/matrix). suffix (e.g. "_optimized") is appended to the
    block's key only, so the stage-2 (averaged) and stage-3 (optimized) results can be
    saved side by side in the same file without colliding."""
    roll, pitch, yaw = rotation_matrix_to_euler_zyx_deg(R)

    def f(x: float) -> str:
        return f"{x:.12f}"

    lines = [
        f"T_{camera1}_{camera2}{suffix}:",
        f"    from_frame: {camera2}",
        f"    to_frame: {camera1}",
        f"    translation_xyz: [{f(t[0])}, {f(t[1])}, {f(t[2])}]",
        "    euler_zyx_deg:",
        f"      roll: {f(roll)}",
        f"      pitch: {f(pitch)}",
        f"      yaw: {f(yaw)}",
        "    matrix:",
    ]
    for i in range(3):
        lines.append(f"      - [{f(R[i, 0])}, {f(R[i, 1])}, {f(R[i, 2])}, {f(t[i])}]")
    lines.append(f"      - [{f(0.0)}, {f(0.0)}, {f(0.0)}, {f(1.0)}]")
    return "\n".join(lines) + "\n"


def compute_extrinsic(
    args: argparse.Namespace, marker_id: int, result1: Dict[str, object], result2: Dict[str, object]
) -> Tuple[str, Dict[str, object]]:
    ts1, pos1, rot1 = result1["trajectory_timestamps"], result1["trajectory"], result1["trajectory_rotations"]
    ts2, pos2, rot2 = result2["trajectory_timestamps"], result2["trajectory"], result2["trajectory_rotations"]
    kf_ids2 = result2["trajectory_kf_ids"]
    anchor1, anchor2 = result1["anchor_point"], result2["anchor_point"]
    anchor1_ts, anchor2_ts = result1["anchor_timestamp"], result2["anchor_timestamp"]

    # First pass: collect every candidate match (only gated by --max-bracket-width-s, a cheap
    # and independent quality gate), keeping distance-from-anchor and time-from-anchor as
    # metadata rather than filtering by them yet -- otherwise the sensitivity sweeps below would
    # be comparing a cutoff against a set that's already been cut to a tighter one.
    matches: List[Dict[str, object]] = []
    skipped_out_of_range = 0
    skipped_bracket = 0
    for idx2 in range(len(ts2)):
        t = ts2[idx2]
        interp = interpolate_pose(ts1, pos1, rot1, t)
        if interp is None:
            skipped_out_of_range += 1
            continue
        pos1_interp, rot1_interp, bracket_width = interp
        if args.max_bracket_width_s is not None and bracket_width > args.max_bracket_width_s:
            skipped_bracket += 1
            continue
        distance_from_anchor = max(
            float(np.linalg.norm(pos1_interp - anchor1)), float(np.linalg.norm(pos2[idx2] - anchor2))
        )
        # Euclidean distance is only a proxy for accumulated SLAM drift -- drift tracks
        # trajectory/time distance from the anchor, not physical proximity, so a keyframe can
        # revisit the anchor's location much later (Euclidean-close) while carrying far more
        # drift than the distance filter alone would catch. Time-from-anchor is independent.
        time_from_anchor = max(abs(t - anchor1_ts), abs(float(ts2[idx2]) - anchor2_ts))
        R_rel, t_rel = relative_extrinsic(rot1_interp, pos1_interp, rot2[idx2], pos2[idx2])
        matches.append(
            {
                "kf2_id": int(kf_ids2[idx2]),
                "timestamp": float(t),
                "bracket_width_s": bracket_width,
                "distance_from_anchor_m": distance_from_anchor,
                "time_from_anchor_s": time_from_anchor,
                "R": R_rel,
                "t": t_rel,
            }
        )

    print(
        f"\n{len(ts2)} {args.camera2_name} keyframes; {skipped_out_of_range} outside {args.camera1_name}'s "
        f"time span; {skipped_bracket} dropped by --max-bracket-width-s; {len(matches)} candidate matches"
    )
    if len(matches) < 3:
        raise SystemExit("Not enough matches to compute a robust extrinsic estimate")

    rotations = np.array([m["R"] for m in matches])
    translations = np.array([m["t"] for m in matches])
    distances = np.array([m["distance_from_anchor_m"] for m in matches])
    time_distances = np.array([m["time_from_anchor_s"] for m in matches])

    print(
        "\nDistance from anchor keyframe is one quality signal (monocular SLAM scale/pose drift "
        "grows with distance from the marker anchor). Sensitivity to that cutoff, over all "
        f"{len(matches)} candidate matches:"
    )
    print(f"  {'cutoff(m)':>10}  {'n':>5}  {'|t| median(m)':>15}  {'rot_dev_median(deg)':>20}  {'trans_dev_median(mm)':>20}")
    for cutoff in sorted({0.5, 1.0, 2.0, 5.0, float(distances.max())}):
        mask = distances <= cutoff
        if mask.sum() < 3:
            continue
        _, t_med, a, d = summarize(rotations[mask], translations[mask])
        print(f"  {cutoff:10.2f}  {mask.sum():5d}  {np.linalg.norm(t_med):15.3f}  {np.median(a):20.3f}  {np.median(d) * 1000:20.1f}")

    print(
        "\nTime from anchor keyframe is an independent quality signal (drift accumulates with "
        "trajectory distance, which Euclidean distance alone can miss -- e.g. a later revisit of "
        f"the anchor's location). Sensitivity to that cutoff, over all {len(matches)} candidate matches:"
    )
    print(f"  {'cutoff(s)':>10}  {'n':>5}  {'|t| median(m)':>15}  {'rot_dev_median(deg)':>20}  {'trans_dev_median(mm)':>20}")
    for cutoff in sorted({1.0, 5.0, 15.0, 30.0, float(time_distances.max())}):
        mask = time_distances <= cutoff
        if mask.sum() < 3:
            continue
        _, t_med, a, d = summarize(rotations[mask], translations[mask])
        print(f"  {cutoff:10.2f}  {mask.sum():5d}  {np.linalg.norm(t_med):15.3f}  {np.median(a):20.3f}  {np.median(d) * 1000:20.1f}")

    # Second pass: apply the user's chosen cutoffs (if any) to select the matches that actually
    # determine the reported/saved extrinsic. Both filters are independent and combined with AND.
    final_mask = np.ones(len(matches), dtype=bool)
    if args.max_distance_from_anchor_m is not None:
        final_mask &= distances <= args.max_distance_from_anchor_m
    if args.max_time_from_anchor_s is not None:
        final_mask &= time_distances <= args.max_time_from_anchor_s
    skipped_distance = int((~final_mask).sum())
    rotations, translations, distances, time_distances = (
        rotations[final_mask],
        translations[final_mask],
        distances[final_mask],
        time_distances[final_mask],
    )
    matches = [m for m, keep in zip(matches, final_mask) if keep]
    print(
        f"\n{skipped_distance} candidate matches dropped by --max-distance-from-anchor-m / "
        f"--max-time-from-anchor-s; {len(matches)} used for the final extrinsic"
    )
    if len(matches) < 3:
        raise SystemExit("Not enough matches survive the anchor-distance/time cutoffs for a robust estimate")

    R_mean, t_median, angle_devs, trans_devs = summarize(rotations, translations)
    print(
        f"\nUsing all {len(matches)} matches: rotation deviation from mean "
        f"(deg) median={np.median(angle_devs):.3f} mean={angle_devs.mean():.3f} max={angle_devs.max():.3f}"
    )
    print(
        f"translation deviation from median (m) median={np.median(trans_devs):.4f} "
        f"mean={trans_devs.mean():.4f} max={trans_devs.max():.4f}"
    )

    R_opt, t_opt, opt_diag = optimize_extrinsic(
        rotations, translations, R_mean, t_median, loss=args.optimize_loss, f_scale_m=args.optimize_f_scale_m
    )
    print(
        f"\nStage 3 joint optimization ({opt_diag['loss']} loss, f_scale={opt_diag['f_scale_m'] * 1000:.1f}mm): "
        f"cost {opt_diag['initial_cost']:.6f} -> {opt_diag['final_cost']:.6f} "
        f"({opt_diag['nfev']} evals, success={opt_diag['success']}); moved "
        f"{opt_diag['rotation_change_from_average_deg']:.3f} deg / "
        f"{opt_diag['translation_change_from_average_m'] * 1000:.1f} mm from the averaged (stage 2) result"
    )

    R_grp, t_grp, grp_diag = optimize_extrinsic_grouped(
        rotations, translations, R_mean, t_median, loss=args.optimize_loss, f_scale_m=args.optimize_f_scale_m
    )
    print(
        f"\nStage 3b grouped joint optimization ({grp_diag['loss']} loss, f_scale={grp_diag['f_scale_m'] * 1000:.1f}mm): "
        f"cost {grp_diag['initial_cost']:.6f} -> {grp_diag['final_cost']:.6f} "
        f"({grp_diag['nfev']} evals, success={grp_diag['success']}); moved "
        f"{grp_diag['rotation_change_from_average_deg']:.3f} deg / "
        f"{grp_diag['translation_change_from_average_m'] * 1000:.1f} mm from the averaged (stage 2) result"
    )

    extrinsic_yaml_averaged = format_extrinsic_yaml(args.camera1_name, args.camera2_name, R_mean, t_median)
    extrinsic_yaml_optimized = format_extrinsic_yaml(
        args.camera1_name, args.camera2_name, R_opt, t_opt, suffix="_optimized"
    )
    extrinsic_yaml_grouped = format_extrinsic_yaml(
        args.camera1_name, args.camera2_name, R_grp, t_grp, suffix="_optimized_grouped"
    )
    print(f"\n{extrinsic_yaml_averaged}")
    print(f"\n{extrinsic_yaml_optimized}")
    print(f"\n{extrinsic_yaml_grouped}")
    extrinsic_yaml = extrinsic_yaml_averaged + "\n" + extrinsic_yaml_optimized + "\n" + extrinsic_yaml_grouped

    diagnostics = {
        "marker_id": marker_id,
        "camera1": args.camera1_name,
        "camera2": args.camera2_name,
        "num_camera2_keyframes": int(len(ts2)),
        "num_skipped_out_of_range": int(skipped_out_of_range),
        "num_skipped_bracket_width": int(skipped_bracket),
        "num_skipped_anchor_distance_or_time": int(skipped_distance),
        "num_matches_used": int(len(matches)),
        "max_bracket_width_s": args.max_bracket_width_s,
        "max_distance_from_anchor_m": args.max_distance_from_anchor_m,
        "max_time_from_anchor_s": args.max_time_from_anchor_s,
        "rotation_deviation_from_mean_deg": {
            "median": float(np.median(angle_devs)),
            "mean": float(angle_devs.mean()),
            "max": float(angle_devs.max()),
        },
        "translation_deviation_from_median_m": {
            "median": float(np.median(trans_devs)),
            "mean": float(trans_devs.mean()),
            "max": float(trans_devs.max()),
        },
        "optimization": opt_diag,
        "optimization_grouped": grp_diag,
    }
    return extrinsic_yaml, diagnostics


def main() -> int:
    args = parse_args()

    run_dir = args.camera1_raw_csv.parent
    alignment_dir = run_dir / "aruco_alignment"
    pair = f"{args.camera1_name}_{args.camera2_name}"
    output_plot = args.output_plot or alignment_dir / f"{pair}_trajectories.png"
    output_alignment_json = args.output_alignment_json or alignment_dir / f"{pair}_alignment.json"
    output_extrinsic_yaml = args.output_extrinsic_yaml or alignment_dir / f"{pair}_extrinsic.yaml"

    rows1 = read_points_csv(args.camera1_points_csv)
    rows2 = read_points_csv(args.camera2_points_csv)
    calib1 = read_calibration(args.camera1_config)
    calib2 = read_calibration(args.camera2_config)

    marker_id = choose_marker_id(rows1, rows2, args.camera1_name, args.camera2_name, args.marker_id)
    kf1, _ = select_best_keyframe(args.camera1_name, rows1, marker_id, calib1.K, calib1.D, args.keyframe_candidates)
    kf2, _ = select_best_keyframe(args.camera2_name, rows2, marker_id, calib2.K, calib2.D, args.keyframe_candidates)

    # Computed once, reused by both stages below.
    result1 = align_camera(args.camera1_name, rows1, marker_id, kf1, args.camera1_raw_csv, calib1, args.camera1_scale)
    result2 = align_camera(args.camera2_name, rows2, marker_id, kf2, args.camera2_raw_csv, calib2, args.camera2_scale)

    # --- Stage 1 outputs ---
    plot_trajectories([result1, result2], marker_id, args.marker_length_m, output_plot, args.show)

    alignment_summary = {
        "marker_id": marker_id,
        "cameras": [
            {
                "name": res["name"],
                "keyframe_id": res["kf_id"],
                "num_marker_points_used": res["num_marker_points"],
                "marker_pnp_reprojection_rms_px": res["marker_pnp_rms_px"],
                "scale_m_per_slam_unit": res["scale"],
                "scale_source": res["scale_source"],
                "rotation_slam_to_marker": res["R_slam_to_marker"].tolist(),
                "translation_slam_to_marker_m": res["t_slam_to_marker"].tolist(),
                "trajectory_num_keyframes": int(res["trajectory"].shape[0]),
            }
            for res in (result1, result2)
        ],
    }
    output_alignment_json.parent.mkdir(parents=True, exist_ok=True)
    with output_alignment_json.open("w") as handle:
        json.dump(alignment_summary, handle, indent=2)
    print(f"Saved alignment summary: {output_alignment_json}")

    # --- Stage 2 outputs ---
    extrinsic_yaml, diagnostics = compute_extrinsic(args, marker_id, result1, result2)
    output_extrinsic_yaml.parent.mkdir(parents=True, exist_ok=True)
    def yaml_scalar(value: object) -> str:
        return "null" if value is None else str(value)

    with output_extrinsic_yaml.open("w") as handle:
        handle.write(extrinsic_yaml)
        handle.write("\ncalibration_diagnostics:\n")
        for key, value in diagnostics.items():
            if isinstance(value, dict):
                handle.write(f"  {key}:\n")
                for sub_key, sub_value in value.items():
                    handle.write(f"    {sub_key}: {yaml_scalar(sub_value)}\n")
            else:
                handle.write(f"  {key}: {yaml_scalar(value)}\n")
    print(f"Saved: {output_extrinsic_yaml}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
