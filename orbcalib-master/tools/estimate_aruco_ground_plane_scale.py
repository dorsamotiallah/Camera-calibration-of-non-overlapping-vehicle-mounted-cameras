#!/usr/bin/env python3
"""Estimate map scale from ArUco-on-ground points and known camera height."""

from __future__ import annotations

import argparse
import csv
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

from estimate_aruco_pattern_distance import detect_markers, dictionary_name
from estimate_aruco_slam_scale import (
    default_observations_csv,
    marker_metric_coordinates,
    read_raw_observations,
    robust_summary,
)
from find_aruco_map_points import (
    load_side_image,
    marker_normalized_coordinates,
    side_for_camera,
    unique_observations,
)
from visualize_calib_keyframe_matches import (
    read_frame_map,
    read_manifest,
    resolve_container_path,
    row_float,
    sorted_pngs,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path, help="results_agilex/<run_id> directory")
    parser.add_argument(
        "--camera",
        action="append",
        help="Camera/map to process. Can be repeated. Default: camera1 and camera2 from manifest.",
    )
    parser.add_argument("--camera1-dir", type=Path, help="Override camera 1 PNG directory")
    parser.add_argument("--camera2-dir", type=Path, help="Override camera 2 PNG directory")
    parser.add_argument("--dictionary-size", type=int, default=50, help="OpenCV 6x6 dictionary size. Default: 50")
    parser.add_argument("--marker-id", type=int, action="append", help="Marker id to keep. Can be repeated.")
    parser.add_argument("--margin-px", type=float, default=0.0, help="Include points this far outside marker polygon.")
    parser.add_argument("--min-points", type=int, default=12, help="Minimum unique map points required per camera.")
    parser.add_argument("--max-keyframes", type=int, help="Stop after checking this many keyframes per camera.")
    parser.add_argument("--frame-id-offset", type=int, default=0, help="Add this to mnFrameId before indexing sorted PNGs")
    parser.add_argument(
        "--no-frame-id-wrap",
        action="store_true",
        help="Fail when a frame id is outside the PNG range instead of using frame_id %% num_frames.",
    )
    parser.add_argument(
        "--marker-length-m",
        type=float,
        help="Optional physical marker side length; adds marker-plane metric coordinates to the point CSV.",
    )
    parser.add_argument(
        "--camera-height-m",
        type=float,
        help="Known camera optical-center height above ground in meters. Used for every camera unless overridden.",
    )
    parser.add_argument(
        "--camera-height",
        action="append",
        default=[],
        metavar="CAMERA=METERS",
        help="Per-camera optical-center height, e.g. front=0.72. Can be repeated.",
    )
    parser.add_argument("--ransac-threshold", type=float, default=0.03, help="Plane inlier threshold in SLAM units.")
    parser.add_argument("--ransac-iterations", type=int, default=3000, help="Number of RANSAC plane samples.")
    parser.add_argument("--outlier-mad", type=float, default=3.5, help="MAD cutoff for camera-distance scale summary.")
    parser.add_argument("--seed", type=int, default=0, help="RANSAC random seed.")
    parser.add_argument("--out-dir", type=Path, help="Output directory. Defaults to <run-dir>/aruco_ground_scale.")
    return parser.parse_args()


def parse_camera_heights(args: argparse.Namespace) -> Dict[str, float]:
    heights: Dict[str, float] = {}
    for item in args.camera_height:
        if "=" not in item:
            raise ValueError(f"--camera-height must look like CAMERA=METERS, got '{item}'")
        camera, value = item.split("=", 1)
        height = float(value)
        if height <= 0.0:
            raise ValueError("--camera-height values must be positive")
        heights[camera.strip().lower()] = height
    if args.camera_height_m is not None and args.camera_height_m <= 0.0:
        raise ValueError("--camera-height-m must be positive")
    return heights


def camera_height_for(camera: str, default_height: Optional[float], heights: Dict[str, float]) -> Optional[float]:
    return heights.get(camera.lower(), default_height)


def unique_by_map_point(points: Iterable[Dict[str, object]]) -> List[Dict[str, object]]:
    best_by_mp: Dict[str, Dict[str, object]] = {}
    for point in points:
        mp_id = str(point["mp_id"])
        previous = best_by_mp.get(mp_id)
        if previous is None or float(point["signed_distance_to_marker_px"]) > float(previous["signed_distance_to_marker_px"]):
            best_by_mp[mp_id] = point
    return list(best_by_mp.values())


def unique_camera_centers(points: Iterable[Dict[str, object]]) -> List[Dict[str, object]]:
    by_kf: Dict[str, List[np.ndarray]] = defaultdict(list)
    metadata: Dict[str, Dict[str, object]] = {}
    for point in points:
        center = np.array(
            [float(point["camera_x"]), float(point["camera_y"]), float(point["camera_z"])],
            dtype=float,
        )
        if not np.isfinite(center).all():
            continue
        kf_id = str(point["kf_id"])
        by_kf[kf_id].append(center)
        metadata.setdefault(
            kf_id,
            {
                "camera": point["camera"],
                "kf_id": point["kf_id"],
                "frame_id": point["frame_id"],
                "timestamp": point["timestamp"],
            },
        )

    centers: List[Dict[str, object]] = []
    for kf_id, values in by_kf.items():
        center = np.mean(np.vstack(values), axis=0)
        row = dict(metadata[kf_id])
        row.update({"camera_x": center[0], "camera_y": center[1], "camera_z": center[2]})
        centers.append(row)
    return sorted(centers, key=lambda row: int(float(str(row["kf_id"]))))


def plane_from_points(points: np.ndarray) -> Optional[Tuple[np.ndarray, float]]:
    v1 = points[1] - points[0]
    v2 = points[2] - points[0]
    normal = np.cross(v1, v2)
    norm = float(np.linalg.norm(normal))
    if norm < 1e-12:
        return None
    normal /= norm
    offset = -float(normal.dot(points[0]))
    return normal, offset


def canonical_plane(normal: np.ndarray, offset: float, camera_centers: np.ndarray) -> Tuple[np.ndarray, float]:
    if camera_centers.size == 0:
        return normal, offset
    signed = camera_centers.dot(normal) + offset
    if float(np.median(signed)) < 0.0:
        return -normal, -offset
    return normal, offset


def fit_ransac_plane(
    points: np.ndarray,
    camera_centers: np.ndarray,
    threshold: float,
    iterations: int,
    seed: int,
) -> Dict[str, object]:
    if points.shape[0] < 3:
        raise ValueError("At least 3 points are required to fit a plane")
    if threshold <= 0.0:
        raise ValueError("--ransac-threshold must be positive")

    rng = np.random.default_rng(seed)
    best_inliers: Optional[np.ndarray] = None
    best_normal: Optional[np.ndarray] = None
    best_offset: Optional[float] = None
    best_error = float("inf")

    for _ in range(iterations):
        sample_idx = rng.choice(points.shape[0], size=3, replace=False)
        candidate = plane_from_points(points[sample_idx])
        if candidate is None:
            continue
        normal, offset = candidate
        distances = np.abs(points.dot(normal) + offset)
        inliers = distances <= threshold
        count = int(np.count_nonzero(inliers))
        if count < 3:
            continue
        error = float(np.median(distances[inliers]))
        if best_inliers is None or count > int(np.count_nonzero(best_inliers)) or (
            count == int(np.count_nonzero(best_inliers)) and error < best_error
        ):
            best_inliers = inliers
            best_normal = normal
            best_offset = offset
            best_error = error

    if best_inliers is None or best_normal is None or best_offset is None:
        raise RuntimeError("RANSAC failed to find a valid plane")

    inlier_points = points[best_inliers]
    centroid = np.mean(inlier_points, axis=0)
    _, _, vh = np.linalg.svd(inlier_points - centroid, full_matrices=False)
    normal = vh[-1]
    normal /= np.linalg.norm(normal)
    offset = -float(normal.dot(centroid))
    normal, offset = canonical_plane(normal, offset, camera_centers)

    point_distances = np.abs(points.dot(normal) + offset)
    inliers = point_distances <= threshold
    return {
        "normal": normal,
        "offset": offset,
        "point_distances": point_distances,
        "inliers": inliers,
    }


def collect_camera_points(
    run_dir: Path,
    repo_root: Path,
    manifest: Dict[str, str],
    camera: str,
    args: argparse.Namespace,
) -> Tuple[List[Dict[str, object]], int, int]:
    side = side_for_camera(camera, manifest)
    observations_csv = default_observations_csv(run_dir, camera, side, manifest, repo_root).resolve()
    grouped = read_raw_observations(observations_csv)

    camera1_dir = args.camera1_dir or resolve_container_path(manifest.get("camera1_dir", ""), repo_root)
    camera2_dir = args.camera2_dir or resolve_container_path(manifest.get("camera2_dir", ""), repo_root)
    frames = sorted_pngs(camera1_dir if side == "src" else camera2_dir)
    frame_map1, frame_map2 = read_frame_map(run_dir / "frame_pairs.csv", repo_root)
    frame_map = frame_map1 if side == "src" else frame_map2
    marker_filter: Optional[set[int]] = set(args.marker_id) if args.marker_id else None

    point_rows: List[Dict[str, object]] = []
    checked_keyframes = 0
    hit_keyframes = 0
    for kf_id, raw_rows in sorted(grouped.items(), key=lambda item: int(float(item[0]))):
        if args.max_keyframes is not None and checked_keyframes >= args.max_keyframes:
            break
        checked_keyframes += 1

        rows = unique_observations(raw_rows, side, "raw")
        image_path, image = load_side_image(
            rows,
            side,
            "raw",
            frames,
            frame_map,
            args.frame_id_offset,
            not args.no_frame_id_wrap,
        )
        corners_list, ids, _ = detect_markers(image, args.dictionary_size)
        if ids is None or len(ids) == 0:
            continue

        keyframe_hit = False
        detected_ids = [int(marker_id) for marker_id in ids.reshape(-1)]
        for marker_corners, marker_id in zip(corners_list, detected_ids):
            if marker_filter is not None and marker_id not in marker_filter:
                continue

            polygon = marker_corners.reshape(4, 2).astype(np.float32)
            for row in rows:
                u = row_float(row, "u")
                v = row_float(row, "v")
                signed_distance = cv2.pointPolygonTest(polygon, (float(u), float(v)), True)
                if signed_distance < -args.margin_px:
                    continue

                marker_x_norm, marker_y_norm = marker_normalized_coordinates(marker_corners, (u, v))
                point = {
                    "camera": camera,
                    "side": side,
                    "marker_id": marker_id,
                    "kf_id": kf_id,
                    "frame_id": row["frame_id"],
                    "timestamp": row.get("timestamp", ""),
                    "image_path": str(image_path),
                    "kp_idx": row["kp_idx"],
                    "mp_id": row["mp_id"],
                    "u": u,
                    "v": v,
                    "marker_x_norm": marker_x_norm,
                    "marker_y_norm": marker_y_norm,
                    "marker_x_m": "",
                    "marker_y_m": "",
                    "signed_distance_to_marker_px": float(signed_distance),
                    "mp_x": row_float(row, "mp_x"),
                    "mp_y": row_float(row, "mp_y"),
                    "mp_z": row_float(row, "mp_z"),
                    "camera_x": row_float(row, "camera_x"),
                    "camera_y": row_float(row, "camera_y"),
                    "camera_z": row_float(row, "camera_z"),
                }
                if args.marker_length_m is not None:
                    marker_x_m, marker_y_m = marker_metric_coordinates(marker_corners, (u, v), args.marker_length_m)
                    point["marker_x_m"] = marker_x_m
                    point["marker_y_m"] = marker_y_m
                point_rows.append(point)
                keyframe_hit = True
        if keyframe_hit:
            hit_keyframes += 1

    return point_rows, checked_keyframes, hit_keyframes


def write_csv(path: Path, rows: List[Dict[str, object]], fieldnames: List[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def summarize_camera(
    camera: str,
    points: List[Dict[str, object]],
    checked_keyframes: int,
    hit_keyframes: int,
    camera_height_m: Optional[float],
    args: argparse.Namespace,
    out_dir: Path,
) -> Dict[str, object]:
    unique_points = unique_by_map_point(points)
    if len(unique_points) < args.min_points:
        raise RuntimeError(f"{camera}: only {len(unique_points)} unique ArUco map points, need {args.min_points}")

    point_xyz = np.array(
        [[float(row["mp_x"]), float(row["mp_y"]), float(row["mp_z"])] for row in unique_points],
        dtype=float,
    )
    camera_centers = unique_camera_centers(points)
    center_xyz = np.array(
        [[float(row["camera_x"]), float(row["camera_y"]), float(row["camera_z"])] for row in camera_centers],
        dtype=float,
    )
    fit = fit_ransac_plane(point_xyz, center_xyz, args.ransac_threshold, args.ransac_iterations, args.seed)
    normal = np.asarray(fit["normal"], dtype=float)
    offset = float(fit["offset"])
    inliers = np.asarray(fit["inliers"], dtype=bool)
    point_distances = np.asarray(fit["point_distances"], dtype=float)

    for row, distance, is_inlier in zip(unique_points, point_distances, inliers):
        row["plane_distance_slam"] = float(distance)
        row["plane_inlier"] = int(bool(is_inlier))

    camera_distance_rows: List[Dict[str, object]] = []
    scale_values: List[float] = []
    signed_distances = center_xyz.dot(normal) + offset if center_xyz.size else np.array([], dtype=float)
    for row, signed_distance in zip(camera_centers, signed_distances):
        distance = abs(float(signed_distance))
        scale = ""
        if camera_height_m is not None and distance > 1e-12:
            scale = camera_height_m / distance
            scale_values.append(float(scale))
        camera_distance_rows.append(
            {
                **row,
                "signed_plane_distance_slam": float(signed_distance),
                "plane_distance_slam": distance,
                "camera_height_m": "" if camera_height_m is None else camera_height_m,
                "scale_m_per_slam_unit": scale,
            }
        )

    point_fields = [
        "camera",
        "side",
        "marker_id",
        "kf_id",
        "frame_id",
        "timestamp",
        "image_path",
        "kp_idx",
        "mp_id",
        "u",
        "v",
        "marker_x_norm",
        "marker_y_norm",
        "marker_x_m",
        "marker_y_m",
        "signed_distance_to_marker_px",
        "mp_x",
        "mp_y",
        "mp_z",
        "camera_x",
        "camera_y",
        "camera_z",
        "plane_distance_slam",
        "plane_inlier",
    ]
    center_fields = [
        "camera",
        "kf_id",
        "frame_id",
        "timestamp",
        "camera_x",
        "camera_y",
        "camera_z",
        "signed_plane_distance_slam",
        "plane_distance_slam",
        "camera_height_m",
        "scale_m_per_slam_unit",
    ]
    write_csv(out_dir / f"{camera}_aruco_ground_points.csv", unique_points, point_fields)
    write_csv(out_dir / f"{camera}_camera_plane_distances.csv", camera_distance_rows, center_fields)

    point_inlier_distances = point_distances[inliers]
    scale_stats = robust_summary(np.asarray(scale_values, dtype=float), args.outlier_mad)
    return {
        "camera": camera,
        "checked_keyframes": checked_keyframes,
        "marker_keyframes": hit_keyframes,
        "marker_point_observations": len(points),
        "unique_marker_points": len(unique_points),
        "plane_inlier_points": int(np.count_nonzero(inliers)),
        "plane_inlier_fraction": float(np.count_nonzero(inliers) / len(inliers)),
        "plane_normal": normal.tolist(),
        "plane_offset": offset,
        "ransac_threshold_slam": args.ransac_threshold,
        "plane_point_distance_slam": {
            "median_all": float(np.median(point_distances)),
            "median_inliers": float(np.median(point_inlier_distances)) if point_inlier_distances.size else None,
            "mean_inliers": float(np.mean(point_inlier_distances)) if point_inlier_distances.size else None,
        },
        "camera_centers_used": len(camera_distance_rows),
        "camera_height_m": camera_height_m,
        "camera_plane_distance_slam": {
            "median": float(np.median(np.abs(signed_distances))) if signed_distances.size else None,
            "mean": float(np.mean(np.abs(signed_distances))) if signed_distances.size else None,
            "std": float(np.std(np.abs(signed_distances))) if signed_distances.size else None,
        },
        "scale_statistics": scale_stats,
        "recommended_scale_m_per_slam_unit": scale_stats["kept_median"],
        "outputs": {
            "points_csv": str(out_dir / f"{camera}_aruco_ground_points.csv"),
            "camera_distances_csv": str(out_dir / f"{camera}_camera_plane_distances.csv"),
        },
    }


def main() -> int:
    args = parse_args()
    if args.marker_length_m is not None and args.marker_length_m <= 0.0:
        raise ValueError("--marker-length-m must be positive")
    if args.ransac_iterations <= 0:
        raise ValueError("--ransac-iterations must be positive")

    repo_root = Path(__file__).resolve().parents[1]
    run_dir = args.run_dir.resolve()
    manifest = read_manifest(run_dir / "manifest.txt")
    cameras = args.camera or [manifest.get("camera1_name", "camera1"), manifest.get("camera2_name", "camera2")]
    heights = parse_camera_heights(args)
    out_dir = args.out_dir or run_dir / "aruco_ground_scale"
    out_dir.mkdir(parents=True, exist_ok=True)

    summaries: List[Dict[str, object]] = []
    for camera in cameras:
        camera_points, checked_keyframes, hit_keyframes = collect_camera_points(run_dir, repo_root, manifest, camera, args)
        height = camera_height_for(camera, args.camera_height_m, heights)
        summary = summarize_camera(camera, camera_points, checked_keyframes, hit_keyframes, height, args, out_dir)
        summaries.append(summary)

    summary = {
        "run_dir": str(run_dir),
        "aruco_dictionary": dictionary_name(args.dictionary_size),
        "marker_ids": sorted(args.marker_id) if args.marker_id else "all",
        "margin_px": args.margin_px,
        "marker_length_m": args.marker_length_m,
        "cameras": summaries,
    }
    summary_path = out_dir / "aruco_ground_scale_summary.json"
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    print(f"Wrote ground-plane scale outputs: {out_dir}")
    for item in summaries:
        print(
            f"{item['camera']}: {item['unique_marker_points']} points, "
            f"{item['plane_inlier_points']} inliers, recommended scale "
            f"{item['recommended_scale_m_per_slam_unit']}"
        )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
