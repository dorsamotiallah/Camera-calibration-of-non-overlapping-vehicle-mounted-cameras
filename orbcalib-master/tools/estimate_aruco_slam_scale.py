#!/usr/bin/env python3
"""Estimate monocular ORB-SLAM map scale from ArUco-marker map points."""

from __future__ import annotations

import argparse
import csv
import itertools
import json
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

from estimate_aruco_pattern_distance import detect_markers, dictionary_name
from find_aruco_map_points import (
    draw_debug,
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
    row_int,
    sorted_pngs,
)


RAW_REQUIRED_COLUMNS = {
    "kf_id",
    "frame_id",
    "timestamp",
    "kp_idx",
    "u",
    "v",
    "mp_id",
    "mp_x",
    "mp_y",
    "mp_z",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path, help="results_agilex/<run_id> directory")
    parser.add_argument("--camera", required=True, help="Camera name from manifest, or camera1/camera2/src/dst")
    parser.add_argument("--observations-csv", type=Path, help="Raw atlas observations CSV. Defaults from run manifest/name.")
    parser.add_argument("--camera1-dir", type=Path, help="Override camera 1 PNG directory")
    parser.add_argument("--camera2-dir", type=Path, help="Override camera 2 PNG directory")
    parser.add_argument("--marker-length-m", type=float, required=True, help="Physical ArUco marker side length in meters.")
    parser.add_argument("--dictionary-size", type=int, default=50, help="OpenCV 6x6 dictionary size. Default: 50")
    parser.add_argument("--marker-id", type=int, action="append", help="Marker id to keep. Can be repeated.")
    parser.add_argument("--margin-px", type=float, default=0.0, help="Include points this far outside marker polygon.")
    parser.add_argument("--min-points", type=int, default=4, help="Minimum selected points for a marker observation.")
    parser.add_argument(
        "--min-real-distance-m",
        type=float,
        default=None,
        help="Minimum metric pair distance. Default: 15%% of marker side length.",
    )
    parser.add_argument("--min-slam-distance", type=float, default=1e-7, help="Minimum SLAM pair distance.")
    parser.add_argument(
        "--max-pairs-per-marker",
        type=int,
        default=5000,
        help="Maximum point pairs sampled per keyframe/marker. 0 means no cap.",
    )
    parser.add_argument("--outlier-mad", type=float, default=3.5, help="Median absolute deviation outlier cutoff.")
    parser.add_argument("--max-keyframes", type=int, help="Stop after checking this many keyframes.")
    parser.add_argument("--frame-id-offset", type=int, default=0, help="Add this to mnFrameId before indexing sorted PNGs")
    parser.add_argument(
        "--no-frame-id-wrap",
        action="store_true",
        help="Fail when a frame id is outside the PNG range instead of using frame_id %% num_frames.",
    )
    parser.add_argument("--out-dir", type=Path, help="Output directory. Defaults to <run-dir>/aruco_scale/<camera>.")
    parser.add_argument(
        "--debug-images",
        dest="debug_images",
        action="store_true",
        default=True,
        help="Write marker/point overlay images. Default: enabled.",
    )
    parser.add_argument(
        "--no-debug-images",
        dest="debug_images",
        action="store_false",
        help="Skip marker/point overlay images.",
    )
    parser.add_argument("--no-pairs-csv", action="store_true", help="Do not write the detailed pair CSV.")
    return parser.parse_args()


def default_observations_csv(run_dir: Path, camera: str, side: str, manifest: Dict[str, str], repo_root: Path) -> Path:
    manifest_key = "camera1_raw_observations_csv" if side == "src" else "camera2_raw_observations_csv"
    value = manifest.get(manifest_key)
    if value:
        path = Path(value)
        if not path.is_absolute():
            path = repo_root / path
        if path.exists():
            return path
    return run_dir / f"{camera}_raw_keyframe_observations.csv"


def read_raw_observations(path: Path) -> Dict[str, List[Dict[str, str]]]:
    with path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
        missing = RAW_REQUIRED_COLUMNS - fields
        if missing:
            raise ValueError(f"{path} is missing raw observation columns: {', '.join(sorted(missing))}")

        grouped: Dict[str, List[Dict[str, str]]] = defaultdict(list)
        for row in reader:
            grouped[row["kf_id"]].append(row)
    return grouped


def marker_metric_coordinates(
    corners: np.ndarray,
    point: Tuple[float, float],
    marker_length_m: float,
) -> Tuple[float, float]:
    half = marker_length_m * 0.5
    src = corners.reshape(4, 2).astype(np.float32)
    dst = np.array(
        [
            [-half, half],
            [half, half],
            [half, -half],
            [-half, -half],
        ],
        dtype=np.float32,
    )
    homography = cv2.getPerspectiveTransform(src, dst)
    projected = cv2.perspectiveTransform(np.array([[point]], dtype=np.float32), homography)
    return float(projected[0, 0, 0]), float(projected[0, 0, 1])


def unique_by_map_point(points: Iterable[Dict[str, object]]) -> List[Dict[str, object]]:
    best_by_mp: Dict[str, Dict[str, object]] = {}
    for point in points:
        mp_id = str(point["mp_id"])
        previous = best_by_mp.get(mp_id)
        if previous is None or float(point["signed_distance_to_marker_px"]) > float(previous["signed_distance_to_marker_px"]):
            best_by_mp[mp_id] = point
    return list(best_by_mp.values())


def sampled_pairs(points: List[Dict[str, object]], max_pairs: int) -> Iterable[Tuple[Dict[str, object], Dict[str, object]]]:
    total = len(points) * (len(points) - 1) // 2
    if max_pairs <= 0 or total <= max_pairs:
        yield from itertools.combinations(points, 2)
        return

    rng = np.random.default_rng(0)
    seen = set()
    while len(seen) < max_pairs:
        i, j = sorted(rng.choice(len(points), size=2, replace=False).tolist())
        key = (i, j)
        if key in seen:
            continue
        seen.add(key)
        yield points[i], points[j]


def robust_summary(values: np.ndarray, outlier_mad: float) -> Dict[str, object]:
    if values.size == 0:
        return {
            "count": 0,
            "median": None,
            "mean": None,
            "std": None,
            "mad": None,
            "kept_count": 0,
            "rejected_count": 0,
            "kept_median": None,
            "kept_mean": None,
            "kept_std": None,
        }

    median = float(np.median(values))
    abs_dev = np.abs(values - median)
    mad = float(np.median(abs_dev))
    if mad > 0.0 and outlier_mad > 0.0:
        keep = abs_dev <= outlier_mad * 1.4826 * mad
    else:
        keep = np.ones(values.shape, dtype=bool)
    kept = values[keep]
    return {
        "count": int(values.size),
        "median": median,
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "mad": mad,
        "kept_count": int(kept.size),
        "rejected_count": int(values.size - kept.size),
        "kept_median": float(np.median(kept)) if kept.size else None,
        "kept_mean": float(np.mean(kept)) if kept.size else None,
        "kept_std": float(np.std(kept)) if kept.size else None,
    }


def draw_scale_debug(image: np.ndarray, marker_infos: List[Dict[str, object]], output_path: Path) -> None:
    draw_debug(image, marker_infos, output_path)


def main() -> int:
    args = parse_args()
    if args.marker_length_m <= 0:
        raise ValueError("--marker-length-m must be positive")

    repo_root = Path(__file__).resolve().parents[1]
    run_dir = args.run_dir.resolve()
    manifest = read_manifest(run_dir / "manifest.txt")
    side = side_for_camera(args.camera, manifest)
    observations_csv = args.observations_csv or default_observations_csv(run_dir, args.camera, side, manifest, repo_root)
    observations_csv = observations_csv.resolve()

    out_dir = args.out_dir or run_dir / "aruco_scale" / args.camera
    out_dir.mkdir(parents=True, exist_ok=True)
    debug_dir = out_dir / "debug_images"

    camera1_dir = args.camera1_dir or resolve_container_path(manifest.get("camera1_dir", ""), repo_root)
    camera2_dir = args.camera2_dir or resolve_container_path(manifest.get("camera2_dir", ""), repo_root)
    frames = sorted_pngs(camera1_dir if side == "src" else camera2_dir)
    frame_map1, frame_map2 = read_frame_map(run_dir / "frame_pairs.csv", repo_root)
    frame_map = frame_map1 if side == "src" else frame_map2

    grouped = read_raw_observations(observations_csv)
    marker_filter: Optional[set[int]] = set(args.marker_id) if args.marker_id else None
    min_real_distance_m = args.min_real_distance_m
    if min_real_distance_m is None:
        min_real_distance_m = args.marker_length_m * 0.15

    point_rows: List[Dict[str, object]] = []
    pair_rows: List[Dict[str, object]] = []
    checked_keyframes = 0
    marker_observations = 0
    marker_observations_with_pairs = 0

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

        marker_infos: List[Dict[str, object]] = []
        detected_ids = [int(marker_id) for marker_id in ids.reshape(-1)]
        for marker_corners, marker_id in zip(corners_list, detected_ids):
            if marker_filter is not None and marker_id not in marker_filter:
                continue

            polygon = marker_corners.reshape(4, 2).astype(np.float32)
            selected: List[Dict[str, object]] = []
            for row in rows:
                u = row_float(row, "u")
                v = row_float(row, "v")
                signed_distance = cv2.pointPolygonTest(polygon, (float(u), float(v)), True)
                if signed_distance < -args.margin_px:
                    continue

                marker_x_norm, marker_y_norm = marker_normalized_coordinates(marker_corners, (u, v))
                marker_x_m, marker_y_m = marker_metric_coordinates(marker_corners, (u, v), args.marker_length_m)
                point = {
                    "camera": args.camera,
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
                    "marker_x_m": marker_x_m,
                    "marker_y_m": marker_y_m,
                    "signed_distance_to_marker_px": float(signed_distance),
                    "mp_x": row_float(row, "mp_x"),
                    "mp_y": row_float(row, "mp_y"),
                    "mp_z": row_float(row, "mp_z"),
                    "camera_x": row.get("camera_x", ""),
                    "camera_y": row.get("camera_y", ""),
                    "camera_z": row.get("camera_z", ""),
                }
                selected.append(point)

            selected = unique_by_map_point(selected)
            if len(selected) < args.min_points:
                continue

            marker_observations += 1
            marker_pair_count = 0
            for p1, p2 in sampled_pairs(selected, args.max_pairs_per_marker):
                real_distance_m = math.hypot(
                    float(p2["marker_x_m"]) - float(p1["marker_x_m"]),
                    float(p2["marker_y_m"]) - float(p1["marker_y_m"]),
                )
                if real_distance_m < min_real_distance_m:
                    continue

                slam_distance = math.sqrt(
                    (float(p2["mp_x"]) - float(p1["mp_x"])) ** 2
                    + (float(p2["mp_y"]) - float(p1["mp_y"])) ** 2
                    + (float(p2["mp_z"]) - float(p1["mp_z"])) ** 2
                )
                if slam_distance < args.min_slam_distance:
                    continue

                scale = real_distance_m / slam_distance
                pair_rows.append(
                    {
                        "camera": args.camera,
                        "marker_id": marker_id,
                        "kf_id": kf_id,
                        "frame_id": p1["frame_id"],
                        "timestamp": p1["timestamp"],
                        "mp_id_1": p1["mp_id"],
                        "mp_id_2": p2["mp_id"],
                        "real_distance_m": real_distance_m,
                        "slam_distance": slam_distance,
                        "scale_m_per_slam_unit": scale,
                        "u1": p1["u"],
                        "v1": p1["v"],
                        "u2": p2["u"],
                        "v2": p2["v"],
                    }
                )
                marker_pair_count += 1

            if marker_pair_count > 0:
                marker_observations_with_pairs += 1
                point_rows.extend(selected)
                marker_infos.append({"marker_id": marker_id, "corners": marker_corners, "points": selected})

        if args.debug_images and marker_infos:
            out_name = f"{args.camera}_kf{kf_id}_{Path(image_path).stem}_scale_points.png"
            draw_scale_debug(image, marker_infos, debug_dir / out_name)

    points_path = out_dir / f"{args.camera}_aruco_scale_points.csv"
    pairs_path = out_dir / f"{args.camera}_aruco_scale_pairs.csv"
    summary_path = out_dir / f"{args.camera}_aruco_scale_summary.json"

    point_fields = [
        "camera",
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
    ]
    with points_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=point_fields)
        writer.writeheader()
        writer.writerows(point_rows)

    if not args.no_pairs_csv:
        pair_fields = [
            "camera",
            "marker_id",
            "kf_id",
            "frame_id",
            "timestamp",
            "mp_id_1",
            "mp_id_2",
            "real_distance_m",
            "slam_distance",
            "scale_m_per_slam_unit",
            "u1",
            "v1",
            "u2",
            "v2",
        ]
        with pairs_path.open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=pair_fields)
            writer.writeheader()
            writer.writerows(pair_rows)

    scales = np.array([float(row["scale_m_per_slam_unit"]) for row in pair_rows], dtype=float)
    stats = robust_summary(scales, args.outlier_mad)
    summary = {
        "run_dir": str(run_dir),
        "camera": args.camera,
        "side": side,
        "observations_csv": str(observations_csv),
        "aruco_dictionary": dictionary_name(args.dictionary_size),
        "marker_length_m": args.marker_length_m,
        "marker_ids": sorted(marker_filter) if marker_filter is not None else "all",
        "margin_px": args.margin_px,
        "min_points_per_marker": args.min_points,
        "min_real_distance_m": min_real_distance_m,
        "min_slam_distance": args.min_slam_distance,
        "max_pairs_per_marker": args.max_pairs_per_marker,
        "checked_keyframes": checked_keyframes,
        "marker_observations": marker_observations,
        "marker_observations_with_pairs": marker_observations_with_pairs,
        "point_observations_used": len(point_rows),
        "pair_observations_used": len(pair_rows),
        "scale_statistics": stats,
        "recommended_scale_m_per_slam_unit": stats["kept_median"],
        "outputs": {
            "summary_json": str(summary_path),
            "points_csv": str(points_path),
            "pairs_csv": None if args.no_pairs_csv else str(pairs_path),
            "debug_images_dir": str(debug_dir) if args.debug_images else None,
        },
    }
    with summary_path.open("w", encoding="utf-8") as handle:
        json.dump(summary, handle, indent=2)
        handle.write("\n")

    print(f"Checked {checked_keyframes} keyframe(s) for camera '{args.camera}'.")
    print(f"Used {len(point_rows)} point observation(s) and {len(pair_rows)} pair observation(s).")
    print(f"Recommended scale: {summary['recommended_scale_m_per_slam_unit']}")
    print(f"Output folder: {out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
