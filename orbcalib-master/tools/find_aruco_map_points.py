#!/usr/bin/env python3
"""Find ORB-SLAM map points whose keypoints fall on detected ArUco markers."""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np

from estimate_aruco_pattern_distance import detect_markers, dictionary_name
from visualize_calib_keyframe_matches import (
    load_image,
    load_image_by_timestamp,
    read_frame_map,
    read_manifest,
    resolve_container_path,
    row_float,
    row_int,
    sorted_pngs,
)


CAMERA_SIDE = {
    "camera1": "src",
    "camera2": "dst",
    "src": "src",
    "dst": "dst",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path, help="results_agilex/<run_id> directory")
    parser.add_argument("--csv", type=Path, help="Match CSV. Defaults to <run-dir>/calib_keyframe_matches.csv")
    parser.add_argument("--camera", required=True, help="Camera name from manifest, or camera1/camera2/src/dst")
    parser.add_argument("--camera1-dir", type=Path, help="Override camera 1 PNG directory")
    parser.add_argument("--camera2-dir", type=Path, help="Override camera 2 PNG directory")
    parser.add_argument("--stage", default="projection_after_optimized_pose", help="CSV stage to inspect")
    parser.add_argument("--marker-id", type=int, action="append", help="Marker id to keep. Can be repeated.")
    parser.add_argument("--dictionary-size", type=int, default=50, help="OpenCV 6x6 dictionary size. Default: 50")
    parser.add_argument("--margin-px", type=float, default=8.0, help="Include points this far outside marker polygon.")
    parser.add_argument("--min-points", type=int, default=3, help="Only report marker/keyframes with at least this many points.")
    parser.add_argument("--max-keyframes", type=int, help="Stop after checking this many keyframes.")
    parser.add_argument("--frame-id-offset", type=int, default=0, help="Add this to mnFrameId before indexing sorted PNGs")
    parser.add_argument(
        "--no-frame-id-wrap",
        action="store_true",
        help="Fail when a frame id is outside the PNG range instead of using frame_id %% num_frames.",
    )
    parser.add_argument("--output", type=Path, help="Output CSV. Defaults to <run-dir>/aruco_map_points_<camera>.csv")
    parser.add_argument("--debug-dir", type=Path, help="Directory for debug images with markers/map points drawn.")
    return parser.parse_args()


def side_for_camera(camera: str, manifest: Dict[str, str]) -> str:
    normalized = camera.strip().lower()
    if normalized in CAMERA_SIDE:
        return CAMERA_SIDE[normalized]

    camera1_name = manifest.get("camera1_name", "").strip().lower()
    camera2_name = manifest.get("camera2_name", "").strip().lower()
    if normalized == camera1_name:
        return "src"
    if normalized == camera2_name:
        return "dst"

    raise ValueError(
        f"Camera '{camera}' is not camera1/camera2/src/dst and does not match manifest "
        f"camera names '{camera1_name}'/'{camera2_name}'."
    )


def csv_mode(csv_path: Path) -> str:
    with csv_path.open(newline="") as handle:
        reader = csv.DictReader(handle)
        fields = set(reader.fieldnames or [])
    if {"kf_id", "frame_id", "timestamp", "u", "v", "mp_id", "mp_x", "mp_y", "mp_z"}.issubset(fields):
        return "raw"
    return "calib"


def group_rows_by_keyframe(
    csv_path: Path,
    stage: str,
    side: str,
) -> Tuple[str, Dict[str, List[Dict[str, str]]]]:
    mode = csv_mode(csv_path)
    grouped: Dict[str, List[Dict[str, str]]] = defaultdict(list)
    kf_key = "src_kf_id" if side == "src" else "dst_kf_id"
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if mode == "raw":
                grouped[row["kf_id"]].append(row)
                continue
            if row.get("stage") != stage:
                continue
            grouped[row[kf_key]].append(row)
    return mode, grouped


def unique_observations(rows: Iterable[Dict[str, str]], side: str, mode: str) -> List[Dict[str, str]]:
    if mode == "raw":
        seen = set()
        unique: List[Dict[str, str]] = []
        for row in rows:
            key = (row["mp_id"], row["kp_idx"])
            if key in seen:
                continue
            seen.add(key)
            unique.append(row)
        return unique

    prefix = "src" if side == "src" else "dst"
    seen = set()
    unique: List[Dict[str, str]] = []
    for row in rows:
        key = (row[f"{prefix}_mp_id"], row[f"{prefix}_kp_idx"])
        if key in seen:
            continue
        seen.add(key)
        unique.append(row)
    return unique


def load_side_image(
    rows: List[Dict[str, str]],
    side: str,
    mode: str,
    frames: List[Path],
    frame_map: List[Tuple[float, Path]],
    frame_id_offset: int,
    wrap_frame_id: bool,
) -> Tuple[Path, np.ndarray]:
    if mode == "raw":
        first = rows[0]
        if frame_map and first.get("timestamp"):
            return load_image_by_timestamp(frame_map, row_float(first, "timestamp"))
        return load_image(frames, row_int(first, "frame_id"), frame_id_offset, wrap_frame_id)

    prefix = "src" if side == "src" else "dst"
    first = rows[0]
    timestamp = first.get(f"{prefix}_timestamp")
    if frame_map and timestamp:
        return load_image_by_timestamp(frame_map, row_float(first, f"{prefix}_timestamp"))
    return load_image(frames, row_int(first, f"{prefix}_frame_id"), frame_id_offset, wrap_frame_id)


def marker_normalized_coordinates(corners: np.ndarray, point: Tuple[float, float]) -> Tuple[float, float]:
    src = corners.reshape(4, 2).astype(np.float32)
    dst = np.array([[0.0, 0.0], [1.0, 0.0], [1.0, 1.0], [0.0, 1.0]], dtype=np.float32)
    homography = cv2.getPerspectiveTransform(src, dst)
    projected = cv2.perspectiveTransform(np.array([[point]], dtype=np.float32), homography)
    return float(projected[0, 0, 0]), float(projected[0, 0, 1])


def row_point(row: Dict[str, str], side: str, mode: str) -> Tuple[float, float]:
    if mode == "raw":
        return row_float(row, "u"), row_float(row, "v")
    prefix = "src" if side == "src" else "dst"
    return row_float(row, f"{prefix}_u"), row_float(row, f"{prefix}_v")


def row_xyz(row: Dict[str, str], side: str, mode: str) -> Tuple[float, float, float]:
    if mode == "raw":
        return row_float(row, "mp_x"), row_float(row, "mp_y"), row_float(row, "mp_z")
    prefix = "src" if side == "src" else "dst"
    return (
        row_float(row, f"{prefix}_mp_x"),
        row_float(row, f"{prefix}_mp_y"),
        row_float(row, f"{prefix}_mp_z"),
    )


def draw_debug(
    image: np.ndarray,
    marker_infos: List[Dict[str, object]],
    output_path: Path,
) -> None:
    debug = image.copy()
    for marker in marker_infos:
        corners = np.asarray(marker["corners"], dtype=np.float32).reshape(4, 2)
        marker_id = int(marker["marker_id"])
        pts = np.round(corners).astype(int)
        cv2.polylines(debug, [pts], True, (0, 255, 0), 3, cv2.LINE_AA)
        center = tuple(np.round(corners.mean(axis=0)).astype(int))
        cv2.putText(debug, f"id {marker_id}", center, cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2, cv2.LINE_AA)
        for point in marker["points"]:
            u, v = int(round(point["u"])), int(round(point["v"]))
            cv2.circle(debug, (u, v), 5, (0, 255, 255), -1, cv2.LINE_AA)
            cv2.putText(debug, str(point["mp_id"]), (u + 6, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), debug):
        raise RuntimeError(f"Failed to write debug image: {output_path}")


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    run_dir = args.run_dir.resolve()
    csv_path = args.csv or run_dir / "calib_keyframe_matches.csv"
    manifest = read_manifest(run_dir / "manifest.txt")
    side = side_for_camera(args.camera, manifest)
    prefix = "src" if side == "src" else "dst"

    camera1_dir = args.camera1_dir or resolve_container_path(manifest.get("camera1_dir", ""), repo_root)
    camera2_dir = args.camera2_dir or resolve_container_path(manifest.get("camera2_dir", ""), repo_root)
    frames = sorted_pngs(camera1_dir if side == "src" else camera2_dir)
    frame_map1, frame_map2 = read_frame_map(run_dir / "frame_pairs.csv", repo_root)
    frame_map = frame_map1 if side == "src" else frame_map2

    output_path = args.output or run_dir / f"aruco_map_points_{args.camera}.csv"
    mode, grouped = group_rows_by_keyframe(csv_path, args.stage, side)
    if not grouped:
        if mode == "raw":
            raise ValueError(f"No rows found in raw observation CSV {csv_path}")
        raise ValueError(f"No rows found for stage '{args.stage}' in {csv_path}")

    marker_filter: Optional[set[int]] = set(args.marker_id) if args.marker_id else None
    output_rows: List[Dict[str, object]] = []
    checked = 0
    hit_keyframes = 0

    for kf_id, rows in sorted(grouped.items(), key=lambda item: int(float(item[0]))):
        if args.max_keyframes is not None and checked >= args.max_keyframes:
            break
        checked += 1
        rows = unique_observations(rows, side, mode)
        image_path, image = load_side_image(
            rows,
            side,
            mode,
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
            center = polygon.mean(axis=0)
            selected = []
            for row in rows:
                u, v = row_point(row, side, mode)
                signed_distance = cv2.pointPolygonTest(polygon, (float(u), float(v)), True)
                if signed_distance < -args.margin_px:
                    continue

                marker_x, marker_y = marker_normalized_coordinates(marker_corners, (u, v))
                mp_x, mp_y, mp_z = row_xyz(row, side, mode)
                center_distance_px = math.hypot(float(u - center[0]), float(v - center[1]))
                row_stage = "raw_atlas_observation" if mode == "raw" else args.stage
                frame_id = row["frame_id"] if mode == "raw" else row[f"{prefix}_frame_id"]
                timestamp = row.get("timestamp", "") if mode == "raw" else row.get(f"{prefix}_timestamp", "")
                kp_idx = row["kp_idx"] if mode == "raw" else row[f"{prefix}_kp_idx"]
                mp_id = row["mp_id"] if mode == "raw" else row[f"{prefix}_mp_id"]
                point = {
                    "camera": args.camera,
                    "side": side,
                    "stage": row_stage,
                    "marker_id": marker_id,
                    "kf_id": kf_id,
                    "frame_id": frame_id,
                    "timestamp": timestamp,
                    "image_path": str(image_path),
                    "kp_idx": kp_idx,
                    "mp_id": mp_id,
                    "u": u,
                    "v": v,
                    "marker_x_norm": marker_x,
                    "marker_y_norm": marker_y,
                    "signed_distance_to_marker_px": float(signed_distance),
                    "distance_to_marker_center_px": center_distance_px,
                    "mp_x": mp_x,
                    "mp_y": mp_y,
                    "mp_z": mp_z,
                }
                selected.append(point)

            if len(selected) < args.min_points:
                continue
            output_rows.extend(selected)
            marker_infos.append({"marker_id": marker_id, "corners": marker_corners, "points": selected})

        if marker_infos:
            hit_keyframes += 1
            if args.debug_dir:
                out_name = f"{args.camera}_kf{kf_id}_{Path(image_path).stem}_aruco_points.png"
                draw_debug(image, marker_infos, args.debug_dir / out_name)

    fieldnames = [
        "camera",
        "side",
        "stage",
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
        "signed_distance_to_marker_px",
        "distance_to_marker_center_px",
        "mp_x",
        "mp_y",
        "mp_z",
    ]
    output_path.parent.mkdir(parents=True, exist_ok=True)
    with output_path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(output_rows)

    print(f"Checked {checked} keyframe(s) for camera '{args.camera}' ({side}).")
    print(f"Found ArUco-associated map points in {hit_keyframes} keyframe(s).")
    print(f"Wrote {len(output_rows)} point observation(s): {output_path}")
    if args.debug_dir:
        print(f"Debug images: {args.debug_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
