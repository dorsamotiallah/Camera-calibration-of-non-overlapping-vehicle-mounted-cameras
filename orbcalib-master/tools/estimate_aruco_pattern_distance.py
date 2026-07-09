#!/usr/bin/env python3
"""Detect one expected 6x6 ArUco marker and estimate distance to its plane."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Dict, List, Tuple

import cv2
import numpy as np

from estimate_checkerboard_camera_height import (
    CameraCalibration,
    read_calibration,
    scaled_calibration,
)


ARUCO_6X6_DICTIONARIES = {
    50: cv2.aruco.DICT_6X6_50,
    100: cv2.aruco.DICT_6X6_100,
    250: cv2.aruco.DICT_6X6_250,
    1000: cv2.aruco.DICT_6X6_1000,
}
EXPECTED_IDS = tuple(range(4))


def dictionary_name(dictionary_size: int) -> str:
    return f"6x6_{dictionary_size}"


def get_aruco_dictionary(dictionary_size: int) -> cv2.aruco.Dictionary:
    dict_id = ARUCO_6X6_DICTIONARIES[dictionary_size]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(dict_id)
    return cv2.aruco.Dictionary_get(dict_id)


def make_detector_parameters() -> cv2.aruco.DetectorParameters:
    if hasattr(cv2.aruco, "DetectorParameters"):
        return cv2.aruco.DetectorParameters()
    return cv2.aruco.DetectorParameters_create()


def detect_markers(
    image: np.ndarray,
    dictionary_size: int,
) -> Tuple[List[np.ndarray], np.ndarray | None, List[np.ndarray]]:
    dictionary = get_aruco_dictionary(dictionary_size)
    parameters = make_detector_parameters()
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image

    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if corners:
        term = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER, 50, 0.001)
        for marker_corners in corners:
            cv2.cornerSubPix(gray, marker_corners, (5, 5), (-1, -1), term)

    return corners, ids, rejected


def marker_object_points(marker_length: float) -> np.ndarray:
    half = marker_length * 0.5
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def solve_marker_pose(
    corners: np.ndarray,
    marker_length: float,
    calib: CameraCalibration,
) -> Tuple[np.ndarray, np.ndarray]:
    object_points = marker_object_points(marker_length)

    if calib.model == "KannalaBrandt8":
        image_points = cv2.fisheye.undistortPoints(corners.astype(np.float64), calib.K, calib.D)
        camera_matrix = np.eye(3, dtype=np.float64)
        distortion = None
    else:
        image_points = corners.astype(np.float64)
        camera_matrix = calib.K
        distortion = calib.D

    candidate_flags = []
    if hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
        candidate_flags.append(cv2.SOLVEPNP_IPPE_SQUARE)
    candidate_flags.append(cv2.SOLVEPNP_ITERATIVE)

    candidates: List[Tuple[float, np.ndarray, np.ndarray]] = []
    for flags in candidate_flags:
        ok, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, distortion, flags=flags)
        if not ok or not np.isfinite(rvec).all() or not np.isfinite(tvec).all():
            continue
        error = reprojection_error(corners, marker_length, rvec, tvec, calib)
        if np.isfinite(error):
            candidates.append((error, rvec, tvec))

    if not candidates:
        raise RuntimeError("cv2.solvePnP failed for an ArUco marker")
    _, best_rvec, best_tvec = min(candidates, key=lambda item: item[0])
    return best_rvec, best_tvec


def project_points(
    object_points: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    calib: CameraCalibration,
) -> np.ndarray:
    if calib.model == "KannalaBrandt8":
        projected, _ = cv2.fisheye.projectPoints(
            object_points.reshape(-1, 1, 3).astype(np.float64),
            rvec,
            tvec,
            calib.K,
            calib.D,
        )
    else:
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, calib.K, calib.D)
    return projected.reshape(-1, 2)


def reprojection_error(
    corners: np.ndarray,
    marker_length: float,
    rvec: np.ndarray,
    tvec: np.ndarray,
    calib: CameraCalibration,
) -> float:
    projected = project_points(marker_object_points(marker_length), rvec, tvec, calib)
    detected = corners.reshape(-1, 2)
    return float(np.mean(np.linalg.norm(projected - detected, axis=1)))


def plane_distance(rvec: np.ndarray, tvec: np.ndarray) -> Tuple[float, float]:
    rotation, _ = cv2.Rodrigues(rvec)
    normal_camera = rotation[:, 2].reshape(3)
    signed = float(normal_camera.dot(tvec.reshape(3)))
    return abs(signed), signed


def draw_pose_axes(
    image: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    calib: CameraCalibration,
    axis_length: float,
) -> None:
    axis_points = np.array(
        [
            [0.0, 0.0, 0.0],
            [axis_length, 0.0, 0.0],
            [0.0, axis_length, 0.0],
            [0.0, 0.0, axis_length],
        ],
        dtype=np.float64,
    )
    projected = np.round(project_points(axis_points, rvec, tvec, calib)).astype(int)
    origin = tuple(projected[0])
    cv2.line(image, origin, tuple(projected[1]), (0, 0, 255), 2, cv2.LINE_AA)
    cv2.line(image, origin, tuple(projected[2]), (0, 255, 0), 2, cv2.LINE_AA)
    cv2.line(image, origin, tuple(projected[3]), (255, 0, 0), 2, cv2.LINE_AA)


def draw_labeled_marker(
    image: np.ndarray,
    corners: np.ndarray,
    marker_id: int,
    expected_id: int,
) -> None:
    pts = np.round(corners.reshape(-1, 2)).astype(int)
    color = (0, 255, 0) if marker_id == expected_id else (0, 0, 255)
    for idx in range(4):
        cv2.line(image, tuple(pts[idx]), tuple(pts[(idx + 1) % 4]), color, 3, cv2.LINE_AA)
        cv2.circle(image, tuple(pts[idx]), 6, color, -1, cv2.LINE_AA)
        cv2.putText(
            image,
            str(idx),
            tuple(pts[idx] + np.array([8, -8])),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            color,
            2,
            cv2.LINE_AA,
        )

    center = tuple(np.mean(pts, axis=0).astype(int))
    cv2.putText(
        image,
        f"id {marker_id}",
        (center[0] + 12, center[1]),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        color,
        2,
        cv2.LINE_AA,
    )


def estimate_aruco_distances(
    image_path: Path,
    calib_path: Path,
    marker_length: float,
    expected_id: int,
    dictionary_size: int,
) -> Dict[str, object]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Could not read image: {image_path}")

    calib = scaled_calibration(read_calibration(calib_path), (image.shape[1], image.shape[0]))
    corners, ids, rejected = detect_markers(image, dictionary_size)
    if ids is None or len(ids) == 0:
        raise RuntimeError(f"No {dictionary_name(dictionary_size)} ArUco markers found in {image_path}")

    detected_ids = [int(marker_id) for marker_id in ids.reshape(-1)]
    allowed_detected_ids = [marker_id for marker_id in detected_ids if marker_id in EXPECTED_IDS]
    marker_indexes = [idx for idx, marker_id in enumerate(detected_ids) if marker_id == expected_id]
    if not marker_indexes:
        raise RuntimeError(
            f"Expected marker id {expected_id}, but detected ids were {detected_ids}. "
            f"Allowed pattern ids are {list(EXPECTED_IDS)}."
        )

    markers: List[Dict[str, object]] = []
    for marker_index in marker_indexes:
        marker_corners = corners[marker_index]
        marker_id = detected_ids[marker_index]
        rvec, tvec = solve_marker_pose(marker_corners, marker_length, calib)
        perpendicular_distance, signed_plane_distance = plane_distance(rvec, tvec)
        markers.append(
            {
                "id": marker_id,
                "center_distance": float(np.linalg.norm(tvec.reshape(3))),
                "perpendicular_plane_distance": perpendicular_distance,
                "signed_plane_distance": signed_plane_distance,
                "tvec_marker_to_camera": tvec.reshape(3).tolist(),
                "rvec_marker_to_camera": rvec.reshape(3).tolist(),
                "mean_reprojection_error_px": reprojection_error(marker_corners, marker_length, rvec, tvec, calib),
                "corners_px": marker_corners.reshape(-1, 2).tolist(),
            }
        )

    if len(markers) > 1:
        raise RuntimeError(f"Expected one marker id {expected_id}, but detected {len(markers)} copies")

    marker = markers[0]
    return {
        "image": str(image_path),
        "calibration": str(calib_path),
        "camera_model": calib.model,
        "image_width": int(image.shape[1]),
        "image_height": int(image.shape[0]),
        "aruco_dictionary": dictionary_name(dictionary_size),
        "marker_length": marker_length,
        "expected_id": expected_id,
        "detected_ids": detected_ids,
        "allowed_detected_ids": allowed_detected_ids,
        "rejected_candidates": len(rejected),
        "perpendicular_plane_distance": marker["perpendicular_plane_distance"],
        "center_distance": marker["center_distance"],
        "marker": marker,
    }


def make_debug_image(
    image_path: Path,
    calib_path: Path,
    marker_length: float,
    expected_id: int,
    dictionary_size: int,
    result: Dict[str, object],
) -> np.ndarray:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Could not read image for debug output: {image_path}")
    calib = scaled_calibration(read_calibration(calib_path), (image.shape[1], image.shape[0]))
    corners, ids, _ = detect_markers(image, dictionary_size)

    detected_ids = [] if ids is None else [int(marker_id) for marker_id in ids.reshape(-1)]
    for marker_corners, marker_id in zip(corners, detected_ids):
        draw_labeled_marker(image, marker_corners, marker_id, expected_id)
        if marker_id != expected_id:
            continue
        rvec, tvec = solve_marker_pose(marker_corners, marker_length, calib)
        draw_pose_axes(image, rvec, tvec, calib, marker_length * 0.5)

    label = f"\"perpendicular_plane_distance\": {float(result['perpendicular_plane_distance']):.17g}"
    cv2.rectangle(image, (0, 0), (min(image.shape[1], 1120), 70), (0, 0, 0), -1)
    cv2.putText(image, label, (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(
        image,
        "green = expected marker, red = other detected marker, corner labels = OpenCV order",
        (12, 56),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.55,
        (255, 255, 255),
        1,
        cv2.LINE_AA,
    )
    return image


def write_debug_image(
    output_path: Path,
    image_path: Path,
    calib_path: Path,
    marker_length: float,
    expected_id: int,
    dictionary_size: int,
    result: Dict[str, object],
) -> None:
    image = make_debug_image(image_path, calib_path, marker_length, expected_id, dictionary_size, result)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Failed to write debug image: {output_path}")


def show_debug_image(
    image_path: Path,
    calib_path: Path,
    marker_length: float,
    expected_id: int,
    dictionary_size: int,
    result: Dict[str, object],
) -> None:
    image = make_debug_image(image_path, calib_path, marker_length, expected_id, dictionary_size, result)
    cv2.imshow("ArUco distance debug", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Input photo, e.g. the first frame of a recording.")
    parser.add_argument(
        "--calib",
        type=Path,
        required=True,
        help="ORB-SLAM/OpenCV camera YAML, e.g. config/sim/agilex_front_cam.yaml.",
    )
    parser.add_argument(
        "--marker-length",
        type=float,
        required=True,
        help="Physical ArUco marker side length. Output distances use this same unit.",
    )
    parser.add_argument(
        "--expected-id",
        type=int,
        required=True,
        choices=EXPECTED_IDS,
        help="Expected marker id in this image. This rig uses 6x6 ids 0, 1, 2, and 3.",
    )
    parser.add_argument(
        "--dictionary-size",
        type=int,
        choices=sorted(ARUCO_6X6_DICTIONARIES),
        default=50,
        help="OpenCV 6x6 predefined dictionary size. Default is DICT_6X6_50.",
    )
    parser.add_argument(
        "--debug-image",
        type=Path,
        default=None,
        help="Optional path to save the detection visualization.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Open an OpenCV window with the detection visualization without saving it.",
    )
    parser.add_argument("--json", action="store_true", help="Print the full result as JSON.")
    args = parser.parse_args()
    if args.marker_length <= 0.0:
        parser.error("--marker-length must be positive")
    return args


def main() -> int:
    args = parse_args()
    image_path = args.image.expanduser().resolve()
    calib_path = args.calib.expanduser().resolve()

    result = estimate_aruco_distances(
        image_path=image_path,
        calib_path=calib_path,
        marker_length=args.marker_length,
        expected_id=args.expected_id,
        dictionary_size=args.dictionary_size,
    )

    if args.debug_image is not None:
        write_debug_image(
            args.debug_image.expanduser().resolve(),
            image_path,
            calib_path,
            args.marker_length,
            args.expected_id,
            args.dictionary_size,
            result,
        )
    if args.show:
        show_debug_image(
            image_path,
            calib_path,
            args.marker_length,
            args.expected_id,
            args.dictionary_size,
            result,
        )

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        marker = result["marker"]
        print(f"aruco_dictionary: {result['aruco_dictionary']}")
        print(f"expected_id: {result['expected_id']}")
        print(f"detected_ids: {result['detected_ids']}")
        print(f"perpendicular_plane_distance: {result['perpendicular_plane_distance']:.6f}")
        print(f"center_distance: {result['center_distance']:.6f}")
        print(f"mean_reprojection_error_px: {marker['mean_reprojection_error_px']:.3f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
