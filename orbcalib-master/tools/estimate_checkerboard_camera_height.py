#!/usr/bin/env python3
"""Estimate camera height above a floor checkerboard from one image."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Tuple

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraCalibration:
    model: str
    K: np.ndarray
    D: np.ndarray
    width: int
    height: int


def parse_orbslam_yaml(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    with path.open("r", encoding="utf-8") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#") or line.startswith("%"):
                continue
            if ":" not in line:
                continue
            key, value = line.split(":", 1)
            values[key.strip()] = value.strip().strip("\"'")
    return values


def read_calibration(path: Path) -> CameraCalibration:
    values = parse_orbslam_yaml(path)
    required = (
        "Camera.type",
        "Camera1.fx",
        "Camera1.fy",
        "Camera1.cx",
        "Camera1.cy",
        "Camera.width",
        "Camera.height",
    )
    missing = [key for key in required if key not in values]
    if missing:
        raise ValueError(f"{path} is missing required keys: {', '.join(missing)}")

    model = values["Camera.type"]
    K = np.array(
        [
            [float(values["Camera1.fx"]), 0.0, float(values["Camera1.cx"])],
            [0.0, float(values["Camera1.fy"]), float(values["Camera1.cy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )

    if model == "KannalaBrandt8":
        distortion_keys = ("Camera1.k1", "Camera1.k2", "Camera1.k3", "Camera1.k4")
    elif model == "PinHole":
        distortion_keys = ("Camera1.k1", "Camera1.k2", "Camera1.p1", "Camera1.p2", "Camera1.k3")
    else:
        raise ValueError(f"Unsupported Camera.type in {path}: {model}")

    missing_distortion = [key for key in distortion_keys if key not in values]
    if missing_distortion:
        raise ValueError(f"{path} is missing distortion keys: {', '.join(missing_distortion)}")

    D = np.array([float(values[key]) for key in distortion_keys], dtype=np.float64).reshape(-1, 1)
    return CameraCalibration(
        model=model,
        K=K,
        D=D,
        width=int(float(values["Camera.width"])),
        height=int(float(values["Camera.height"])),
    )


def scaled_calibration(calib: CameraCalibration, image_size: Tuple[int, int]) -> CameraCalibration:
    width, height = image_size
    if width == calib.width and height == calib.height:
        return calib

    sx = width / float(calib.width)
    sy = height / float(calib.height)
    K = calib.K.copy()
    K[0, 0] *= sx
    K[0, 2] *= sx
    K[1, 1] *= sy
    K[1, 2] *= sy
    return CameraCalibration(model=calib.model, K=K, D=calib.D, width=width, height=height)


def make_checkerboard_points(pattern_size: Tuple[int, int], square_size: float) -> np.ndarray:
    cols, rows = pattern_size
    points = np.zeros((rows * cols, 3), dtype=np.float64)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    points *= square_size
    return points


def find_checkerboard_corners(image: np.ndarray, pattern_size: Tuple[int, int]) -> np.ndarray:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image

    if hasattr(cv2, "findChessboardCornersSB"):
        found, corners = cv2.findChessboardCornersSB(
            gray,
            pattern_size,
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY,
        )
        if found:
            return corners.astype(np.float64)

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        raise RuntimeError(
            f"Could not detect a {pattern_size[0]}x{pattern_size[1]} inner-corner checkerboard"
        )

    criteria = (
        cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
        50,
        0.001,
    )
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria).astype(np.float64)


def estimate_pose(
    object_points: np.ndarray,
    corners: np.ndarray,
    calib: CameraCalibration,
) -> Tuple[np.ndarray, np.ndarray]:
    if calib.model == "KannalaBrandt8":
        undistorted = cv2.fisheye.undistortPoints(corners, calib.K, calib.D)
        camera_matrix = np.eye(3, dtype=np.float64)
        distortion = None
        image_points = undistorted
    else:
        camera_matrix = calib.K
        distortion = calib.D
        image_points = corners

    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        distortion,
        flags=cv2.SOLVEPNP_ITERATIVE,
    )
    if not ok:
        raise RuntimeError("cv2.solvePnP failed")
    return rvec, tvec


def reprojection_error(
    object_points: np.ndarray,
    corners: np.ndarray,
    rvec: np.ndarray,
    tvec: np.ndarray,
    calib: CameraCalibration,
) -> float:
    if calib.model == "KannalaBrandt8":
        projected, _ = cv2.fisheye.projectPoints(
            object_points.reshape(-1, 1, 3),
            rvec,
            tvec,
            calib.K,
            calib.D,
        )
    else:
        projected, _ = cv2.projectPoints(object_points, rvec, tvec, calib.K, calib.D)

    errors = np.linalg.norm(projected.reshape(-1, 2) - corners.reshape(-1, 2), axis=1)
    return float(np.mean(errors))


def estimate_height(
    image_path: Path,
    calib_path: Path,
    pattern_size: Tuple[int, int],
    square_size: float,
) -> Dict[str, object]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Could not read image: {image_path}")

    calib = scaled_calibration(read_calibration(calib_path), (image.shape[1], image.shape[0]))
    object_points = make_checkerboard_points(pattern_size, square_size)
    corners = find_checkerboard_corners(image, pattern_size)
    rvec, tvec = estimate_pose(object_points, corners, calib)

    R, _ = cv2.Rodrigues(rvec)
    camera_center_board = (-R.T @ tvec).reshape(3)
    signed_height = float(camera_center_board[2])

    return {
        "image": str(image_path),
        "calibration": str(calib_path),
        "camera_model": calib.model,
        "image_width": int(image.shape[1]),
        "image_height": int(image.shape[0]),
        "checkerboard_inner_corners": [pattern_size[0], pattern_size[1]],
        "square_size": square_size,
        "height": abs(signed_height),
        "signed_height": signed_height,
        "camera_center_in_checkerboard_frame": camera_center_board.tolist(),
        "rvec_checkerboard_to_camera": rvec.reshape(3).tolist(),
        "tvec_checkerboard_to_camera": tvec.reshape(3).tolist(),
        "mean_reprojection_error_px": reprojection_error(object_points, corners, rvec, tvec, calib),
        "detected_corners": int(corners.shape[0]),
    }


def write_debug_image(
    output_path: Path,
    image_path: Path,
    pattern_size: Tuple[int, int],
    height: float,
) -> None:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Could not read image for debug output: {image_path}")
    corners = find_checkerboard_corners(image, pattern_size)
    cv2.drawChessboardCorners(image, pattern_size, corners.astype(np.float32), True)
    cv2.putText(
        image,
        f"camera height: {height:.4f}",
        (30, 50),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 255, 0),
        2,
        cv2.LINE_AA,
    )
    output_path.parent.mkdir(parents=True, exist_ok=True)
    if not cv2.imwrite(str(output_path), image):
        raise RuntimeError(f"Failed to write debug image: {output_path}")


def parse_pattern_size(value: str) -> Tuple[int, int]:
    normalized = value.lower().replace("x", ",")
    parts = [part.strip() for part in normalized.split(",") if part.strip()]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("pattern size must look like COLSxROWS")
    cols, rows = int(parts[0]), int(parts[1])
    if cols <= 1 or rows <= 1:
        raise argparse.ArgumentTypeError("pattern dimensions must both be greater than 1")
    return cols, rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("image", type=Path, help="Input image containing the floor checkerboard.")
    parser.add_argument(
        "--calib",
        type=Path,
        required=True,
        help="ORB-SLAM/OpenCV camera YAML, e.g. config/sim/agilex_back_defished_cam.yaml.",
    )
    parser.add_argument(
        "--pattern",
        type=parse_pattern_size,
        required=True,
        help="Checkerboard inner-corner count as COLSxROWS, not number of squares.",
    )
    parser.add_argument(
        "--square-size",
        type=float,
        required=True,
        help="Checkerboard square side length. The reported height uses this same unit.",
    )
    parser.add_argument(
        "--debug-image",
        type=Path,
        default=None,
        help="Optional output image with detected checkerboard corners drawn.",
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="Print the full result as JSON.",
    )
    args = parser.parse_args()
    if args.square_size <= 0.0:
        parser.error("--square-size must be positive")
    return args


def main() -> int:
    args = parse_args()
    result = estimate_height(
        args.image.expanduser().resolve(),
        args.calib.expanduser().resolve(),
        args.pattern,
        args.square_size,
    )

    if args.debug_image is not None:
        write_debug_image(
            args.debug_image.expanduser().resolve(),
            args.image.expanduser().resolve(),
            args.pattern,
            float(result["height"]),
        )

    if args.json:
        print(json.dumps(result, indent=2, sort_keys=True))
    else:
        print(f"camera_height: {result['height']:.6f}")
        print(f"signed_height: {result['signed_height']:.6f}")
        print(f"mean_reprojection_error_px: {result['mean_reprojection_error_px']:.3f}")
        print(f"detected_corners: {result['detected_corners']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
