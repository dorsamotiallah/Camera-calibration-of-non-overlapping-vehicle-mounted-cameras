#!/usr/bin/env python3
"""Batch-undistort Agilex fisheye frame folders with OpenCV."""

from __future__ import annotations

import argparse
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, Tuple

import cv2
import numpy as np


CAMERAS = ("front", "back", "left", "right")
IMAGE_EXTENSIONS = {".png", ".jpg", ".jpeg", ".bmp", ".tif", ".tiff"}


@dataclass(frozen=True)
class CameraCalibration:
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
        "Camera1.fx",
        "Camera1.fy",
        "Camera1.cx",
        "Camera1.cy",
        "Camera1.k1",
        "Camera1.k2",
        "Camera1.k3",
        "Camera1.k4",
        "Camera.width",
        "Camera.height",
    )
    missing = [key for key in required if key not in values]
    if missing:
        missing_text = ", ".join(missing)
        raise ValueError(f"{path} is missing required calibration keys: {missing_text}")

    K = np.array(
        [
            [float(values["Camera1.fx"]), 0.0, float(values["Camera1.cx"])],
            [0.0, float(values["Camera1.fy"]), float(values["Camera1.cy"])],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    D = np.array(
        [
            float(values["Camera1.k1"]),
            float(values["Camera1.k2"]),
            float(values["Camera1.k3"]),
            float(values["Camera1.k4"]),
        ],
        dtype=np.float64,
    ).reshape(4, 1)
    return CameraCalibration(
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
    return CameraCalibration(K=K, D=calib.D, width=width, height=height)


def parse_output_size(value: str | None, input_size: Tuple[int, int]) -> Tuple[int, int]:
    if value is None:
        return input_size
    normalized = value.lower().replace("x", ",")
    parts = [part.strip() for part in normalized.split(",") if part.strip()]
    if len(parts) != 2:
        raise argparse.ArgumentTypeError("--output-size must look like WIDTHxHEIGHT")
    width, height = int(parts[0]), int(parts[1])
    if width <= 0 or height <= 0:
        raise argparse.ArgumentTypeError("--output-size dimensions must be positive")
    return width, height


def make_output_camera(
    output_size: Tuple[int, int],
    fov_degrees: float,
    fov_axis: str,
) -> np.ndarray:
    width, height = output_size
    fov_radians = math.radians(fov_degrees)
    if not 1.0 < fov_degrees < 179.0:
        raise ValueError("--fov-deg must be between 1 and 179 degrees for a pinhole output")

    if fov_axis == "horizontal":
        focal = width / (2.0 * math.tan(fov_radians / 2.0))
    elif fov_axis == "vertical":
        focal = height / (2.0 * math.tan(fov_radians / 2.0))
    elif fov_axis == "diagonal":
        diagonal = math.hypot(width, height)
        focal = diagonal / (2.0 * math.tan(fov_radians / 2.0))
    else:
        raise ValueError(f"Unsupported FOV axis: {fov_axis}")

    return np.array(
        [
            [focal, 0.0, (width - 1.0) / 2.0],
            [0.0, focal, (height - 1.0) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


def iter_images(folder: Path) -> Iterable[Path]:
    for path in sorted(folder.rglob("*")):
        if path.is_file() and path.suffix.lower() in IMAGE_EXTENSIONS:
            yield path


def build_calibration_paths(args: argparse.Namespace) -> Dict[str, Path]:
    paths = {
        camera: args.calib_dir / f"agilex_{camera}_cam.yaml"
        for camera in CAMERAS
    }
    for item in args.calib:
        if "=" not in item:
            raise ValueError("--calib entries must look like front=/path/to/file.yaml")
        camera, raw_path = item.split("=", 1)
        camera = camera.strip().lower()
        if camera not in CAMERAS:
            raise ValueError(f"Unknown camera in --calib: {camera}")
        paths[camera] = Path(raw_path).expanduser()
    return paths


def process_camera(
    camera: str,
    input_dir: Path,
    output_dir: Path,
    calib: CameraCalibration,
    args: argparse.Namespace,
) -> Tuple[int, int]:
    image_paths = list(iter_images(input_dir))
    if args.limit is not None:
        image_paths = image_paths[: args.limit]

    written = 0
    skipped = 0
    map_cache = {}

    for image_path in image_paths:
        relative = image_path.relative_to(input_dir)
        target_path = output_dir / relative
        if target_path.exists() and not args.overwrite:
            skipped += 1
            continue

        image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
        if image is None:
            print(f"[{camera}] warning: could not read {image_path}")
            skipped += 1
            continue

        input_size = (image.shape[1], image.shape[0])
        output_size = parse_output_size(args.output_size, input_size)
        cache_key = input_size + output_size
        if cache_key not in map_cache:
            scaled = scaled_calibration(calib, input_size)
            new_K = make_output_camera(output_size, args.fov_deg, args.fov_axis)
            map1, map2 = cv2.fisheye.initUndistortRectifyMap(
                scaled.K,
                scaled.D,
                np.eye(3, dtype=np.float64),
                new_K,
                output_size,
                cv2.CV_16SC2,
            )
            map_cache[cache_key] = (map1, map2)

        if args.dry_run:
            written += 1
            continue

        target_path.parent.mkdir(parents=True, exist_ok=True)
        map1, map2 = map_cache[cache_key]
        undistorted = cv2.remap(
            image,
            map1,
            map2,
            interpolation=args.interpolation,
            borderMode=cv2.BORDER_CONSTANT,
        )

        params = []
        suffix = target_path.suffix.lower()
        if suffix in {".jpg", ".jpeg"}:
            params = [cv2.IMWRITE_JPEG_QUALITY, args.jpeg_quality]
        if not cv2.imwrite(str(target_path), undistorted, params):
            raise RuntimeError(f"Failed to write {target_path}")
        written += 1

    return written, skipped


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Defish Agilex frame folders named front/back/left/right using "
            "ORB-SLAM3/OpenCV fisheye intrinsics."
        )
    )
    parser.add_argument("frames_root", type=Path, help="Root directory containing front/back/left/right")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=None,
        help="Output root. Defaults to '<frames_root>_defished'.",
    )
    parser.add_argument(
        "--calib-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "config" / "sim",
        help="Directory containing agilex_front_cam.yaml, agilex_back_cam.yaml, etc.",
    )
    parser.add_argument(
        "--calib",
        action="append",
        default=[],
        help="Override one calibration path, e.g. --calib front=/tmp/front.yaml. Can be repeated.",
    )
    parser.add_argument(
        "--fov-deg",
        type=float,
        required=True,
        help="Pinhole output field of view to preserve, in degrees.",
    )
    parser.add_argument(
        "--fov-axis",
        choices=("horizontal", "vertical", "diagonal"),
        default="horizontal",
        help="Axis that --fov-deg refers to. Default: horizontal.",
    )
    parser.add_argument(
        "--output-size",
        default=None,
        help="Optional output size as WIDTHxHEIGHT. Defaults to each input image size.",
    )
    parser.add_argument(
        "--cameras",
        nargs="+",
        choices=CAMERAS,
        default=list(CAMERAS),
        help="Camera folders to process. Default: front back left right.",
    )
    parser.add_argument("--overwrite", action="store_true", help="Overwrite existing output images.")
    parser.add_argument("--dry-run", action="store_true", help="Report what would be processed without writing.")
    parser.add_argument("--limit", type=int, default=None, help="Process at most N images per camera.")
    parser.add_argument("--jpeg-quality", type=int, default=95, help="JPEG output quality. Default: 95.")
    parser.add_argument(
        "--interpolation",
        choices=("nearest", "linear", "cubic", "lanczos"),
        default="linear",
        help="OpenCV remap interpolation. Default: linear.",
    )
    args = parser.parse_args()

    interpolation_map = {
        "nearest": cv2.INTER_NEAREST,
        "linear": cv2.INTER_LINEAR,
        "cubic": cv2.INTER_CUBIC,
        "lanczos": cv2.INTER_LANCZOS4,
    }
    args.interpolation = interpolation_map[args.interpolation]
    if args.limit is not None and args.limit <= 0:
        parser.error("--limit must be positive")
    if not 1 <= args.jpeg_quality <= 100:
        parser.error("--jpeg-quality must be between 1 and 100")
    return args


def main() -> int:
    args = parse_args()
    frames_root = args.frames_root.expanduser().resolve()
    output_root = args.output_root
    if output_root is None:
        output_root = frames_root.with_name(f"{frames_root.name}_defished")
    output_root = output_root.expanduser().resolve()

    calibration_paths = build_calibration_paths(args)
    calibrations = {
        camera: read_calibration(calibration_paths[camera])
        for camera in args.cameras
    }

    total_written = 0
    total_skipped = 0
    for camera in args.cameras:
        input_dir = frames_root / camera
        if not input_dir.is_dir():
            print(f"[{camera}] skipping missing folder: {input_dir}")
            continue

        output_dir = output_root / camera
        written, skipped = process_camera(camera, input_dir, output_dir, calibrations[camera], args)
        total_written += written
        total_skipped += skipped
        action = "would write" if args.dry_run else "wrote"
        print(f"[{camera}] {action} {written} image(s), skipped {skipped}")

    print(f"Done. Output root: {output_root}")
    print(f"Total: {total_written} image(s), skipped {total_skipped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
