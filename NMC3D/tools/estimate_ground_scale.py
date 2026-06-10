#!/usr/bin/env python3
"""Estimate monocular map scale from a ground plane in a saved ORB-SLAM atlas.

The script uses the repo's C++ atlas exporter to read the Boost .osa atlas and
then runs a scored RANSAC plane search on the exported keyframe observations.
It reports the metric scale that should be applied later to monocular
ORB-SLAM/NMC3D translation results.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import re
import subprocess
import tempfile
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


REPO_NMC3D = Path(__file__).resolve().parents[1]
DEFAULT_VOCABULARY = REPO_NMC3D / "Vocabulary" / "ORBvoc.txt"


def default_exporter_candidates() -> list[Path]:
    return [
        REPO_NMC3D / "build_nmc_docker" / "calib" / "atlas_ground_export",
        REPO_NMC3D / "build_nmc" / "calib" / "atlas_ground_export",
        REPO_NMC3D / "build" / "calib" / "atlas_ground_export",
    ]


@dataclass
class PlaneCandidate:
    normal: np.ndarray
    d: float
    score: float
    inlier_ratio: float
    bottom_ratio: float
    camera_side_ratio: float
    height_consistency: float
    coverage: float
    median_height_slam: float
    mad_height_slam: float
    num_inliers: int
    inlier_mask: np.ndarray


@dataclass
class KeyframeScaleEstimate:
    kf_id: int
    kf_time: float
    raw_scale: float
    smoothed_scale: float
    height_slam: float
    plane_score: float
    num_observations: int
    num_candidates: int
    num_inliers: int
    inlier_ratio: float
    bottom_ratio: float
    camera_side: float
    coverage: float
    accepted: bool
    reason: str


@dataclass
class RobustScaleSummary:
    scale: float
    mad: float
    num_used: int
    num_total: int
    lower: float
    upper: float
    source: str


def infer_sequence_and_prefix(atlas: Path, sequence: str | None) -> tuple[str, str]:
    name = atlas.name
    if not name.endswith(".osa"):
        raise ValueError(f"Atlas must end in .osa: {atlas}")

    stem = name[:-4]
    if sequence is not None:
        seq = sequence
        if seq and not stem.endswith(seq):
            raise ValueError(f"Atlas stem '{stem}' does not end with sequence '{seq}'")
        prefix = stem[: -len(seq)] if seq else stem
        return seq, prefix

    match = re.match(r"^(.*?)(Camera\s+\d+)$", stem)
    if match:
        return match.group(2), match.group(1)

    return "", stem


def infer_settings(atlas: Path, explicit_settings: str | None) -> Path:
    if explicit_settings:
        return Path(explicit_settings).resolve()

    lower = atlas.name.lower()
    candidates: list[Path] = []
    if lower.startswith("c1_") or "c1_atlas" in lower:
        candidates.append(REPO_NMC3D / "config" / "sim" / "C1.yaml")
    if lower.startswith("c4_") or "c4_atlas" in lower:
        candidates.append(REPO_NMC3D / "config" / "sim" / "C4.yaml")
    candidates.append(REPO_NMC3D / "config" / "sim" / "mono.yaml")

    for candidate in candidates:
        if candidate.exists():
            return candidate

    raise FileNotFoundError("Could not infer a camera settings file; pass --settings.")


def write_temp_settings(base_settings: Path, atlas_prefix: str, tmpdir: Path) -> Path:
    text = base_settings.read_text()
    replacement = f'System.LoadAtlasFromFile: "{atlas_prefix}"'

    # The exporter is read-only. Some SLAM configs contain SaveAtlasToFile for
    # normal runs, but keeping it here would make Shutdown write another atlas.
    text = re.sub(r"^System\.SaveAtlasToFile:.*$\n?", "", text, flags=re.MULTILINE)

    if re.search(r"^System\.LoadAtlasFromFile:.*$", text, flags=re.MULTILINE):
        text = re.sub(r"^System\.LoadAtlasFromFile:.*$", replacement, text, flags=re.MULTILINE)
    else:
        lines = text.splitlines()
        insert_at = 1 if lines and lines[0].startswith("%YAML") else 0
        lines.insert(insert_at, replacement)
        text = "\n".join(lines) + "\n"

    tmp_settings = tmpdir / "atlas_ground_settings.yaml"
    tmp_settings.write_text(text)
    return tmp_settings


def run_exporter(args: argparse.Namespace, tmpdir: Path, atlas_prefix: str, sequence: str) -> Path:
    if args.exporter:
        exporter = Path(args.exporter).resolve()
    else:
        exporter = next((p.resolve() for p in default_exporter_candidates() if p.exists()), default_exporter_candidates()[0])

    if not exporter.exists():
        raise FileNotFoundError(
            f"Exporter not found: {exporter}\n"
            "Build it first, for example: cmake --build NMC3D/build_nmc_docker --target atlas_ground_export"
        )

    atlas = Path(args.atlas).resolve()
    base_settings = infer_settings(atlas, args.settings)
    settings = write_temp_settings(base_settings, atlas_prefix, tmpdir)
    output_csv = tmpdir / "atlas_observations.csv"

    cmd = [
        str(exporter),
        str(Path(args.vocabulary).resolve()),
        str(settings),
        sequence,
        str(output_csv),
    ]

    env = os.environ.copy()
    build_root = exporter.parent.parent
    lib_dirs = [
        build_root / "orbslam3",
        build_root / "orbslam3" / "Thirdparty" / "DBoW2",
        build_root / "orbslam3" / "Thirdparty" / "g2o",
    ]
    existing_ld = env.get("LD_LIBRARY_PATH", "")
    env["LD_LIBRARY_PATH"] = ":".join([str(p) for p in lib_dirs if p.exists()] + ([existing_ld] if existing_ld else []))

    subprocess.run(cmd, cwd=str(atlas.parent), env=env, check=True)
    return output_csv


def load_observations(csv_path: Path) -> tuple[np.ndarray, np.ndarray, dict[int, list[float]], dict[int, np.ndarray]]:
    mp_positions: dict[int, np.ndarray] = {}
    camera_centers: dict[int, np.ndarray] = {}
    bottom_by_mp: dict[int, list[float]] = {}

    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            mp_id = int(row["mp_id"])
            kf_id = int(row["kf_id"])
            mp_positions.setdefault(
                mp_id,
                np.array([float(row["pw_x"]), float(row["pw_y"]), float(row["pw_z"])], dtype=float),
            )
            camera_centers.setdefault(
                kf_id,
                np.array([float(row["cx"]), float(row["cy"]), float(row["cz"])], dtype=float),
            )

            min_y = float(row["img_min_y"])
            max_y = float(row["img_max_y"])
            denom = max(max_y - min_y, 1.0)
            y_norm = (float(row["kp_y"]) - min_y) / denom
            bottom_by_mp.setdefault(mp_id, []).append(y_norm)

    if len(mp_positions) < 3:
        raise ValueError(f"Need at least 3 map points, got {len(mp_positions)}")

    mp_ids = np.array(list(mp_positions.keys()), dtype=np.int64)
    points = np.vstack([mp_positions[int(mp_id)] for mp_id in mp_ids])
    cameras = np.vstack(list(camera_centers.values()))
    return mp_ids, points, bottom_by_mp, {i: c for i, c in camera_centers.items()}


def fit_plane_from_three(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    p1, p2, p3 = points
    normal = np.cross(p2 - p1, p3 - p1)
    norm = np.linalg.norm(normal)
    if norm < 1e-9:
        return None
    normal = normal / norm
    d = -float(np.dot(normal, p1))
    return normal, d


def plane_distances(points: np.ndarray, normal: np.ndarray, d: float) -> np.ndarray:
    return points @ normal + d


def plane_coverage(inlier_points: np.ndarray, grid_size: int) -> float:
    if len(inlier_points) < 3:
        return 0.0

    centered = inlier_points - np.mean(inlier_points, axis=0)
    _, singular_values, vh = np.linalg.svd(centered, full_matrices=False)
    if len(singular_values) < 2 or singular_values[1] < 1e-9:
        return 0.0

    uv = centered @ vh[:2].T
    mins = uv.min(axis=0)
    maxs = uv.max(axis=0)
    span = np.maximum(maxs - mins, 1e-9)
    ij = np.floor((uv - mins) / span * grid_size).astype(int)
    ij = np.clip(ij, 0, grid_size - 1)
    occupied = len({(int(i), int(j)) for i, j in ij})
    occupancy = occupied / float(grid_size * grid_size)

    # A broad plane patch has two meaningful singular values. Narrow strips are
    # less useful for scale, even if they have many inliers.
    shape = min(float(singular_values[1] / max(singular_values[0], 1e-9)), 1.0)
    return math.sqrt(occupancy * shape)


def score_plane(
    mp_ids: np.ndarray,
    points: np.ndarray,
    cameras: np.ndarray,
    bottom_by_mp: dict[int, list[float]],
    normal: np.ndarray,
    d: float,
    distance_threshold: float,
    bottom_threshold: float,
    grid_size: int,
) -> PlaneCandidate | None:
    point_signed = plane_distances(points, normal, d)
    inlier_mask = np.abs(point_signed) <= distance_threshold
    num_inliers = int(np.count_nonzero(inlier_mask))
    if num_inliers < 3:
        return None

    camera_signed = plane_distances(cameras, normal, d)
    if np.median(camera_signed) < 0.0:
        normal = -normal
        d = -d
        camera_signed = -camera_signed

    camera_distances = np.abs(camera_signed)
    median_height = float(np.median(camera_distances))
    if not np.isfinite(median_height) or median_height <= 1e-9:
        return None

    mad_height = float(np.median(np.abs(camera_distances - median_height)))
    height_consistency = 1.0 / (1.0 + mad_height / median_height)
    camera_side_ratio = float(np.mean(camera_signed > 0.0))

    inlier_ids = mp_ids[inlier_mask]
    bottom_values: list[float] = []
    for mp_id in inlier_ids:
        bottom_values.extend(bottom_by_mp.get(int(mp_id), []))
    bottom_ratio = float(np.mean(np.array(bottom_values) >= bottom_threshold)) if bottom_values else 0.0

    coverage = plane_coverage(points[inlier_mask], grid_size)
    inlier_ratio = num_inliers / float(len(points))

    score = (
        1.50 * inlier_ratio
        + 1.25 * bottom_ratio
        + 1.25 * camera_side_ratio
        + 1.00 * height_consistency
        + 0.75 * coverage
    )

    return PlaneCandidate(
        normal=normal,
        d=float(d),
        score=float(score),
        inlier_ratio=float(inlier_ratio),
        bottom_ratio=bottom_ratio,
        camera_side_ratio=camera_side_ratio,
        height_consistency=float(height_consistency),
        coverage=float(coverage),
        median_height_slam=median_height,
        mad_height_slam=mad_height,
        num_inliers=num_inliers,
        inlier_mask=inlier_mask.copy(),
    )


def write_ply_points(path: Path, points: np.ndarray, colors: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        for p, c in zip(points, colors):
            f.write(f"{p[0]:.9g} {p[1]:.9g} {p[2]:.9g} {int(c[0])} {int(c[1])} {int(c[2])}\n")


def write_ply_mesh(path: Path, vertices: np.ndarray, faces: list[tuple[int, int, int]], color: tuple[int, int, int]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w") as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(vertices)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write(f"element face {len(faces)}\n")
        f.write("property list uchar int vertex_indices\n")
        f.write("end_header\n")
        for v in vertices:
            f.write(f"{v[0]:.9g} {v[1]:.9g} {v[2]:.9g} {color[0]} {color[1]} {color[2]}\n")
        for face in faces:
            f.write(f"3 {face[0]} {face[1]} {face[2]}\n")


def plane_basis(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    axis = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(axis, normal))) > 0.9:
        axis = np.array([0.0, 1.0, 0.0])
    u = np.cross(normal, axis)
    u = u / max(np.linalg.norm(u), 1e-12)
    v = np.cross(normal, u)
    v = v / max(np.linalg.norm(v), 1e-12)
    return u, v


def export_visualization(out_dir: Path, points: np.ndarray, cameras: np.ndarray, candidate: PlaneCandidate) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)

    colors = np.zeros((len(points), 3), dtype=np.uint8)
    colors[:] = np.array([120, 120, 120], dtype=np.uint8)
    colors[candidate.inlier_mask] = np.array([30, 180, 70], dtype=np.uint8)
    write_ply_points(out_dir / "map_points_selected_plane.ply", points, colors)

    camera_colors = np.tile(np.array([[40, 90, 255]], dtype=np.uint8), (len(cameras), 1))
    write_ply_points(out_dir / "camera_centers.ply", cameras, camera_colors)

    inliers = points[candidate.inlier_mask]
    center = np.mean(inliers, axis=0) if len(inliers) else -candidate.d * candidate.normal
    u, v = plane_basis(candidate.normal)
    projected = np.column_stack(((inliers - center) @ u, (inliers - center) @ v)) if len(inliers) else np.zeros((0, 2))
    if len(projected):
        extent = np.percentile(np.abs(projected), 95, axis=0)
        extent = np.maximum(extent, np.array([0.1, 0.1]))
    else:
        extent = np.array([1.0, 1.0])

    corners = np.array([
        center - extent[0] * u - extent[1] * v,
        center + extent[0] * u - extent[1] * v,
        center + extent[0] * u + extent[1] * v,
        center - extent[0] * u + extent[1] * v,
    ])
    write_ply_mesh(out_dir / "selected_plane_patch.ply", corners, [(0, 1, 2), (0, 2, 3)], (255, 210, 40))

    with (out_dir / "selected_plane.txt").open("w") as f:
        f.write(f"normal {candidate.normal[0]:.9g} {candidate.normal[1]:.9g} {candidate.normal[2]:.9g}\n")
        f.write(f"d {candidate.d:.9g}\n")
        f.write(f"score {candidate.score:.9g}\n")
        f.write(f"inliers {candidate.num_inliers}\n")
        f.write(f"median_height_slam {candidate.median_height_slam:.9g}\n")


def export_candidate_visualizations(out_dir: Path, points: np.ndarray, cameras: np.ndarray, candidates: list[PlaneCandidate], camera_height: float) -> None:
    for idx, candidate in enumerate(candidates, start=1):
        candidate_dir = out_dir / f"candidate_{idx:02d}_scale_{camera_height / candidate.median_height_slam:.4g}"
        export_visualization(candidate_dir, points, cameras, candidate)


def is_duplicate_plane(candidate: PlaneCandidate, candidates: Iterable[PlaneCandidate], angle_cos: float, d_tol: float) -> bool:
    for other in candidates:
        cos = abs(float(np.dot(candidate.normal, other.normal)))
        if cos >= angle_cos and abs(abs(candidate.d) - abs(other.d)) <= d_tol:
            return True
    return False


def scored_ransac(
    mp_ids: np.ndarray,
    points: np.ndarray,
    cameras: np.ndarray,
    bottom_by_mp: dict[int, list[float]],
    args: argparse.Namespace,
) -> list[PlaneCandidate]:
    rng = np.random.default_rng(args.seed)
    candidates: list[PlaneCandidate] = []
    n_points = len(points)

    for _ in range(args.iterations):
        idx = rng.choice(n_points, size=3, replace=False)
        fit = fit_plane_from_three(points[idx])
        if fit is None:
            continue

        candidate = score_plane(
            mp_ids,
            points,
            cameras,
            bottom_by_mp,
            fit[0],
            fit[1],
            args.distance_threshold,
            args.bottom_threshold,
            args.coverage_grid,
        )
        if candidate is None or candidate.num_inliers < args.min_inliers:
            continue

        if is_duplicate_plane(candidate, candidates, angle_cos=0.995, d_tol=args.distance_threshold):
            continue

        candidates.append(candidate)
        candidates.sort(key=lambda c: c.score, reverse=True)
        del candidates[args.keep_candidates :]

    if not candidates:
        raise RuntimeError("RANSAC did not find any valid plane candidates.")

    return candidates


def load_keyframe_groups(csv_path: Path) -> dict[int, list[dict[str, float]]]:
    groups: dict[int, list[dict[str, float]]] = defaultdict(list)
    with csv_path.open(newline="") as f:
        reader = csv.DictReader(f)
        for row in reader:
            min_y = float(row["img_min_y"])
            max_y = float(row["img_max_y"])
            denom = max(max_y - min_y, 1.0)
            groups[int(row["kf_id"])].append(
                {
                    "kf_id": float(row["kf_id"]),
                    "kf_time": float(row["kf_time"]),
                    "mp_id": float(row["mp_id"]),
                    "cx": float(row["cx"]),
                    "cy": float(row["cy"]),
                    "cz": float(row["cz"]),
                    "pw_x": float(row["pw_x"]),
                    "pw_y": float(row["pw_y"]),
                    "pw_z": float(row["pw_z"]),
                    "y_norm": (float(row["kp_y"]) - min_y) / denom,
                }
            )
    return dict(groups)


def local_plane_candidate(
    points: np.ndarray,
    y_norm: np.ndarray,
    camera: np.ndarray,
    normal: np.ndarray,
    d: float,
    distance_threshold: float,
    bottom_threshold: float,
    grid_size: int,
) -> PlaneCandidate | None:
    signed = plane_distances(points, normal, d)
    inlier_mask = np.abs(signed) <= distance_threshold
    num_inliers = int(np.count_nonzero(inlier_mask))
    if num_inliers < 3:
        return None

    camera_signed = float(np.dot(camera, normal) + d)
    if camera_signed < 0.0:
        normal = -normal
        d = -d
        camera_signed = -camera_signed

    height = abs(camera_signed)
    if not np.isfinite(height) or height <= 1e-9:
        return None

    inlier_ratio = num_inliers / float(len(points))
    bottom_ratio = float(np.mean(y_norm[inlier_mask] >= bottom_threshold))
    camera_side = 1.0 if camera_signed > 0.0 else 0.0
    coverage = plane_coverage(points[inlier_mask], grid_size)

    # Local keyframe score: inlier support and bottom-image support matter most.
    # Height consistency is evaluated temporally by the smoothing stage.
    score = (
        1.75 * inlier_ratio
        + 1.25 * bottom_ratio
        + 1.00 * camera_side
        + 0.75 * coverage
    )

    return PlaneCandidate(
        normal=normal,
        d=float(d),
        score=float(score),
        inlier_ratio=float(inlier_ratio),
        bottom_ratio=bottom_ratio,
        camera_side_ratio=camera_side,
        height_consistency=1.0,
        coverage=float(coverage),
        median_height_slam=float(height),
        mad_height_slam=0.0,
        num_inliers=num_inliers,
        inlier_mask=inlier_mask.copy(),
    )


def fit_local_keyframe_plane(rows: list[dict[str, float]], args: argparse.Namespace) -> tuple[PlaneCandidate | None, int, int, str]:
    by_mp: dict[int, dict[str, float]] = {}
    for row in rows:
        by_mp.setdefault(int(row["mp_id"]), row)

    unique_rows = list(by_mp.values())
    all_points = np.array([[r["pw_x"], r["pw_y"], r["pw_z"]] for r in unique_rows], dtype=float)
    all_y_norm = np.array([r["y_norm"] for r in unique_rows], dtype=float)
    candidate_mask = all_y_norm >= args.bottom_threshold
    candidate_points = all_points[candidate_mask]
    candidate_y_norm = all_y_norm[candidate_mask]
    camera = np.array([unique_rows[0]["cx"], unique_rows[0]["cy"], unique_rows[0]["cz"]], dtype=float)

    if len(candidate_points) < args.local_min_inliers:
        return None, len(all_points), len(candidate_points), "too_few_bottom_candidates"

    rng = np.random.default_rng(args.seed + int(unique_rows[0]["kf_id"]))
    best: PlaneCandidate | None = None
    for _ in range(args.local_iterations):
        idx = rng.choice(len(candidate_points), size=3, replace=False)
        fit = fit_plane_from_three(candidate_points[idx])
        if fit is None:
            continue
        candidate = local_plane_candidate(
            candidate_points,
            candidate_y_norm,
            camera,
            fit[0],
            fit[1],
            args.distance_threshold,
            args.bottom_threshold,
            args.coverage_grid,
        )
        if candidate is None or candidate.num_inliers < args.local_min_inliers:
            continue
        if best is None or candidate.score > best.score:
            best = candidate

    if best is None:
        return None, len(all_points), len(candidate_points), "ransac_failed"
    return best, len(all_points), len(candidate_points), "ok"


def smooth_keyframe_scales(
    raw_rows: list[tuple[int, float, PlaneCandidate | None, int, int, str]],
    camera_height: float,
    args: argparse.Namespace,
) -> list[KeyframeScaleEstimate]:
    estimates: list[KeyframeScaleEstimate] = []
    accepted_scales: list[float] = []
    last_smoothed: float | None = None

    for kf_id, kf_time, candidate, n_obs, n_candidates, reason in raw_rows:
        if candidate is None:
            smoothed = last_smoothed if last_smoothed is not None else float("nan")
            estimates.append(
                KeyframeScaleEstimate(
                    kf_id=kf_id,
                    kf_time=kf_time,
                    raw_scale=float("nan"),
                    smoothed_scale=smoothed,
                    height_slam=float("nan"),
                    plane_score=0.0,
                    num_observations=n_obs,
                    num_candidates=n_candidates,
                    num_inliers=0,
                    inlier_ratio=0.0,
                    bottom_ratio=0.0,
                    camera_side=0.0,
                    coverage=0.0,
                    accepted=False,
                    reason=reason,
                )
            )
            continue

        raw_scale = camera_height / candidate.median_height_slam
        mean_recent = float(np.mean(accepted_scales[-args.smoothing_window :])) if accepted_scales else raw_scale
        change = abs(raw_scale - mean_recent) / max(abs(mean_recent), 1e-12)

        accepted = True
        reason = "accepted"
        if candidate.score < args.min_plane_score:
            accepted = False
            reason = "low_plane_score"
        elif candidate.num_inliers < args.local_min_inliers:
            accepted = False
            reason = "too_few_inliers"
        elif accepted_scales and change > args.scale_high_change:
            accepted = False
            reason = "scale_jump_rejected"

        if not accepted:
            smoothed = last_smoothed if last_smoothed is not None else mean_recent
        elif not accepted_scales:
            smoothed = raw_scale
            accepted_scales.append(raw_scale)
        else:
            if change <= args.scale_low_change:
                raw_weight = 0.75
            else:
                span = max(args.scale_high_change - args.scale_low_change, 1e-12)
                alpha = min(max((change - args.scale_low_change) / span, 0.0), 1.0)
                raw_weight = 0.75 * (1.0 - alpha) + 0.25 * alpha
            smoothed = raw_weight * raw_scale + (1.0 - raw_weight) * mean_recent
            accepted_scales.append(raw_scale)

        last_smoothed = smoothed
        estimates.append(
            KeyframeScaleEstimate(
                kf_id=kf_id,
                kf_time=kf_time,
                raw_scale=float(raw_scale),
                smoothed_scale=float(smoothed),
                height_slam=float(candidate.median_height_slam),
                plane_score=float(candidate.score),
                num_observations=n_obs,
                num_candidates=n_candidates,
                num_inliers=candidate.num_inliers,
                inlier_ratio=float(candidate.inlier_ratio),
                bottom_ratio=float(candidate.bottom_ratio),
                camera_side=float(candidate.camera_side_ratio),
                coverage=float(candidate.coverage),
                accepted=accepted,
                reason=reason,
            )
        )

    return estimates


def write_keyframe_scale_csv(path: Path, estimates: list[KeyframeScaleEstimate]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(
            [
                "kf_id",
                "kf_time",
                "raw_scale",
                "smoothed_scale",
                "height_slam",
                "plane_score",
                "num_observations",
                "num_candidates",
                "num_inliers",
                "inlier_ratio",
                "bottom_ratio",
                "camera_side",
                "coverage",
                "accepted",
                "reason",
            ]
        )
        for e in estimates:
            writer.writerow(
                [
                    e.kf_id,
                    f"{e.kf_time:.9g}",
                    f"{e.raw_scale:.9g}",
                    f"{e.smoothed_scale:.9g}",
                    f"{e.height_slam:.9g}",
                    f"{e.plane_score:.9g}",
                    e.num_observations,
                    e.num_candidates,
                    e.num_inliers,
                    f"{e.inlier_ratio:.9g}",
                    f"{e.bottom_ratio:.9g}",
                    f"{e.camera_side:.9g}",
                    f"{e.coverage:.9g}",
                    int(e.accepted),
                    e.reason,
                ]
            )


def estimate_keyframe_scale_curve(csv_path: Path, args: argparse.Namespace) -> list[KeyframeScaleEstimate]:
    groups = load_keyframe_groups(csv_path)
    raw_rows: list[tuple[int, float, PlaneCandidate | None, int, int, str]] = []
    for kf_id in sorted(groups):
        rows = groups[kf_id]
        kf_time = rows[0]["kf_time"]
        candidate, n_obs, n_candidates, reason = fit_local_keyframe_plane(rows, args)
        raw_rows.append((kf_id, kf_time, candidate, n_obs, n_candidates, reason))

    return smooth_keyframe_scales(raw_rows, args.camera_height, args)


def robust_keyframe_scale_summary(estimates: list[KeyframeScaleEstimate], args: argparse.Namespace) -> RobustScaleSummary | None:
    accepted = np.array([e.raw_scale for e in estimates if e.accepted and np.isfinite(e.raw_scale)], dtype=float)
    source = "accepted_raw_scale"
    values = accepted

    if len(values) < args.robust_min_cluster_size:
        fallback = np.array([e.raw_scale for e in estimates if np.isfinite(e.raw_scale)], dtype=float)
        values = fallback
        source = "all_raw_scale_fallback"

    values = values[np.isfinite(values) & (values > 0.0)]
    if len(values) == 0:
        return None

    log_values = np.log(values)
    center = float(np.median(log_values))
    log_mad = float(np.median(np.abs(log_values - center)))
    sigma = max(1.4826 * log_mad, 1e-9)
    max_sigma = max(args.robust_cluster_sigma, 1e-9)

    mask = np.abs(log_values - center) <= max_sigma * sigma
    if int(np.count_nonzero(mask)) < args.robust_min_cluster_size and source == "accepted_raw_scale":
        # The accepted set is too fragmented. Fall back to all finite local
        # estimates, but still cluster in log-scale so outliers cannot dominate.
        values = np.array([e.raw_scale for e in estimates if np.isfinite(e.raw_scale) and e.raw_scale > 0.0], dtype=float)
        source = "all_raw_scale_fallback"
        log_values = np.log(values)
        center = float(np.median(log_values))
        log_mad = float(np.median(np.abs(log_values - center)))
        sigma = max(1.4826 * log_mad, 1e-9)
        mask = np.abs(log_values - center) <= max_sigma * sigma

    clustered = values[mask]
    if len(clustered) == 0:
        clustered = values
        mask = np.ones_like(values, dtype=bool)

    scale = float(np.median(clustered))
    mad = float(np.median(np.abs(clustered - scale)))
    lower = float(np.min(clustered))
    upper = float(np.max(clustered))
    return RobustScaleSummary(
        scale=scale,
        mad=mad,
        num_used=int(len(clustered)),
        num_total=int(len(values)),
        lower=lower,
        upper=upper,
        source=source,
    )


def print_keyframe_scale_summary(estimates: list[KeyframeScaleEstimate], args: argparse.Namespace) -> None:
    accepted = [e for e in estimates if e.accepted and np.isfinite(e.raw_scale)]
    print("\nPer-keyframe scale estimate")
    print(f"  keyframes: {len(estimates)}")
    print(f"  accepted: {len(accepted)}")
    print(f"  rejected: {len(estimates) - len(accepted)}")
    if not accepted:
        print("  no accepted keyframe scale estimates")
        return

    raw = np.array([e.raw_scale for e in accepted], dtype=float)
    smooth = np.array([e.smoothed_scale for e in accepted], dtype=float)
    heights = np.array([e.height_slam for e in accepted], dtype=float)
    print(f"  raw scale median: {np.median(raw):.8g} m / SLAM unit")
    print(f"  raw scale MAD: {np.median(np.abs(raw - np.median(raw))):.8g}")
    print(f"  smoothed final scale: {smooth[-1]:.8g} m / SLAM unit")
    print(f"  smoothed median scale: {np.median(smooth):.8g} m / SLAM unit")
    print(f"  median camera-plane height: {np.median(heights):.8g} SLAM units")
    print(f"  real camera height: {args.camera_height:.8g} m")
    robust = robust_keyframe_scale_summary(estimates, args)
    if robust is not None:
        print("  robust cluster scale for global map scaling")
        print(f"    source: {robust.source}")
        print(f"    used: {robust.num_used} / {robust.num_total}")
        print(f"    scale: {robust.scale:.8g} m / SLAM unit")
        print(f"    MAD: {robust.mad:.8g}")
        print(f"    range: [{robust.lower:.8g}, {robust.upper:.8g}]")
    print("  last accepted estimates")
    for e in accepted[-5:]:
        print(
            f"    kf={e.kf_id} raw={e.raw_scale:.6g} smooth={e.smoothed_scale:.6g} "
            f"height={e.height_slam:.6g} score={e.plane_score:.3f} inliers={e.num_inliers}"
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--atlas", required=True, help="Path to a saved .osa atlas.")
    parser.add_argument("--camera-height", type=float, help="Real camera optical-center height above ground, in meters.")
    parser.add_argument("--settings", help="Camera settings YAML. Inferred for c1/c4 FinnForest atlases when omitted.")
    parser.add_argument("--sequence", help='Atlas sequence suffix, e.g. "Camera 1". Inferred from filenames like c1_atlasCamera 1.osa.')
    parser.add_argument("--vocabulary", default=str(DEFAULT_VOCABULARY), help="ORB vocabulary path.")
    parser.add_argument("--exporter", help="Path to atlas_ground_export executable.")
    parser.add_argument("--distance-threshold", type=float, default=0.03, help="RANSAC inlier distance in SLAM units.")
    parser.add_argument("--bottom-threshold", type=float, default=0.50, help="Normalized image y threshold for bottom-image score.")
    parser.add_argument("--iterations", type=int, default=5000)
    parser.add_argument("--min-inliers", type=int, default=50)
    parser.add_argument("--keep-candidates", type=int, default=8)
    parser.add_argument("--coverage-grid", type=int, default=8)
    parser.add_argument("--seed", type=int, default=4)
    parser.add_argument("--keep-export", help="Optional path to keep the intermediate observation CSV.")
    parser.add_argument("--export-only", action="store_true", help="Only export atlas observations CSV for MATLAB; do not estimate scale.")
    parser.add_argument("--viz-dir", help="Optional directory for PLY visualization of selected plane/inliers.")
    parser.add_argument("--viz-candidates", action="store_true", help="When --viz-dir is set, export all kept plane candidates, not only the best one.")
    parser.add_argument("--per-keyframe", action="store_true", help="Also estimate a paper-style per-keyframe scale curve.")
    parser.add_argument("--per-keyframe-output", help="Optional CSV path for the per-keyframe scale curve.")
    parser.add_argument("--local-iterations", type=int, default=1000, help="RANSAC iterations per keyframe for --per-keyframe.")
    parser.add_argument("--local-min-inliers", type=int, default=20, help="Minimum local plane inliers for --per-keyframe.")
    parser.add_argument("--min-plane-score", type=float, default=2.8, help="Reject per-keyframe planes below this score.")
    parser.add_argument("--smoothing-window", type=int, default=5, help="Moving average window for accepted per-keyframe scales.")
    parser.add_argument("--scale-low-change", type=float, default=0.10, help="Relative scale change treated as stable.")
    parser.add_argument("--scale-high-change", type=float, default=0.50, help="Relative scale change rejected as a jump.")
    parser.add_argument("--robust-cluster-sigma", type=float, default=2.5, help="Log-scale MAD gate for the robust per-keyframe global scale.")
    parser.add_argument("--robust-min-cluster-size", type=int, default=5, help="Minimum cluster size for the robust per-keyframe global scale.")
    parser.add_argument("--scale-summary-output", help="Optional CSV path for the selected global scale summaries.")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.export_only and not args.keep_export:
        raise SystemExit("--export-only requires --keep-export because the temporary CSV is deleted after the tool exits.")
    if not args.export_only and args.camera_height is None:
        raise SystemExit("--camera-height is required unless --export-only is used.")

    atlas = Path(args.atlas).resolve()
    sequence, prefix = infer_sequence_and_prefix(atlas, args.sequence)

    with tempfile.TemporaryDirectory(prefix="ground_scale_") as tmp:
        tmpdir = Path(tmp)
        csv_path = run_exporter(args, tmpdir, prefix, sequence)
        if args.keep_export:
            keep_path = Path(args.keep_export).resolve()
            keep_path.parent.mkdir(parents=True, exist_ok=True)
            keep_path.write_text(csv_path.read_text())
            print(f"Atlas observations CSV written to: {keep_path}")

        if args.export_only:
            print("Export-only mode: skipped Python ground-scale and per-keyframe scale estimation.")
            return 0

        mp_ids, points, bottom_by_mp, camera_dict = load_observations(csv_path)
        cameras = np.vstack(list(camera_dict.values()))

        candidates = scored_ransac(mp_ids, points, cameras, bottom_by_mp, args)
        best = candidates[0]
        scale = args.camera_height / best.median_height_slam

        print("\nGround scale estimate")
        print(f"  atlas: {atlas}")
        print(f"  sequence: {sequence!r}")
        print(f"  unique map points: {len(points)}")
        print(f"  keyframe camera centers: {len(cameras)}")
        print(f"  plane: n=({best.normal[0]:.8g}, {best.normal[1]:.8g}, {best.normal[2]:.8g}), d={best.d:.8g}")
        print(f"  score: {best.score:.4f}")
        print(f"  inliers: {best.num_inliers} ({100.0 * best.inlier_ratio:.2f}%)")
        print(f"  bottom image ratio: {100.0 * best.bottom_ratio:.2f}%")
        print(f"  camera side ratio: {100.0 * best.camera_side_ratio:.2f}%")
        print(f"  height consistency: {best.height_consistency:.4f}")
        print(f"  spatial coverage: {best.coverage:.4f}")
        print(f"  median camera-plane height: {best.median_height_slam:.8g} SLAM units")
        print(f"  MAD camera-plane height: {best.mad_height_slam:.8g} SLAM units")
        print(f"  real camera height: {args.camera_height:.8g} m")
        print(f"  metric scale to apply to translations: {scale:.8g} m / SLAM unit")

        if args.viz_dir:
            viz_dir = Path(args.viz_dir).resolve()
            if args.viz_candidates:
                export_candidate_visualizations(viz_dir, points, cameras, candidates, args.camera_height)
            else:
                export_visualization(viz_dir, points, cameras, best)
            print(f"\nVisualization written to: {viz_dir}")
            if args.viz_candidates:
                print("  candidate_XX_scale_*/map_points_selected_plane.ply")
                print("  candidate_XX_scale_*/camera_centers.ply")
                print("  candidate_XX_scale_*/selected_plane_patch.ply")
                print("  candidate_XX_scale_*/selected_plane.txt")
            else:
                print("  map_points_selected_plane.ply  # green=inliers, gray=other map points")
                print("  camera_centers.ply")
                print("  selected_plane_patch.ply")
                print("  selected_plane.txt")

        if len(candidates) > 1:
            print("\nOther plane candidates")
            for i, cand in enumerate(candidates[1:], start=2):
                cand_scale = args.camera_height / cand.median_height_slam
                print(
                    f"  {i}: score={cand.score:.4f}, inliers={cand.num_inliers}, "
                    f"bottom={100.0 * cand.bottom_ratio:.1f}%, "
                    f"height={cand.median_height_slam:.6g}, scale={cand_scale:.6g}"
                )

        if args.per_keyframe:
            estimates = estimate_keyframe_scale_curve(csv_path, args)
            print_keyframe_scale_summary(estimates, args)
            if args.per_keyframe_output:
                out_path = Path(args.per_keyframe_output).resolve()
                write_keyframe_scale_csv(out_path, estimates)
                print(f"  per-keyframe CSV: {out_path}")

            if args.scale_summary_output:
                robust = robust_keyframe_scale_summary(estimates, args)
                summary_path = Path(args.scale_summary_output).resolve()
                summary_path.parent.mkdir(parents=True, exist_ok=True)
                with summary_path.open("w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow(["method", "scale", "mad", "num_used", "num_total", "range_low", "range_high", "source"])
                    writer.writerow([
                        "global_plane",
                        f"{scale:.9g}",
                        f"{best.mad_height_slam * scale / max(best.median_height_slam, 1e-12):.9g}",
                        best.num_inliers,
                        len(points),
                        "",
                        "",
                        "best_global_plane",
                    ])
                    if robust is not None:
                        writer.writerow([
                            "robust_keyframe_cluster",
                            f"{robust.scale:.9g}",
                            f"{robust.mad:.9g}",
                            robust.num_used,
                            robust.num_total,
                            f"{robust.lower:.9g}",
                            f"{robust.upper:.9g}",
                            robust.source,
                        ])
                print(f"  scale summary CSV: {summary_path}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
