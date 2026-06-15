#!/usr/bin/env python3
"""Draw side-by-side keyframe match overlays from calib_keyframe_matches.csv."""

from __future__ import annotations

import argparse
import csv
import hashlib
from collections import Counter, defaultdict
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path, help="results_agilex/<run_id> directory")
    parser.add_argument("--csv", type=Path, help="Match CSV. Defaults to <run-dir>/calib_keyframe_matches.csv")
    parser.add_argument("--camera1-dir", type=Path, help="Override camera 1 PNG directory")
    parser.add_argument("--camera2-dir", type=Path, help="Override camera 2 PNG directory")
    parser.add_argument("--stage", default="projection_after_optimized_pose", help="CSV stage to draw")
    parser.add_argument("--top-pairs", type=int, default=12, help="Number of keyframe pairs to render")
    parser.add_argument("--max-matches", type=int, default=120, help="Maximum matches drawn per pair")
    parser.add_argument("--frame-id-offset", type=int, default=0, help="Add this to mnFrameId before indexing sorted PNGs")
    parser.add_argument(
        "--no-frame-id-wrap",
        action="store_true",
        help="Fail when a frame id is outside the PNG range instead of using frame_id %% num_frames.",
    )
    parser.add_argument("--out-dir", type=Path, help="Output directory. Defaults to <run-dir>/match_visualizations")
    return parser.parse_args()


def read_manifest(path: Path) -> Dict[str, str]:
    values: Dict[str, str] = {}
    if not path.exists():
        return values
    for line in path.read_text(errors="replace").splitlines():
        if "=" not in line or line.startswith(" "):
            continue
        key, value = line.split("=", 1)
        values[key.strip()] = value.strip()
    return values


def resolve_container_path(path_text: str, repo_root: Path) -> Path:
    path = Path(path_text)
    if path.exists():
        return path

    replacements = {
        "/ws/src/orbcalib-master": repo_root,
        "/ws/src/Agilex_Recordings": repo_root.parent / "Agilex Recordings",
        "/ws/src/NMC3D": repo_root.parent / "NMC3D",
        "/ws/src/T7": Path("/media/civit/T7"),
    }
    for prefix, host_prefix in replacements.items():
        if path_text.startswith(prefix):
            candidate = host_prefix / path_text[len(prefix) :].lstrip("/")
            if candidate.exists():
                return candidate

    return path


def sorted_pngs(folder: Path) -> List[Path]:
    frames = sorted(folder.glob("*.png"), key=lambda p: int("".join(ch for ch in p.stem if ch.isdigit()) or 0))
    if not frames:
        raise FileNotFoundError(f"No PNG files found in {folder}")
    return frames


def read_frame_map(path: Path, repo_root: Path) -> Tuple[List[Tuple[float, Path]], List[Tuple[float, Path]]]:
    cam1: List[Tuple[float, Path]] = []
    cam2: List[Tuple[float, Path]] = []
    if not path.exists():
        return cam1, cam2

    with path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            cam1.append((float(row["camera1_ros_stamp"]), resolve_container_path(row["camera1_png"], repo_root)))
            cam2.append((float(row["camera2_ros_stamp"]), resolve_container_path(row["camera2_png"], repo_root)))

    cam1.sort(key=lambda item: item[0])
    cam2.sort(key=lambda item: item[0])
    return cam1, cam2


def row_float(row: Dict[str, str], key: str) -> float:
    return float(row[key])


def row_int(row: Dict[str, str], key: str) -> int:
    return int(float(row[key]))


def color_for_ids(*ids: str) -> Tuple[int, int, int]:
    digest = hashlib.sha1("|".join(ids).encode("utf-8")).digest()
    return int(digest[0]), int(digest[1]), int(digest[2])


def choose_rows(rows: List[Dict[str, str]], max_matches: int) -> List[Dict[str, str]]:
    if len(rows) <= max_matches:
        return rows
    step = len(rows) / max_matches
    return [rows[int(i * step)] for i in range(max_matches)]


def load_image(frames: List[Path], frame_id: int, offset: int, wrap_frame_id: bool) -> Tuple[Path, np.ndarray]:
    idx = frame_id + offset
    if idx < 0 or idx >= len(frames):
        if not wrap_frame_id:
            raise IndexError(f"frame id {frame_id} + offset {offset} outside [0, {len(frames) - 1}]")
        idx %= len(frames)
    path = frames[idx]
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    return path, image


def load_image_by_timestamp(frame_map: List[Tuple[float, Path]], timestamp: float) -> Tuple[Path, np.ndarray]:
    if not frame_map:
        raise ValueError("empty frame timestamp map")
    stamp, path = min(frame_map, key=lambda item: abs(item[0] - timestamp))
    image = cv2.imread(str(path), cv2.IMREAD_COLOR)
    if image is None:
        raise ValueError(f"Could not read image: {path}")
    return path, image


def draw_pair(
    rows: List[Dict[str, str]],
    frames1: List[Path],
    frames2: List[Path],
    out_path: Path,
    max_matches: int,
    frame_id_offset: int,
    wrap_frame_id: bool,
    frame_map1: Optional[List[Tuple[float, Path]]] = None,
    frame_map2: Optional[List[Tuple[float, Path]]] = None,
) -> None:
    first = rows[0]
    src_frame_id = row_int(first, "src_frame_id")
    dst_frame_id = row_int(first, "dst_frame_id")
    if frame_map1 and frame_map2 and first.get("src_timestamp") and first.get("dst_timestamp"):
        img1_path, img1 = load_image_by_timestamp(frame_map1, row_float(first, "src_timestamp"))
        img2_path, img2 = load_image_by_timestamp(frame_map2, row_float(first, "dst_timestamp"))
    else:
        img1_path, img1 = load_image(frames1, src_frame_id, frame_id_offset, wrap_frame_id)
        img2_path, img2 = load_image(frames2, dst_frame_id, frame_id_offset, wrap_frame_id)

    h = max(img1.shape[0], img2.shape[0])
    w1, w2 = img1.shape[1], img2.shape[1]
    canvas = np.zeros((h, w1 + w2, 3), dtype=np.uint8)
    canvas[: img1.shape[0], :w1] = img1
    canvas[: img2.shape[0], w1 : w1 + w2] = img2

    selected = choose_rows(rows, max_matches)
    for row in selected:
        p1 = (int(round(row_float(row, "src_u"))), int(round(row_float(row, "src_v"))))
        p2 = (w1 + int(round(row_float(row, "dst_u"))), int(round(row_float(row, "dst_v"))))
        color = color_for_ids(row["src_mp_id"], row["dst_mp_id"])
        cv2.circle(canvas, p1, 4, color, -1, lineType=cv2.LINE_AA)
        cv2.circle(canvas, p2, 4, color, -1, lineType=cv2.LINE_AA)
        cv2.line(canvas, p1, p2, color, 1, lineType=cv2.LINE_AA)

    label = (
        f"{first['stage']}  KF {first['src_kf_id']} -> {first['dst_kf_id']}  "
        f"matches {len(rows)} drawn {len(selected)}"
    )
    cv2.rectangle(canvas, (0, 0), (min(canvas.shape[1], 1200), 30), (0, 0, 0), -1)
    cv2.putText(canvas, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, img1_path.name, (8, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(canvas, img2_path.name, (w1 + 8, canvas.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out_path), canvas)


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    run_dir = args.run_dir.resolve()
    csv_path = args.csv or run_dir / "calib_keyframe_matches.csv"
    out_dir = args.out_dir or run_dir / "match_visualizations"

    manifest = read_manifest(run_dir / "manifest.txt")
    camera1_dir = args.camera1_dir or resolve_container_path(manifest.get("camera1_dir", ""), repo_root)
    camera2_dir = args.camera2_dir or resolve_container_path(manifest.get("camera2_dir", ""), repo_root)

    frames1 = sorted_pngs(camera1_dir)
    frames2 = sorted_pngs(camera2_dir)
    frame_map1, frame_map2 = read_frame_map(run_dir / "frame_pairs.csv", repo_root)
    if frame_map1 and frame_map2:
        print(f"Using timestamp image map: {run_dir / 'frame_pairs.csv'}")

    grouped: Dict[Tuple[str, str], List[Dict[str, str]]] = defaultdict(list)
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("stage") != args.stage:
                continue
            if "src_frame_id" not in row or "dst_frame_id" not in row:
                raise ValueError("CSV is missing src_frame_id/dst_frame_id. Rerun calibration with the latest build.")
            grouped[(row["src_kf_id"], row["dst_kf_id"])].append(row)

    top = Counter({pair: len(rows) for pair, rows in grouped.items()}).most_common(args.top_pairs)
    if not top:
        raise ValueError(f"No rows found for stage '{args.stage}' in {csv_path}")

    for rank, (pair, count) in enumerate(top, start=1):
        rows = grouped[pair]
        out_path = out_dir / f"{rank:03d}_{args.stage}_kf{pair[0]}_kf{pair[1]}_{count}_matches.png"
        draw_pair(
            rows,
            frames1,
            frames2,
            out_path,
            args.max_matches,
            args.frame_id_offset,
            not args.no_frame_id_wrap,
            frame_map1,
            frame_map2,
        )
        print(out_path)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
