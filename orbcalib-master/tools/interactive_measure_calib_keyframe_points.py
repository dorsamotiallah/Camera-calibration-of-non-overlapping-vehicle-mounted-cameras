#!/usr/bin/env python3
"""Interactively compare two-point 3D distances in each keyframe map."""

from __future__ import annotations

import argparse
import csv
import hashlib
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np

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


@dataclass
class PickPoint:
    row: Dict[str, str]
    side: str
    point_id: str
    xy: Tuple[int, int]
    xyz: np.ndarray


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path, help="results_agilex/<run_id> directory")
    parser.add_argument("--csv", type=Path, help="Match CSV. Defaults to <run-dir>/calib_keyframe_matches.csv")
    parser.add_argument("--camera1-dir", type=Path, help="Override camera 1 PNG directory")
    parser.add_argument("--camera2-dir", type=Path, help="Override camera 2 PNG directory")
    parser.add_argument("--stage", default="projection_after_optimized_pose", help="CSV stage to inspect")
    parser.add_argument("--pair-rank", type=int, default=1, help="Open the Nth most populated keyframe pair")
    parser.add_argument("--src-kf-id", help="Open a specific source keyframe id")
    parser.add_argument("--dst-kf-id", help="Open a specific destination keyframe id")
    parser.add_argument("--max-matches", type=int, default=250, help="Maximum match endpoints drawn and pickable")
    parser.add_argument("--pick-radius", type=float, default=18.0, help="Maximum click-to-point snap radius in pixels")
    parser.add_argument("--scale-src", type=float, help="Optional camera1 scale in meters / SLAM unit")
    parser.add_argument("--scale-dst", type=float, help="Optional camera2 scale in meters / SLAM unit")
    parser.add_argument("--frame-id-offset", type=int, default=0, help="Add this to mnFrameId before indexing sorted PNGs")
    parser.add_argument(
        "--no-frame-id-wrap",
        action="store_true",
        help="Fail when a frame id is outside the PNG range instead of using frame_id %% num_frames.",
    )
    return parser.parse_args()


def color_for_ids(*ids: str) -> Tuple[int, int, int]:
    digest = hashlib.sha1("|".join(ids).encode("utf-8")).digest()
    return int(digest[0]), int(digest[1]), int(digest[2])


def choose_rows(rows: List[Dict[str, str]], max_matches: int) -> List[Dict[str, str]]:
    if len(rows) <= max_matches:
        return rows
    step = len(rows) / max_matches
    return [rows[int(i * step)] for i in range(max_matches)]


def xyz_from_row(row: Dict[str, str], side: str) -> np.ndarray:
    prefix = "src" if side == "src" else "dst"
    return np.array(
        [
            row_float(row, f"{prefix}_mp_x"),
            row_float(row, f"{prefix}_mp_y"),
            row_float(row, f"{prefix}_mp_z"),
        ],
        dtype=float,
    )


def point_from_row(row: Dict[str, str], side: str, offset_x: int) -> PickPoint:
    prefix = "src" if side == "src" else "dst"
    x = int(round(row_float(row, f"{prefix}_u"))) + offset_x
    y = int(round(row_float(row, f"{prefix}_v")))
    return PickPoint(
        row=row,
        side=side,
        point_id=row[f"{prefix}_mp_id"],
        xy=(x, y),
        xyz=xyz_from_row(row, side),
    )


def load_pair_images(
    rows: List[Dict[str, str]],
    frames1: List[Path],
    frames2: List[Path],
    frame_id_offset: int,
    wrap_frame_id: bool,
    frame_map1: Optional[List[Tuple[float, Path]]] = None,
    frame_map2: Optional[List[Tuple[float, Path]]] = None,
) -> Tuple[Path, np.ndarray, Path, np.ndarray]:
    first = rows[0]
    if frame_map1 and frame_map2 and first.get("src_timestamp") and first.get("dst_timestamp"):
        img1_path, img1 = load_image_by_timestamp(frame_map1, row_float(first, "src_timestamp"))
        img2_path, img2 = load_image_by_timestamp(frame_map2, row_float(first, "dst_timestamp"))
    else:
        img1_path, img1 = load_image(frames1, row_int(first, "src_frame_id"), frame_id_offset, wrap_frame_id)
        img2_path, img2 = load_image(frames2, row_int(first, "dst_frame_id"), frame_id_offset, wrap_frame_id)
    return img1_path, img1, img2_path, img2


def build_canvas(img1: np.ndarray, img2: np.ndarray) -> np.ndarray:
    h = max(img1.shape[0], img2.shape[0])
    w1, w2 = img1.shape[1], img2.shape[1]
    canvas = np.zeros((h, w1 + w2, 3), dtype=np.uint8)
    canvas[: img1.shape[0], :w1] = img1
    canvas[: img2.shape[0], w1 : w1 + w2] = img2
    return canvas


def draw_text_box(canvas: np.ndarray, lines: List[str]) -> None:
    if not lines:
        return
    line_h = 22
    width = min(canvas.shape[1], 1650)
    height = 8 + line_h * len(lines)
    cv2.rectangle(canvas, (0, 0), (width, height), (0, 0, 0), -1)
    for i, line in enumerate(lines):
        cv2.putText(
            canvas,
            line,
            (8, 22 + i * line_h),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.58,
            (255, 255, 255),
            1,
            cv2.LINE_AA,
        )


def render(
    base: np.ndarray,
    points: List[PickPoint],
    selected_by_side: Dict[str, List[PickPoint]],
    status_lines: List[str],
) -> np.ndarray:
    canvas = base.copy()
    for point in points:
        color = color_for_ids(point.row["src_mp_id"], point.row["dst_mp_id"])
        cv2.circle(canvas, point.xy, 4, color, -1, lineType=cv2.LINE_AA)

    side_styles = {
        "src": ((0, 255, 255), "L"),
        "dst": ((255, 180, 0), "R"),
    }
    for side, selected in selected_by_side.items():
        color, label_prefix = side_styles[side]
        if len(selected) == 2:
            cv2.line(canvas, selected[0].xy, selected[1].xy, color, 2, lineType=cv2.LINE_AA)

        for idx, point in enumerate(selected, start=1):
            cv2.circle(canvas, point.xy, 11, color, 2, lineType=cv2.LINE_AA)
            cv2.putText(
                canvas,
                f"{label_prefix}{idx}",
                (point.xy[0] + 12, point.xy[1] - 8),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                color,
                2,
                cv2.LINE_AA,
            )

    draw_text_box(canvas, status_lines)
    return canvas


def nearest_point(points: List[PickPoint], x: int, y: int, radius: float) -> Optional[PickPoint]:
    best: Optional[PickPoint] = None
    best_dist = radius
    click = np.array([x, y], dtype=float)
    for point in points:
        dist = float(np.linalg.norm(np.array(point.xy, dtype=float) - click))
        if dist <= best_dist:
            best = point
            best_dist = dist
    return best


def side_distance(selected: List[PickPoint]) -> Optional[float]:
    if len(selected) != 2:
        return None
    return float(np.linalg.norm(selected[1].xyz - selected[0].xyz))


def fmt_xyz(xyz: np.ndarray) -> str:
    return f"[{xyz[0]:.6g}, {xyz[1]:.6g}, {xyz[2]:.6g}]"


def side_lines(side: str, selected: List[PickPoint], scale: Optional[float]) -> List[str]:
    label = "source distance" if side == "src" else "dst distance"
    if len(selected) == 0:
        return [f"{label}: pick 2 points"]
    if len(selected) == 1:
        return [f"{label}: pick 1/2 selected"]

    dist = side_distance(selected)
    assert dist is not None
    line = f"{label}: {dist:.9g} SLAM units"
    if scale is not None:
        line += f" = {dist * scale:.9g} m"
    else:
        line += " (no scale provided)"
    return [line]


def comparison_lines(
    selected_by_side: Dict[str, List[PickPoint]],
    scale_src: Optional[float],
    scale_dst: Optional[float],
) -> List[str]:
    lines: List[str] = []
    lines.extend(side_lines("src", selected_by_side["src"], scale_src))
    lines.extend(side_lines("dst", selected_by_side["dst"], scale_dst))

    src_dist = side_distance(selected_by_side["src"])
    dst_dist = side_distance(selected_by_side["dst"])
    if src_dist is not None and dst_dist is not None:
        ratio = dst_dist / src_dist if src_dist != 0.0 else float("inf")
        lines.append(f"SLAM-unit comparison: dst - src = {dst_dist - src_dist:.9g}, dst/src = {ratio:.9g}")
    if src_dist is not None and dst_dist is not None and scale_src is not None and scale_dst is not None:
        src_m = src_dist * scale_src
        dst_m = dst_dist * scale_dst
        ratio_m = dst_m / src_m if src_m != 0.0 else float("inf")
        lines.append(f"metric comparison: dst - src = {dst_m - src_m:.9g} m, dst/src = {ratio_m:.9g}")

    return lines


def grouped_rows(csv_path: Path, stage: str) -> Dict[Tuple[str, str], List[Dict[str, str]]]:
    grouped: Dict[Tuple[str, str], List[Dict[str, str]]] = defaultdict(list)
    with csv_path.open(newline="") as handle:
        for row in csv.DictReader(handle):
            if row.get("stage") == stage:
                grouped[(row["src_kf_id"], row["dst_kf_id"])].append(row)
    if not grouped:
        raise ValueError(f"No rows found for stage '{stage}' in {csv_path}")
    return grouped


def choose_pair(
    grouped: Dict[Tuple[str, str], List[Dict[str, str]]],
    pair_rank: int,
    src_kf_id: Optional[str],
    dst_kf_id: Optional[str],
) -> Tuple[Tuple[str, str], List[Dict[str, str]], List[Tuple[Tuple[str, str], int]]]:
    top = Counter({pair: len(rows) for pair, rows in grouped.items()}).most_common()
    if src_kf_id is not None or dst_kf_id is not None:
        if src_kf_id is None or dst_kf_id is None:
            raise ValueError("--src-kf-id and --dst-kf-id must be used together")
        pair = (src_kf_id, dst_kf_id)
        if pair not in grouped:
            raise ValueError(f"No rows for keyframe pair {pair[0]} -> {pair[1]}")
        return pair, grouped[pair], top

    if pair_rank < 1 or pair_rank > len(top):
        raise ValueError(f"--pair-rank must be in [1, {len(top)}]")
    pair = top[pair_rank - 1][0]
    return pair, grouped[pair], top


def main() -> int:
    args = parse_args()
    repo_root = Path(__file__).resolve().parents[1]
    run_dir = args.run_dir.resolve()
    csv_path = args.csv or run_dir / "calib_keyframe_matches.csv"

    manifest = read_manifest(run_dir / "manifest.txt")
    camera1_dir = args.camera1_dir or resolve_container_path(manifest.get("camera1_dir", ""), repo_root)
    camera2_dir = args.camera2_dir or resolve_container_path(manifest.get("camera2_dir", ""), repo_root)

    frames1 = sorted_pngs(camera1_dir)
    frames2 = sorted_pngs(camera2_dir)
    frame_map1, frame_map2 = read_frame_map(run_dir / "frame_pairs.csv", repo_root)

    groups = grouped_rows(csv_path, args.stage)
    pair, rows, top = choose_pair(groups, args.pair_rank, args.src_kf_id, args.dst_kf_id)
    chosen = choose_rows(rows, args.max_matches)

    img1_path, img1, img2_path, img2 = load_pair_images(
        rows,
        frames1,
        frames2,
        args.frame_id_offset,
        not args.no_frame_id_wrap,
        frame_map1,
        frame_map2,
    )
    base = build_canvas(img1, img2)
    w1 = img1.shape[1]
    points: List[PickPoint] = []
    for row in chosen:
        points.append(point_from_row(row, "src", 0))
        points.append(point_from_row(row, "dst", w1))

    print(f"Opened {args.stage} KF {pair[0]} -> {pair[1]} with {len(rows)} matches; {len(chosen)} drawn.")
    print(f"camera1 image: {img1_path}")
    print(f"camera2 image: {img2_path}")
    print("Controls: left-click two left points and two right points, r reset all, l reset left, d reset right, s save, q/Esc quit.")
    print("Top pairs:")
    for rank, (top_pair, count) in enumerate(top[:10], start=1):
        marker = "*" if top_pair == pair else " "
        print(f"  {marker} {rank:2d}: KF {top_pair[0]} -> {top_pair[1]}  {count} matches")

    selected_by_side: Dict[str, List[PickPoint]] = {"src": [], "dst": []}
    last_lines: List[str] = comparison_lines(selected_by_side, args.scale_src, args.scale_dst)
    window = "calib keyframe distance picker"

    def status() -> List[str]:
        lines = [
            f"{args.stage}  KF {pair[0]} -> {pair[1]}  matches {len(rows)} drawn {len(chosen)}",
            "left image: pick 2 src points   right image: pick 2 dst points   r/l/d reset   s save   q/Esc quit",
        ]
        lines.extend(last_lines[-4:])
        return lines

    def redraw() -> None:
        cv2.imshow(window, render(base, points, selected_by_side, status()))

    def on_mouse(event: int, x: int, y: int, _flags: int, _userdata: object) -> None:
        nonlocal last_lines
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        point = nearest_point(points, x, y, args.pick_radius)
        if point is None:
            print(f"No drawn point within {args.pick_radius:g} px of click ({x}, {y})")
            return
        selected = selected_by_side[point.side]
        if len(selected) >= 2:
            selected.clear()
        selected.append(point)
        side_label = "left/src" if point.side == "src" else "right/dst"
        print(f"picked {side_label} {len(selected)}: mp {point.point_id} at image {point.xy}, xyz={point.xyz}")
        last_lines = comparison_lines(selected_by_side, args.scale_src, args.scale_dst)
        for line in last_lines:
            print(line)
        redraw()

    cv2.namedWindow(window, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(window, on_mouse)
    redraw()

    while True:
        key = cv2.waitKey(30) & 0xFF
        if key in (ord("q"), 27):
            break
        if key == ord("r"):
            selected_by_side["src"].clear()
            selected_by_side["dst"].clear()
            last_lines = comparison_lines(selected_by_side, args.scale_src, args.scale_dst)
            redraw()
        if key == ord("l"):
            selected_by_side["src"].clear()
            last_lines = comparison_lines(selected_by_side, args.scale_src, args.scale_dst)
            redraw()
        if key == ord("d"):
            selected_by_side["dst"].clear()
            last_lines = comparison_lines(selected_by_side, args.scale_src, args.scale_dst)
            redraw()
        if key == ord("s"):
            out_path = run_dir / f"distance_picker_kf{pair[0]}_kf{pair[1]}.png"
            cv2.imwrite(str(out_path), render(base, points, selected_by_side, status()))
            print(f"saved {out_path}")

    cv2.destroyWindow(window)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
