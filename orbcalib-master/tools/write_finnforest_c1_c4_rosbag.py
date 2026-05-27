#!/usr/bin/env python3
"""
Write a ROS1 bag directly from FinnForest S01 C1/C4 debayered frames.

This avoids the live ROS publisher bottleneck by reading PNGs and raw
timestamps from disk, then writing sensor_msgs/Image messages straight into
the bag file.
"""

import argparse
import bisect
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

import cv2
import rosbag
import rospy
from sensor_msgs.msg import Image


DEFAULT_ROOT = Path(
    "/data/Finnforest/RAWData_40Hz/"
    "S01_40Hz_summer_seq1_shortLoop/Summer_seq1_day_shortLoop"
)
RAW_ROOT = "raw_C1_C4"
DEBAYERED_ROOT = "debayered_C1_C4"
C1_DIR = "C1_cam22573022"
C4_DIR = "C4_cam22555668"
FRAME_RE = re.compile(r"^(?P<frame>\d+)_cam(?P<serial>\d+)_ts(?P<ts>\d+)\.raw$")


@dataclass(frozen=True)
class RawFrame:
    frame_id: int
    timestamp_us: int


@dataclass(frozen=True)
class FramePair:
    frame_id: int
    c1: RawFrame
    c4: RawFrame
    pair_time_us: int
    c1_png: Path
    c4_png: Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--root",
        default=str(DEFAULT_ROOT),
        help="FinnForest S01 sequence root containing raw_C1_C4 and debayered_C1_C4.",
    )
    parser.add_argument(
        "--output",
        required=True,
        help="Output ROS1 bag path.",
    )
    parser.add_argument(
        "--hz",
        type=float,
        default=40.0,
        help="Target dataset rate. Common values: 40, 13.3333333333, 8.",
    )
    parser.add_argument(
        "--topic-c1",
        default="/cam_c1/image",
        help="ROS1 topic for camera C1.",
    )
    parser.add_argument(
        "--topic-c4",
        default="/cam_c4/image",
        help="ROS1 topic for camera C4.",
    )
    parser.add_argument(
        "--frame-id-c1",
        default="camera_c1",
        help="frame_id for C1 image messages.",
    )
    parser.add_argument(
        "--frame-id-c4",
        default="camera_c4",
        help="frame_id for C4 image messages.",
    )
    parser.add_argument(
        "--color-order",
        choices=("rgb8", "bgr8"),
        default="rgb8",
        help="Image encoding to store in the bag. Match this with Camera.RGB in orbcalib.",
    )
    parser.add_argument(
        "--start-frame",
        type=int,
        default=None,
        help="Optional inclusive starting frame id.",
    )
    parser.add_argument(
        "--end-frame",
        type=int,
        default=None,
        help="Optional inclusive ending frame id.",
    )
    return parser.parse_args()


def scan_camera_folder(folder: Path) -> Dict[int, RawFrame]:
    frames: Dict[int, RawFrame] = {}
    for path in sorted(folder.glob("*.raw")):
        match = FRAME_RE.match(path.name)
        if not match:
            continue
        frame_id = int(match.group("frame"))
        timestamp_us = int(match.group("ts"))
        frames[frame_id] = RawFrame(frame_id=frame_id, timestamp_us=timestamp_us)
    if not frames:
        raise RuntimeError(f"No raw frames found in {folder}")
    return frames


def png_for_frame(folder: Path, frame_id: int) -> Path:
    candidates = [
        folder / f"{frame_id:07d}.png",
        folder / f"{frame_id:06d}.png",
        folder / f"{frame_id}.png",
    ]
    for png_path in candidates:
        if png_path.exists():
            return png_path
    raise RuntimeError(f"Missing debayered PNG for frame {frame_id}: tried {candidates}")


def build_pairs(root: Path, start_frame: int, end_frame: int) -> List[FramePair]:
    c1_frames = scan_camera_folder(root / RAW_ROOT / C1_DIR)
    c4_frames = scan_camera_folder(root / RAW_ROOT / C4_DIR)
    c1_png_dir = root / DEBAYERED_ROOT / C1_DIR
    c4_png_dir = root / DEBAYERED_ROOT / C4_DIR

    common_ids = sorted(set(c1_frames) & set(c4_frames))
    if start_frame is not None:
        common_ids = [frame_id for frame_id in common_ids if frame_id >= start_frame]
    if end_frame is not None:
        common_ids = [frame_id for frame_id in common_ids if frame_id <= end_frame]

    if not common_ids:
        raise RuntimeError("No common C1/C4 frames remain after filtering.")

    pairs: List[FramePair] = []
    for frame_id in common_ids:
        c1 = c1_frames[frame_id]
        c4 = c4_frames[frame_id]
        pair_time_us = (c1.timestamp_us + c4.timestamp_us) // 2
        pairs.append(
            FramePair(
                frame_id=frame_id,
                c1=c1,
                c4=c4,
                pair_time_us=pair_time_us,
                c1_png=png_for_frame(c1_png_dir, frame_id),
                c4_png=png_for_frame(c4_png_dir, frame_id),
            )
        )
    return pairs


def select_by_rate(pairs: Sequence[FramePair], target_hz: float) -> List[FramePair]:
    if target_hz <= 0:
        raise ValueError("Target hz must be > 0.")
    if len(pairs) < 2:
        return list(pairs)
    if target_hz >= 39.0:
        return list(pairs)

    step_us = int(round(1e6 / target_hz))
    selected: List[FramePair] = [pairs[0]]
    next_target = pairs[0].pair_time_us + step_us

    times = [pair.pair_time_us for pair in pairs]
    last_selected_index = 0
    while next_target <= times[-1]:
        idx = bisect.bisect_left(times, next_target, lo=last_selected_index + 1)
        if idx >= len(pairs):
            break

        candidates = [idx]
        if idx - 1 > last_selected_index:
            candidates.append(idx - 1)
        best_idx = min(candidates, key=lambda i: abs(times[i] - next_target))

        if best_idx <= last_selected_index:
            break
        selected.append(pairs[best_idx])
        last_selected_index = best_idx
        next_target += step_us

    return selected


def read_png_image(png_path: Path, encoding: str):
    image_bgr = cv2.imread(str(png_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError(f"Failed to read debayered PNG: {png_path}")
    if encoding == "rgb8":
        return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return image_bgr


def make_ros_image(image, stamp: rospy.Time, frame_id: str, encoding: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = image.shape[1] * image.shape[2]
    msg.data = image.tobytes()
    return msg


def to_ros_time(base_sec: float, timestamp_us: int, first_timestamp_us: int) -> rospy.Time:
    return rospy.Time.from_sec(base_sec + (timestamp_us - first_timestamp_us) / 1e6)


def write_bag(
    bag_path: Path,
    selected_pairs: Sequence[FramePair],
    topic_c1: str,
    topic_c4: str,
    frame_id_c1: str,
    frame_id_c4: str,
    encoding: str,
) -> None:
    bag_path.parent.mkdir(parents=True, exist_ok=True)
    first_ts_us = min(selected_pairs[0].c1.timestamp_us, selected_pairs[0].c4.timestamp_us)
    base_sec = 1.0

    with rosbag.Bag(str(bag_path), "w") as bag:
        for index, pair in enumerate(selected_pairs, start=1):
            c1_image = read_png_image(pair.c1_png, encoding)
            c4_image = read_png_image(pair.c4_png, encoding)

            c1_stamp = to_ros_time(base_sec, pair.c1.timestamp_us, first_ts_us)
            c4_stamp = to_ros_time(base_sec, pair.c4.timestamp_us, first_ts_us)

            c1_msg = make_ros_image(c1_image, c1_stamp, frame_id_c1, encoding)
            c4_msg = make_ros_image(c4_image, c4_stamp, frame_id_c4, encoding)

            if c1_stamp <= c4_stamp:
                bag.write(topic_c1, c1_msg, t=c1_stamp)
                bag.write(topic_c4, c4_msg, t=c4_stamp)
            else:
                bag.write(topic_c4, c4_msg, t=c4_stamp)
                bag.write(topic_c1, c1_msg, t=c1_stamp)

            if index == 1 or index % 200 == 0 or index == len(selected_pairs):
                print(
                    f"Wrote {index}/{len(selected_pairs)} frame pairs "
                    f"(last frame id {pair.frame_id:06d})"
                )


def main() -> None:
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    output = Path(args.output).expanduser().resolve()

    pairs = build_pairs(root, args.start_frame, args.end_frame)
    selected = select_by_rate(pairs, args.hz)

    print(f"Sequence root: {root}")
    print(f"Output bag: {output}")
    print(f"Raw common frame pairs available: {len(pairs)}")
    print(f"Selected frame pairs at {args.hz:.6f} Hz: {len(selected)}")
    print(f"Topics: {args.topic_c1} and {args.topic_c4}")
    print(f"Encoding: {args.color_order}")

    write_bag(
        bag_path=output,
        selected_pairs=selected,
        topic_c1=args.topic_c1,
        topic_c4=args.topic_c4,
        frame_id_c1=args.frame_id_c1,
        frame_id_c4=args.frame_id_c4,
        encoding=args.color_order,
    )

    print("Done.")


if __name__ == "__main__":
    main()
