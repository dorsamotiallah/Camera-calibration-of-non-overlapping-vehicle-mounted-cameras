#!/usr/bin/env python3
"""
Publish FinnForest S01 C1/C4 debayered frames as ROS1 Image topics.

This script is intentionally standalone and does not modify orbcalib's
existing Gazebo/MCAP workflow. It reads FinnForest raw timestamps plus the
already-debayered PNG folders and publishes ROS1 sensor_msgs/Image messages.
"""

import argparse
import bisect
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Sequence

import cv2
import numpy as np
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
        "--hz",
        type=float,
        default=13.3333333333,
        help="Target dataset rate. Common values: 40, 13.3333333333, 8.",
    )
    parser.add_argument(
        "--playback-rate",
        type=float,
        default=1.0,
        help="Wall-clock speed multiplier for replay.",
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
        help="Image encoding to publish. Match this with Camera.RGB in orbcalib.",
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
    parser.add_argument(
        "--loop",
        action="store_true",
        help="Replay the selected sequence continuously.",
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

    if selected[-1].frame_id != pairs[-1].frame_id and target_hz >= 39.0:
        selected.append(pairs[-1])

    return selected


def read_png_image(png_path: Path, encoding: str):
    image_bgr = cv2.imread(str(png_path), cv2.IMREAD_COLOR)
    if image_bgr is None:
        raise RuntimeError(f"Failed to read debayered PNG: {png_path}")
    if encoding == "rgb8":
        return cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    return image_bgr


def make_ros_image(image: np.ndarray, stamp: rospy.Time, frame_id: str, encoding: str) -> Image:
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


def sleep_until(target_monotonic: float) -> None:
    while True:
        remaining = target_monotonic - time.monotonic()
        if remaining <= 0:
            return
        time.sleep(min(remaining, 0.01))


def replay_sequence(
    selected_pairs: Sequence[FramePair],
    topic_c1: str,
    topic_c4: str,
    frame_id_c1: str,
    frame_id_c4: str,
    encoding: str,
    playback_rate: float,
) -> None:
    pub_c1 = rospy.Publisher(topic_c1, Image, queue_size=2)
    pub_c4 = rospy.Publisher(topic_c4, Image, queue_size=2)
    time.sleep(1.0)

    first_pair_time = selected_pairs[0].pair_time_us
    first_c1_time = selected_pairs[0].c1.timestamp_us
    first_c4_time = selected_pairs[0].c4.timestamp_us
    wall_start = time.monotonic()
    ros_base = rospy.Time.now()

    for index, pair in enumerate(selected_pairs, start=1):
        if rospy.is_shutdown():
            return

        target_elapsed = (pair.pair_time_us - first_pair_time) / 1e6 / playback_rate
        sleep_until(wall_start + target_elapsed)

        c1_image = read_png_image(pair.c1_png, encoding)
        c4_image = read_png_image(pair.c4_png, encoding)

        c1_stamp = ros_base + rospy.Duration.from_sec(
            (pair.c1.timestamp_us - first_c1_time) / 1e6 / playback_rate
        )
        c4_stamp = ros_base + rospy.Duration.from_sec(
            (pair.c4.timestamp_us - first_c4_time) / 1e6 / playback_rate
        )

        pub_c1.publish(make_ros_image(c1_image, c1_stamp, frame_id_c1, encoding))
        pub_c4.publish(make_ros_image(c4_image, c4_stamp, frame_id_c4, encoding))

        if index == 1 or index % 200 == 0 or index == len(selected_pairs):
            rospy.loginfo(
                "Published %d/%d frame pairs (last frame id %06d)",
                index,
                len(selected_pairs),
                pair.frame_id,
            )


def main() -> None:
    args = parse_args()
    root = Path(args.root).expanduser().resolve()
    pairs = build_pairs(root, args.start_frame, args.end_frame)
    selected = select_by_rate(pairs, args.hz)

    rospy.init_node("finnforest_c1_c4_ros1_publisher", anonymous=True)

    rospy.loginfo("Sequence root: %s", root)
    rospy.loginfo("Raw common frame pairs available: %d", len(pairs))
    rospy.loginfo("Selected frame pairs at %.6f Hz: %d", args.hz, len(selected))
    rospy.loginfo("Publishing topics: %s and %s", args.topic_c1, args.topic_c4)
    rospy.loginfo("Playback rate multiplier: %.3f", args.playback_rate)
    rospy.loginfo("Encoding: %s", args.color_order)

    while not rospy.is_shutdown():
        replay_sequence(
            selected_pairs=selected,
            topic_c1=args.topic_c1,
            topic_c4=args.topic_c4,
            frame_id_c1=args.frame_id_c1,
            frame_id_c4=args.frame_id_c4,
            encoding=args.color_order,
            playback_rate=args.playback_rate,
        )
        if not args.loop:
            break
        rospy.loginfo("Looping replay from the start.")


if __name__ == "__main__":
    main()
