#!/usr/bin/env python3
"""
ACK-driven ROS1 player for two timestamped PNG image folders.

This mirrors the FinnForest controlled player, but the source is two folders of
PNG images instead of a ROS bag. It publishes one image pair, waits for the
orbcalib processed-frame ACKs, then publishes the next pair.
"""

import argparse
import bisect
import re
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, Iterator, List, Sequence, Tuple

import cv2
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


StampKey = Tuple[int, int]
PNG_RE = re.compile(r"(?P<stamp>\d+)")


@dataclass(frozen=True)
class PngFrame:
    path: Path
    stamp_ns: int


@dataclass(frozen=True)
class PngPair:
    index: int
    frame1: PngFrame
    frame2: PngFrame
    stamp_ns: int


def stamp_key(stamp: rospy.Time) -> StampKey:
    return stamp.secs, stamp.nsecs


def log(message: str) -> None:
    print(message, flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--camera1-dir", required=True, help="PNG folder for camera 1.")
    parser.add_argument("--camera2-dir", required=True, help="PNG folder for camera 2.")
    parser.add_argument("--topic1", default="/cam_front/image", help="ROS1 output topic for camera 1.")
    parser.add_argument("--topic2", default="/cam_back/image", help="ROS1 output topic for camera 2.")
    parser.add_argument("--frame-id1", default="camera1", help="frame_id for camera 1.")
    parser.add_argument("--frame-id2", default="camera2", help="frame_id for camera 2.")
    parser.add_argument("--ack1", default="/orbcalib/camera1/processed", help="ACK topic for camera 1.")
    parser.add_argument("--ack2", default="/orbcalib/camera2/processed", help="ACK topic for camera 2.")
    parser.add_argument("--encoding", choices=("rgb8", "bgr8", "mono8"), default="rgb8")
    parser.add_argument("--pairing", choices=("ordered", "nearest"), default="nearest")
    parser.add_argument("--max-skew-sec", type=float, default=0.05, help="Max timestamp skew for nearest pairing.")
    parser.add_argument("--hz", type=float, default=0.0, help="Optional downsample rate. 0 publishes all pairs.")
    parser.add_argument("--playback-rate", type=float, default=0.0, help="0 means ACK-paced as fast as SLAM allows.")
    parser.add_argument("--max-in-flight", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--start-index", type=int, default=1, help="1-based inclusive pair index.")
    parser.add_argument("--max-pairs", type=int, default=0, help="0 means all pairs.")
    parser.add_argument("--wait-for-subscribers", action="store_true")
    parser.add_argument("--subscriber-timeout-sec", type=float, default=60.0)
    parser.add_argument("--queue-size", type=int, default=10)
    parser.add_argument("--log-every", type=int, default=100)
    return parser.parse_args()


class AckTracker:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._acked1 = set()
        self._acked2 = set()
        self._pending: Deque[Tuple[StampKey, StampKey]] = deque()

    def add_pending(self, stamp1: rospy.Time, stamp2: rospy.Time) -> None:
        with self._condition:
            self._pending.append((stamp_key(stamp1), stamp_key(stamp2)))
            self._condition.notify_all()

    def ack1(self, msg: Header) -> None:
        with self._condition:
            self._acked1.add(stamp_key(msg.stamp))
            self._prune_locked()
            self._condition.notify_all()

    def ack2(self, msg: Header) -> None:
        with self._condition:
            self._acked2.add(stamp_key(msg.stamp))
            self._prune_locked()
            self._condition.notify_all()

    def pending_count(self) -> int:
        with self._condition:
            self._prune_locked()
            return len(self._pending)

    def wait_for_capacity(self, max_in_flight: int, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while len(self._pending) >= max_in_flight and not rospy.is_shutdown():
                self._prune_locked()
                if len(self._pending) < max_in_flight:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Timed out waiting for ACK capacity. Pending pairs: {len(self._pending)}")
                self._condition.wait(min(remaining, 0.2))

    def wait_for_all(self, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while self._pending and not rospy.is_shutdown():
                self._prune_locked()
                if not self._pending:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Timed out waiting for final ACKs. Pending pairs: {len(self._pending)}")
                self._condition.wait(min(remaining, 0.2))

    def _prune_locked(self) -> None:
        while self._pending:
            stamp1, stamp2 = self._pending[0]
            if stamp1 in self._acked1 and stamp2 in self._acked2:
                self._pending.popleft()
                self._acked1.discard(stamp1)
                self._acked2.discard(stamp2)
            else:
                break


def wait_for_subscribers(pub1: rospy.Publisher, pub2: rospy.Publisher, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if pub1.get_num_connections() > 0 and pub2.get_num_connections() > 0:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for image subscribers.")
        rate.sleep()


def parse_stamp(path: Path) -> int:
    match = PNG_RE.search(path.stem)
    if not match:
        raise ValueError(f"Could not parse timestamp from PNG filename: {path.name}")
    return int(match.group("stamp"))


def scan_png_folder(folder: Path) -> List[PngFrame]:
    frames = [PngFrame(path=p, stamp_ns=parse_stamp(p)) for p in folder.glob("*.png")]
    frames.sort(key=lambda frame: frame.stamp_ns)
    if not frames:
        raise RuntimeError(f"No PNG images found in {folder}")
    return frames


def pair_ordered(frames1: Sequence[PngFrame], frames2: Sequence[PngFrame]) -> List[PngPair]:
    count = min(len(frames1), len(frames2))
    return [
        PngPair(index=i + 1, frame1=frames1[i], frame2=frames2[i], stamp_ns=(frames1[i].stamp_ns + frames2[i].stamp_ns) // 2)
        for i in range(count)
    ]


def pair_nearest(frames1: Sequence[PngFrame], frames2: Sequence[PngFrame], max_skew_ns: int) -> List[PngPair]:
    stamps2 = [frame.stamp_ns for frame in frames2]
    pairs: List[PngPair] = []
    used2 = set()
    for frame1 in frames1:
        idx = bisect.bisect_left(stamps2, frame1.stamp_ns)
        candidates = []
        if idx < len(frames2):
            candidates.append(idx)
        if idx > 0:
            candidates.append(idx - 1)
        candidates = [i for i in candidates if i not in used2]
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(frames2[i].stamp_ns - frame1.stamp_ns))
        if abs(frames2[best].stamp_ns - frame1.stamp_ns) <= max_skew_ns:
            used2.add(best)
            pairs.append(
                PngPair(
                    index=len(pairs) + 1,
                    frame1=frame1,
                    frame2=frames2[best],
                    stamp_ns=(frame1.stamp_ns + frames2[best].stamp_ns) // 2,
                )
            )
    return pairs


def downsample_pairs(pairs: Sequence[PngPair], hz: float) -> List[PngPair]:
    if hz <= 0.0 or len(pairs) < 2:
        return list(pairs)
    step_ns = int(round(1e9 / hz))
    selected = [pairs[0]]
    next_target = pairs[0].stamp_ns + step_ns
    last_index = 0
    stamps = [pair.stamp_ns for pair in pairs]
    while next_target <= stamps[-1]:
        idx = bisect.bisect_left(stamps, next_target, lo=last_index + 1)
        if idx >= len(pairs):
            break
        candidates = [idx]
        if idx - 1 > last_index:
            candidates.append(idx - 1)
        best = min(candidates, key=lambda i: abs(stamps[i] - next_target))
        selected.append(pairs[best])
        last_index = best
        next_target += step_ns
    return selected


def should_publish_pair(pair_index: int, start_index: int, max_pairs: int) -> bool:
    if start_index < 1:
        raise ValueError("--start-index must be >= 1.")
    if pair_index < start_index:
        return False
    return max_pairs <= 0 or pair_index < start_index + max_pairs


def read_png(path: Path, encoding: str):
    flag = cv2.IMREAD_GRAYSCALE if encoding == "mono8" else cv2.IMREAD_COLOR
    image = cv2.imread(str(path), flag)
    if image is None:
        raise RuntimeError(f"Failed to read PNG: {path}")
    if encoding == "rgb8":
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    return image


def make_ros_image(image, stamp: rospy.Time, frame_id: str, encoding: str) -> Image:
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = image.shape[0]
    msg.width = image.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    channels = 1 if image.ndim == 2 else image.shape[2]
    msg.step = image.shape[1] * channels
    msg.data = image.tobytes()
    return msg


def selected_pairs(args: argparse.Namespace) -> List[PngPair]:
    frames1 = scan_png_folder(Path(args.camera1_dir).expanduser().resolve())
    frames2 = scan_png_folder(Path(args.camera2_dir).expanduser().resolve())
    if args.pairing == "ordered":
        pairs = pair_ordered(frames1, frames2)
    else:
        pairs = pair_nearest(frames1, frames2, int(args.max_skew_sec * 1e9))
    if not pairs:
        raise RuntimeError("No frame pairs found. Try --pairing ordered or increase --max-skew-sec.")
    return downsample_pairs(pairs, args.hz)


def main() -> None:
    args = parse_args()
    if args.max_in_flight < 1:
        raise ValueError("--max-in-flight must be >= 1.")

    pairs = selected_pairs(args)
    rospy.init_node("controlled_png_pair_player", anonymous=True)

    log(f"Camera 1 dir: {Path(args.camera1_dir).expanduser().resolve()}")
    log(f"Camera 2 dir: {Path(args.camera2_dir).expanduser().resolve()}")
    log(f"Selected frame pairs: {len(pairs)}")
    log(f"Publishing topics: {args.topic1} and {args.topic2}")

    tracker = AckTracker()
    rospy.Subscriber(args.ack1, Header, tracker.ack1, queue_size=100)
    rospy.Subscriber(args.ack2, Header, tracker.ack2, queue_size=100)
    pub1 = rospy.Publisher(args.topic1, Image, queue_size=args.queue_size)
    pub2 = rospy.Publisher(args.topic2, Image, queue_size=args.queue_size)

    if args.wait_for_subscribers:
        log("Waiting for image subscribers...")
        wait_for_subscribers(pub1, pub2, args.subscriber_timeout_sec)
        log("Image subscribers connected.")

    rospy.sleep(1.0)
    started = time.monotonic()
    first_stamp_ns = pairs[0].stamp_ns
    ros_base = rospy.Time.now()
    wall_start = time.monotonic()
    published = 0
    log_every = max(args.log_every, 1)

    for pair in pairs:
        if rospy.is_shutdown():
            break
        if not should_publish_pair(pair.index, args.start_index, args.max_pairs):
            if args.max_pairs > 0 and pair.index >= args.start_index + args.max_pairs:
                break
            continue

        if args.playback_rate > 0.0:
            target_elapsed = (pair.stamp_ns - first_stamp_ns) / 1e9 / args.playback_rate
            while time.monotonic() < wall_start + target_elapsed and not rospy.is_shutdown():
                time.sleep(0.005)

        tracker.wait_for_capacity(args.max_in_flight, args.timeout_sec)
        stamp1 = ros_base + rospy.Duration.from_sec((pair.frame1.stamp_ns - first_stamp_ns) / 1e9)
        stamp2 = ros_base + rospy.Duration.from_sec((pair.frame2.stamp_ns - first_stamp_ns) / 1e9)
        tracker.add_pending(stamp1, stamp2)

        image1 = read_png(pair.frame1.path, args.encoding)
        image2 = read_png(pair.frame2.path, args.encoding)
        pub1.publish(make_ros_image(image1, stamp1, args.frame_id1, args.encoding))
        pub2.publish(make_ros_image(image2, stamp2, args.frame_id2, args.encoding))
        published += 1

        if published == 1 or published % log_every == 0:
            elapsed = max(time.monotonic() - started, 1e-6)
            log(
                f"Published {published} pairs through source pair index {pair.index}, "
                f"pending={tracker.pending_count()}, effective rate={published / elapsed:.3f} pairs/s"
            )

    if published == 0:
        raise RuntimeError("No frame pairs were published. Check --start-index and --max-pairs.")

    log("All selected pairs published. Waiting for final ACKs...")
    tracker.wait_for_all(args.timeout_sec)
    log("All selected frame pairs were ACKed by orbcalib.")


if __name__ == "__main__":
    main()
