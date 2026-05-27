#!/usr/bin/env python3
"""
ACK-driven ROS1 player for FinnForest C1/C4 bags.

This still publishes normal sensor_msgs/Image topics for orbcalib/ORB-SLAM.
The difference from `rosbag play` is backpressure: after publishing image pairs,
the player waits for orbcalib ACK topics that are emitted after TrackMonocular
returns. That prevents unbounded ROS queues while preserving all selected frames.
"""

import argparse
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Deque, Iterator, Tuple

import rosbag
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import Header


StampKey = Tuple[int, int]


def stamp_key(stamp: rospy.Time) -> StampKey:
    return stamp.secs, stamp.nsecs


def log(message: str) -> None:
    print(message, flush=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--bag",
        required=True,
        help="Input ROS1 bag containing C1/C4 image topics.",
    )
    parser.add_argument(
        "--topic-c1",
        default="/cam_c1/image",
        help="Input and output topic for C1 images.",
    )
    parser.add_argument(
        "--topic-c4",
        default="/cam_c4/image",
        help="Input and output topic for C4 images.",
    )
    parser.add_argument(
        "--ack-c1",
        default="/orbcalib/camera1/processed",
        help="ACK topic published by orbcalib after C1 TrackMonocular returns.",
    )
    parser.add_argument(
        "--ack-c4",
        default="/orbcalib/camera2/processed",
        help="ACK topic published by orbcalib after C4 TrackMonocular returns.",
    )
    parser.add_argument(
        "--max-in-flight",
        type=int,
        default=1,
        help="Maximum image pairs published but not fully ACKed.",
    )
    parser.add_argument(
        "--timeout-sec",
        type=float,
        default=120.0,
        help="Maximum time to wait for an ACK before failing.",
    )
    parser.add_argument(
        "--start-index",
        type=int,
        default=1,
        help="1-based inclusive pair index to start from after pairing by order.",
    )
    parser.add_argument(
        "--max-pairs",
        type=int,
        default=0,
        help="Optional maximum number of pairs to publish. 0 means all pairs.",
    )
    parser.add_argument(
        "--wait-for-subscribers",
        action="store_true",
        help="Wait until both image topics have at least one subscriber.",
    )
    parser.add_argument(
        "--subscriber-timeout-sec",
        type=float,
        default=60.0,
        help="Timeout used with --wait-for-subscribers.",
    )
    parser.add_argument(
        "--log-every",
        type=int,
        default=100,
        help="Log progress every N pairs.",
    )
    parser.add_argument(
        "--queue-size",
        type=int,
        default=10,
        help="ROS publisher queue size.",
    )
    return parser.parse_args()


class AckTracker:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._acked_c1 = set()
        self._acked_c4 = set()
        self._pending: Deque[Tuple[StampKey, StampKey]] = deque()

    def add_pending(self, c1_stamp: rospy.Time, c4_stamp: rospy.Time) -> None:
        with self._condition:
            self._pending.append((stamp_key(c1_stamp), stamp_key(c4_stamp)))
            self._condition.notify_all()

    def ack_c1(self, msg: Header) -> None:
        with self._condition:
            self._acked_c1.add(stamp_key(msg.stamp))
            self._prune_locked()
            self._condition.notify_all()

    def ack_c4(self, msg: Header) -> None:
        with self._condition:
            self._acked_c4.add(stamp_key(msg.stamp))
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
                    raise TimeoutError(
                        f"Timed out waiting for ACK capacity. Pending pairs: {len(self._pending)}"
                    )
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
                    raise TimeoutError(
                        f"Timed out waiting for final ACKs. Pending pairs: {len(self._pending)}"
                    )
                self._condition.wait(min(remaining, 0.2))

    def _prune_locked(self) -> None:
        while self._pending:
            c1_stamp, c4_stamp = self._pending[0]
            if c1_stamp in self._acked_c1 and c4_stamp in self._acked_c4:
                self._pending.popleft()
                self._acked_c1.discard(c1_stamp)
                self._acked_c4.discard(c4_stamp)
            else:
                break


def wait_for_subscribers(pub_c1: rospy.Publisher, pub_c4: rospy.Publisher, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if pub_c1.get_num_connections() > 0 and pub_c4.get_num_connections() > 0:
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for C1/C4 image subscribers.")
        rate.sleep()


def iter_ordered_pairs(
    bag_path: Path,
    topic_c1: str,
    topic_c4: str,
) -> Iterator[Tuple[int, Image, Image]]:
    c1_queue: Deque[Image] = deque()
    c4_queue: Deque[Image] = deque()
    pair_index = 0

    with rosbag.Bag(str(bag_path), "r") as bag:
        for topic, msg, _ in bag.read_messages(topics=[topic_c1, topic_c4]):
            if topic == topic_c1:
                c1_queue.append(msg)
            elif topic == topic_c4:
                c4_queue.append(msg)

            while c1_queue and c4_queue:
                pair_index += 1
                yield pair_index, c1_queue.popleft(), c4_queue.popleft()

    if pair_index == 0:
        raise RuntimeError(
            f"No C1/C4 pairs found in {bag_path} for topics {topic_c1} and {topic_c4}"
        )
    if c1_queue or c4_queue:
        rospy.logwarn(
            "Unpaired messages left at end of bag: %s=%d, %s=%d",
            topic_c1,
            len(c1_queue),
            topic_c4,
            len(c4_queue),
        )


def should_publish_pair(
    pair_index: int,
    start_index: int,
    max_pairs: int,
) -> bool:
    if start_index < 1:
        raise ValueError("--start-index is 1-based and must be >= 1.")
    if pair_index < start_index:
        return False
    return max_pairs <= 0 or pair_index < start_index + max_pairs


def main() -> None:
    args = parse_args()
    if args.max_in_flight < 1:
        raise ValueError("--max-in-flight must be >= 1.")

    bag_path = Path(args.bag).expanduser().resolve()
    rospy.init_node("controlled_finnforest_bag_player", anonymous=True)

    log(f"Streaming bag: {bag_path}")

    tracker = AckTracker()
    rospy.Subscriber(args.ack_c1, Header, tracker.ack_c1, queue_size=100)
    rospy.Subscriber(args.ack_c4, Header, tracker.ack_c4, queue_size=100)
    pub_c1 = rospy.Publisher(args.topic_c1, Image, queue_size=args.queue_size)
    pub_c4 = rospy.Publisher(args.topic_c4, Image, queue_size=args.queue_size)

    if args.wait_for_subscribers:
        log("Waiting for image subscribers...")
        wait_for_subscribers(pub_c1, pub_c4, args.subscriber_timeout_sec)
        log("Image subscribers connected.")

    rospy.sleep(1.0)
    started = time.monotonic()
    log_every = max(args.log_every, 1)

    published = 0
    for pair_index, c1_msg, c4_msg in iter_ordered_pairs(bag_path, args.topic_c1, args.topic_c4):
        if rospy.is_shutdown():
            break
        if not should_publish_pair(pair_index, args.start_index, args.max_pairs):
            if args.max_pairs > 0 and pair_index >= args.start_index + args.max_pairs:
                break
            continue

        tracker.wait_for_capacity(args.max_in_flight, args.timeout_sec)
        tracker.add_pending(c1_msg.header.stamp, c4_msg.header.stamp)
        pub_c1.publish(c1_msg)
        pub_c4.publish(c4_msg)
        published += 1

        if published == 1 or published % log_every == 0:
            elapsed = max(time.monotonic() - started, 1e-6)
            log(
                "Published "
                f"{published} pairs through source pair index {pair_index}, "
                f"pending={tracker.pending_count()}, "
                f"effective rate={published / elapsed:.3f} pairs/s"
            )

    if published == 0:
        raise RuntimeError("No frame pairs were published. Check --start-index and --max-pairs.")

    log("All selected pairs published. Waiting for final ACKs...")
    tracker.wait_for_all(args.timeout_sec)
    log("All selected frame pairs were ACKed by orbcalib.")


if __name__ == "__main__":
    main()
