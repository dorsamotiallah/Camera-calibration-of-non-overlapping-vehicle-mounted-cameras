#!/usr/bin/env python3
"""
ACK-driven ROS1 player for RGB-D front/back image topics stored in a ROS2 MCAP.

It publishes one synchronized four-message bundle:
  front RGB, front depth, back RGB, back depth
then waits for orbcalib ACKs from both cameras before publishing the next bundle.
"""

import argparse
import threading
import time
from collections import defaultdict, deque
from typing import Deque, Dict, Tuple

import rospy
from mcap_ros2.reader import read_ros2_messages
from sensor_msgs.msg import Image
from std_msgs.msg import Header


StampKey = Tuple[int, int]
TOPIC_FRONT_IMAGE = "/cam_front/image"
TOPIC_FRONT_DEPTH = "/cam_front/depth_image"
TOPIC_BACK_IMAGE = "/cam_back/image"
TOPIC_BACK_DEPTH = "/cam_back/depth_image"
TOPICS = [TOPIC_FRONT_IMAGE, TOPIC_FRONT_DEPTH, TOPIC_BACK_IMAGE, TOPIC_BACK_DEPTH]


def stamp_key_from_ros2(msg) -> StampKey:
    return msg.header.stamp.sec, msg.header.stamp.nanosec


def stamp_key_from_ros1(stamp: rospy.Time) -> StampKey:
    return stamp.secs, stamp.nsecs


def stamp_to_ros1(key: StampKey) -> rospy.Time:
    return rospy.Time(key[0], key[1])


def log(message: str) -> None:
    print(message, flush=True)


def ros2_to_ros1_image(msg2) -> Image:
    msg1 = Image()
    msg1.header.stamp = rospy.Time(msg2.header.stamp.sec, msg2.header.stamp.nanosec)
    msg1.header.frame_id = msg2.header.frame_id
    msg1.height = msg2.height
    msg1.width = msg2.width
    msg1.encoding = msg2.encoding
    msg1.is_bigendian = msg2.is_bigendian
    msg1.step = msg2.step
    msg1.data = bytes(msg2.data)
    return msg1


class AckTracker:
    def __init__(self) -> None:
        self._condition = threading.Condition()
        self._acked1 = set()
        self._acked2 = set()
        self._pending: Deque[Tuple[StampKey, StampKey]] = deque()

    def add_pending(self, stamp1: rospy.Time, stamp2: rospy.Time) -> None:
        with self._condition:
            self._pending.append((stamp_key_from_ros1(stamp1), stamp_key_from_ros1(stamp2)))
            self._condition.notify_all()

    def ack1(self, msg: Header) -> None:
        with self._condition:
            self._acked1.add(stamp_key_from_ros1(msg.stamp))
            self._prune_locked()
            self._condition.notify_all()

    def ack2(self, msg: Header) -> None:
        with self._condition:
            self._acked2.add(stamp_key_from_ros1(msg.stamp))
            self._prune_locked()
            self._condition.notify_all()

    def wait_for_capacity(self, max_in_flight: int, timeout_sec: float) -> None:
        deadline = time.monotonic() + timeout_sec
        with self._condition:
            while len(self._pending) >= max_in_flight and not rospy.is_shutdown():
                self._prune_locked()
                if len(self._pending) < max_in_flight:
                    return
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise TimeoutError(f"Timed out waiting for ACK capacity. Pending bundles: {len(self._pending)}")
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
                    raise TimeoutError(f"Timed out waiting for final ACKs. Pending bundles: {len(self._pending)}")
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


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("mcap")
    parser.add_argument("--front-image-topic", default=TOPIC_FRONT_IMAGE)
    parser.add_argument("--front-depth-topic", default=TOPIC_FRONT_DEPTH)
    parser.add_argument("--back-image-topic", default=TOPIC_BACK_IMAGE)
    parser.add_argument("--back-depth-topic", default=TOPIC_BACK_DEPTH)
    parser.add_argument("--ack1", default="/orbcalib/camera1/processed")
    parser.add_argument("--ack2", default="/orbcalib/camera2/processed")
    parser.add_argument("--max-in-flight", type=int, default=1)
    parser.add_argument("--timeout-sec", type=float, default=120.0)
    parser.add_argument("--start-index", type=int, default=1, help="1-based inclusive bundle index.")
    parser.add_argument("--max-bundles", type=int, default=0, help="0 means all bundles.")
    parser.add_argument("--wait-for-subscribers", action="store_true")
    parser.add_argument("--subscriber-timeout-sec", type=float, default=60.0)
    parser.add_argument("--queue-size", type=int, default=10)
    parser.add_argument("--log-every", type=int, default=100)
    return parser.parse_args()


def wait_for_subscribers(pubs, timeout_sec: float) -> None:
    deadline = time.monotonic() + timeout_sec
    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        if all(pub.get_num_connections() > 0 for pub in pubs):
            return
        if time.monotonic() >= deadline:
            raise TimeoutError("Timed out waiting for image/depth subscribers.")
        rate.sleep()


def should_publish(index: int, start_index: int, max_bundles: int) -> bool:
    if start_index < 1:
        raise ValueError("--start-index must be >= 1.")
    if index < start_index:
        return False
    return max_bundles <= 0 or index < start_index + max_bundles


def main() -> None:
    args = parse_args()
    topics = [args.front_image_topic, args.front_depth_topic, args.back_image_topic, args.back_depth_topic]

    rospy.init_node("controlled_mcap_rgbd_player", anonymous=True)
    pubs = {
        args.front_image_topic: rospy.Publisher(args.front_image_topic, Image, queue_size=args.queue_size),
        args.front_depth_topic: rospy.Publisher(args.front_depth_topic, Image, queue_size=args.queue_size),
        args.back_image_topic: rospy.Publisher(args.back_image_topic, Image, queue_size=args.queue_size),
        args.back_depth_topic: rospy.Publisher(args.back_depth_topic, Image, queue_size=args.queue_size),
    }

    ack = AckTracker()
    rospy.Subscriber(args.ack1, Header, ack.ack1, queue_size=100)
    rospy.Subscriber(args.ack2, Header, ack.ack2, queue_size=100)

    if args.wait_for_subscribers:
        log("Waiting for orbcalib image/depth subscribers...")
        wait_for_subscribers(list(pubs.values()), args.subscriber_timeout_sec)

    buffers: Dict[StampKey, Dict[str, object]] = defaultdict(dict)
    bundle_index = 0
    published = 0

    for m in read_ros2_messages(args.mcap, topics=topics):
        if rospy.is_shutdown():
            break

        topic = m.channel.topic
        key = stamp_key_from_ros2(m.ros_msg)
        buffers[key][topic] = ros2_to_ros1_image(m.ros_msg)

        bundle = buffers.get(key)
        if bundle is None or not all(t in bundle for t in topics):
            continue

        bundle_index += 1
        if should_publish(bundle_index, args.start_index, args.max_bundles):
            ack.wait_for_capacity(args.max_in_flight, args.timeout_sec)
            pubs[args.front_image_topic].publish(bundle[args.front_image_topic])
            pubs[args.front_depth_topic].publish(bundle[args.front_depth_topic])
            pubs[args.back_image_topic].publish(bundle[args.back_image_topic])
            pubs[args.back_depth_topic].publish(bundle[args.back_depth_topic])
            ack.add_pending(bundle[args.front_image_topic].header.stamp, bundle[args.back_image_topic].header.stamp)
            published += 1
            if published == 1 or (args.log_every > 0 and published % args.log_every == 0):
                log(f"Published RGB-D bundle {published} stamp {stamp_to_ros1(key).to_sec():.6f}")

        del buffers[key]

        # Drop incomplete old bundles. Exact timestamp matching should keep this
        # near zero, but this prevents unbounded growth if a topic is missing.
        if len(buffers) > 100:
            for old_key in sorted(buffers.keys())[:-50]:
                del buffers[old_key]

    ack.wait_for_all(args.timeout_sec)
    log(f"Finished controlled MCAP playback. Published bundles: {published}")


if __name__ == "__main__":
    main()
