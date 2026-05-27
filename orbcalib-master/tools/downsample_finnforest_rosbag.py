#!/usr/bin/env python3
"""
Downsample an existing FinnForest C1/C4 ROS1 bag without reading raw/PNG data.

This is useful when the original raw timestamp folders are not available but a
full-rate ROS1 bag already exists. Messages are selected independently per image
topic using their header stamps.
"""

import argparse
from pathlib import Path
from typing import Dict, Iterable

import rosbag
from rosbag import Compression
import rospy


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", required=True, help="Input ROS1 bag path.")
    parser.add_argument("--output", required=True, help="Output downsampled ROS1 bag path.")
    parser.add_argument(
        "--hz",
        type=float,
        default=13.3333333333,
        help="Target output rate per topic.",
    )
    parser.add_argument(
        "--topics",
        nargs="+",
        default=["/cam_c1/image", "/cam_c4/image"],
        help="Topics to downsample and write.",
    )
    parser.add_argument(
        "--compression",
        choices=("none", "bz2", "lz4"),
        default="none",
        help="Output bag compression. lz4 is fast if supported by the ROS image.",
    )
    return parser.parse_args()


def msg_stamp(msg, fallback: rospy.Time) -> rospy.Time:
    header = getattr(msg, "header", None)
    stamp = getattr(header, "stamp", None)
    if stamp is None or stamp == rospy.Time(0):
        return fallback
    return stamp


def compression_mode(name: str):
    if name == "bz2":
        return Compression.BZ2
    if name == "lz4":
        return Compression.LZ4
    return Compression.NONE


def downsample(input_path: Path, output_path: Path, topics: Iterable[str], hz: float, compression: str) -> None:
    if hz <= 0:
        raise ValueError("--hz must be > 0")

    topic_set = set(topics)
    step = rospy.Duration.from_sec(1.0 / hz)
    next_stamp_by_topic: Dict[str, rospy.Time] = {}
    written_by_topic: Dict[str, int] = {topic: 0 for topic in topic_set}
    read_by_topic: Dict[str, int] = {topic: 0 for topic in topic_set}

    output_path.parent.mkdir(parents=True, exist_ok=True)

    with rosbag.Bag(str(input_path), "r") as in_bag, rosbag.Bag(str(output_path), "w", compression=compression_mode(compression)) as out_bag:
        for topic, msg, bag_time in in_bag.read_messages(topics=list(topic_set)):
            read_by_topic[topic] += 1
            stamp = msg_stamp(msg, bag_time)

            if topic not in next_stamp_by_topic:
                next_stamp_by_topic[topic] = stamp

            if stamp + rospy.Duration.from_sec(1e-9) >= next_stamp_by_topic[topic]:
                out_bag.write(topic, msg, t=bag_time)
                written_by_topic[topic] += 1
                while next_stamp_by_topic[topic] <= stamp:
                    next_stamp_by_topic[topic] += step

                count = written_by_topic[topic]
                if count == 1 or count % 500 == 0:
                    print(f"{topic}: wrote {count} messages at stamp {stamp.to_sec():.6f}")

    print("Done.")
    for topic in sorted(topic_set):
        print(f"{topic}: read {read_by_topic[topic]}, wrote {written_by_topic[topic]}")


def main() -> None:
    args = parse_args()
    downsample(
        input_path=Path(args.input).expanduser().resolve(),
        output_path=Path(args.output).expanduser().resolve(),
        topics=args.topics,
        hz=args.hz,
        compression=args.compression,
    )


if __name__ == "__main__":
    main()
