#!/usr/bin/env python3
import argparse
import time
import rospy
from sensor_msgs.msg import Image
from mcap_ros2.reader import read_ros2_messages

TOPICS = [
    "/cam_front/image",
    "/cam_front/depth_image",
    "/cam_back/image",
    "/cam_back/depth_image",
    #"/robot2/cam_front_left/image",
    #"/robot2/cam_front_left/depth_image",
    #"/robot2/cam_front_right/image",
    #"/robot2/cam_front_right/depth_image",
]

def ros2_to_ros1_image(msg2):
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

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("mcap")
    parser.add_argument("--rate", type=float, default=1.0)
    args = parser.parse_args()

    rospy.init_node("mcap_to_ros1_relay", anonymous=True)
    pubs = {topic: rospy.Publisher(topic, Image, queue_size=10) for topic in TOPICS}

    first_log_ns = None
    wall_start = None

    for m in read_ros2_messages(args.mcap, topics=TOPICS):
        if rospy.is_shutdown():
            break
        if first_log_ns is None:
            first_log_ns = m.log_time_ns
            wall_start = time.monotonic()
        target = (m.log_time_ns - first_log_ns) / 1e9 / args.rate
        while True:
            now = time.monotonic() - wall_start
            dt = target - now
            if dt <= 0:
                break
            time.sleep(min(dt, 0.01))
        pubs[m.channel.topic].publish(ros2_to_ros1_image(m.ros_msg))

if __name__ == "__main__":
    main()
