#!/usr/bin/env python3
"""Interactively record nav_msgs/Odometry messages and save them as CSV.

Run this script in a terminal, then type ``s`` to begin buffering samples
and ``e`` to write the buffered samples to a CSV file.  The ROS subscriber
remains active after saving, so multiple recordings can be made in one run.
"""

import argparse
import csv
import os
import sys
import threading
from datetime import datetime

import rospy
from nav_msgs.msg import Odometry


CSV_HEADER = (
    "stamp_sec",
    "frame_id",
    "child_frame_id",
    "position_x",
    "position_y",
    "position_z",
    "orientation_x",
    "orientation_y",
    "orientation_z",
    "orientation_w",
    "linear_velocity_x",
    "linear_velocity_y",
    "linear_velocity_z",
    "angular_velocity_x",
    "angular_velocity_y",
    "angular_velocity_z",
)


class OdometryCsvRecorder:
    def __init__(self, topic, output_dir):
        self._output_dir = os.path.abspath(os.path.expanduser(output_dir))
        self._lock = threading.Lock()
        self._recording = False
        self._samples = []
        self._recording_started_at = None
        self._subscriber = rospy.Subscriber(topic, Odometry, self._callback, queue_size=1000)

    def _callback(self, message):
        pose = message.pose.pose
        twist = message.twist.twist
        row = (
            "{:.9f}".format(message.header.stamp.to_sec()),
            message.header.frame_id,
            message.child_frame_id,
            pose.position.x,
            pose.position.y,
            pose.position.z,
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
            twist.linear.x,
            twist.linear.y,
            twist.linear.z,
            twist.angular.x,
            twist.angular.y,
            twist.angular.z,
        )
        with self._lock:
            if self._recording:
                self._samples.append(row)

    def start(self):
        with self._lock:
            if self._recording:
                return False, "正在记录中；请先输入 e 保存当前记录。"
            self._samples = []
            self._recording = True
            self._recording_started_at = datetime.now()
        return True, "已开始记录。输入 e 结束并保存 CSV。"

    def stop_and_save(self):
        with self._lock:
            if not self._recording:
                return False, "当前未在记录。请先输入 s。"
            self._recording = False
            samples = self._samples
            self._samples = []
            started_at = self._recording_started_at
            self._recording_started_at = None

        os.makedirs(self._output_dir, exist_ok=True)
        filename = "odometry_{}.csv".format(started_at.strftime("%Y%m%d_%H%M%S"))
        output_path = os.path.join(self._output_dir, filename)
        try:
            with open(output_path, "w", newline="") as csv_file:
                writer = csv.writer(csv_file)
                writer.writerow(CSV_HEADER)
                writer.writerows(samples)
        except OSError as error:
            return False, "CSV 保存失败: {}".format(error)

        return True, "已保存 {} 条里程计到 {}".format(len(samples), output_path)

    def discard(self):
        with self._lock:
            was_recording = self._recording
            self._recording = False
            self._samples = []
            self._recording_started_at = None
        return was_recording


def parse_arguments():
    parser = argparse.ArgumentParser(description="Record a ROS1 Odometry topic to CSV on demand.")
    parser.add_argument("--topic", default="/Odometry", help="Odometry topic to subscribe to (default: /Odometry).")
    parser.add_argument(
        "--output-dir",
        default="/home/ugv",
        help="Directory where CSV files are written (default: /home/ugv).",
    )
    return parser.parse_args(rospy.myargv(argv=sys.argv)[1:])


def print_help():
    print("可用命令: s（开始/清空本次记录）, e（结束并保存 CSV）, i（查看状态）, q（退出）, h（帮助）")


def main():
    rospy.init_node("odometry_csv_recorder", anonymous=True)
    args = parse_arguments()
    recorder = OdometryCsvRecorder(args.topic, args.output_dir)
    rospy.loginfo("订阅 %s；CSV 将保存到 %s", args.topic, os.path.abspath(os.path.expanduser(args.output_dir)))
    print_help()

    try:
        while not rospy.is_shutdown():
            try:
                command = input("odometry-recorder> ").strip().lower()
            except EOFError:
                command = "quit"

            if command == "s":
                _, message = recorder.start()
                print(message)
            elif command == "e":
                _, message = recorder.stop_and_save()
                print(message)
            elif command == "i":
                with recorder._lock:
                    state = "记录中" if recorder._recording else "空闲"
                    count = len(recorder._samples)
                print("状态: {}，当前缓存 {} 条。".format(state, count))
            elif command == "q":
                if recorder.discard():
                    print("未保存的记录已丢弃；如需保存请先输入 e。")
                break
            elif command == "h":
                print_help()
            elif command:
                print("未知命令: {}".format(command))
                print_help()
    except KeyboardInterrupt:
        if recorder.discard():
            print("\n未保存的记录已丢弃；如需保存请先输入 e。")
    finally:
        recorder._subscriber.unregister()


if __name__ == "__main__":
    main()
