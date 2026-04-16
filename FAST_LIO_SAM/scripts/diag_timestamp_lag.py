#!/usr/bin/env python3
"""
TF 时间戳滞后诊断脚本
监控 IMU、点云、里程计话题的时间戳延迟，并输出到 CSV 供分析。

运行方式（roscore+bag+SLAM 均已启动后执行）：
  source catkin_slam/devel/setup.bash
  python3 catkin_slam/diag_timestamp_lag.py
"""

import rospy
import csv
import time
import sys
import os
from sensor_msgs.msg import Imu, PointCloud2
from nav_msgs.msg import Odometry

# ---------- 配置 ----------
IMU_TOPIC        = "/imu/data"
LIDAR_TOPIC      = "/rslidar_points"
ODOM_TOPIC       = "/Odometry"
ACCUM_MAP_TOPIC  = "/accumulated_map_points"

LOG_FILE         = os.path.expanduser("~/catkin_slam/timestamp_lag.csv")
PRINT_INTERVAL   = 5.0  # 每隔多少秒打印一次统计
# --------------------------

stats = {}          # topic -> {count, sum_lag, max_lag, last_lag, last_sim_t, last_cloud_size}
lock_data = {}

def make_callback(topic, is_cloud=False):
    def cb(msg):
        now = rospy.Time.now()
        stamp = msg.header.stamp
        if now.to_sec() < 1e-3 or stamp.to_sec() < 1e-3:
            return
        lag = (now - stamp).to_sec()
        sim_t = now.to_sec()
        size = 0
        if is_cloud:
            size = msg.width * msg.height  # 点数

        if topic not in stats:
            stats[topic] = dict(count=0, sum_lag=0.0, max_lag=0.0,
                                last_lag=0.0, last_sim_t=0.0, last_cloud_size=0)
        s = stats[topic]
        s['count'] += 1
        s['sum_lag'] += lag
        s['max_lag'] = max(s['max_lag'], lag)
        s['last_lag'] = lag
        s['last_sim_t'] = sim_t
        s['last_cloud_size'] = size

        # 写入 CSV（每帧追加）
        with open(LOG_FILE, 'a', newline='') as f:
            w = csv.writer(f)
            w.writerow([topic, f"{sim_t:.4f}", f"{lag:.4f}", size])

    return cb


def main():
    rospy.init_node('diag_timestamp_lag', anonymous=True)

    # 等待 clock 就绪
    rospy.loginfo("等待 /clock（use_sim_time=true）…")
    rospy.sleep(1.0)

    # 写 CSV 表头
    with open(LOG_FILE, 'w', newline='') as f:
        csv.writer(f).writerow(["topic", "sim_time_s", "lag_s", "cloud_size_pts"])
    rospy.loginfo(f"记录到 {LOG_FILE}")

    # 订阅
    rospy.Subscriber(IMU_TOPIC,       Imu,          make_callback(IMU_TOPIC))
    rospy.Subscriber(LIDAR_TOPIC,     PointCloud2,  make_callback(LIDAR_TOPIC,  is_cloud=True))
    rospy.Subscriber(ODOM_TOPIC,      Odometry,     make_callback(ODOM_TOPIC))
    rospy.Subscriber(ACCUM_MAP_TOPIC, PointCloud2,  make_callback(ACCUM_MAP_TOPIC, is_cloud=True))

    rospy.loginfo("开始监控，按 Ctrl+C 停止")
    rospy.loginfo(f"  IMU        : {IMU_TOPIC}")
    rospy.loginfo(f"  LiDAR      : {LIDAR_TOPIC}")
    rospy.loginfo(f"  里程计     : {ODOM_TOPIC}")
    rospy.loginfo(f"  累积点云   : {ACCUM_MAP_TOPIC}")
    print("-" * 80)
    print(f"{'话题':<30} {'仿真时间':>10} {'当前延迟(s)':>12} {'平均延迟(s)':>12} {'最大延迟(s)':>12} {'点数':>10}")
    print("-" * 80)

    rate = rospy.Rate(1.0 / PRINT_INTERVAL)
    while not rospy.is_shutdown():
        rate.sleep()
        print("-" * 80)
        for topic, s in sorted(stats.items()):
            avg = s['sum_lag'] / s['count'] if s['count'] > 0 else 0
            pts = s['last_cloud_size'] if s['last_cloud_size'] > 0 else '-'
            name = topic.split('/')[-1][:28]
            print(f"{name:<30} {s['last_sim_t']:>10.1f} {s['last_lag']:>12.3f} "
                  f"{avg:>12.3f} {s['max_lag']:>12.3f} {str(pts):>10}")

    print(f"\n完成。日志已写入 {LOG_FILE}")


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
