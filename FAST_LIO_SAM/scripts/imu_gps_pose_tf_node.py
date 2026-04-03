#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math

import rospy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import Imu, NavSatFix
import tf2_ros


class ImuGpsPoseTfNode(object):
    def __init__(self):
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.child_frame = rospy.get_param("~child_frame", "imu_link")
        self.gps_topic = rospy.get_param("~gps_topic", "/imu/gps")
        self.imu_topic = rospy.get_param("~imu_topic", "/imu/data")
        self.pose_topic = rospy.get_param("~pose_topic", "/imu/gps_pose")
        self.path_topic = rospy.get_param("~path_topic", "/imu/gps_path")
        self.path_max_len = rospy.get_param("~path_max_len", 5000000)

        self.origin_lat = None
        self.origin_lon = None
        self.origin_alt = None

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0

        self.has_orientation = False
        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        self.pose_pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=10)
        self.path_pub = rospy.Publisher(self.path_topic, Path, queue_size=10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

        self.path_msg = Path()
        self.path_msg.header.frame_id = self.map_frame

        self.gps_sub = rospy.Subscriber(self.gps_topic, NavSatFix, self.gps_callback, queue_size=50)
        self.imu_sub = rospy.Subscriber(self.imu_topic, Imu, self.imu_callback, queue_size=100)

        rospy.loginfo("imu_gps_pose_tf_node started")
        rospy.loginfo("gps_topic: %s, imu_topic: %s", self.gps_topic, self.imu_topic)
        rospy.loginfo(
            "publish pose: %s, path: %s, tf: %s -> %s",
            self.pose_topic,
            self.path_topic,
            self.map_frame,
            self.child_frame,
        )

    @staticmethod
    def wgs84_to_local_xy(lat_deg, lon_deg, lat0_deg, lon0_deg):
        # Use local tangent plane approximation around the first GPS frame.
        earth_radius = 6378137.0
        lat = math.radians(lat_deg)
        lon = math.radians(lon_deg)
        lat0 = math.radians(lat0_deg)
        lon0 = math.radians(lon0_deg)

        x = (lon - lon0) * math.cos(lat0) * earth_radius
        y = (lat - lat0) * earth_radius
        return x, y

    def imu_callback(self, msg):
        self.qx = msg.orientation.x
        self.qy = msg.orientation.y
        self.qz = msg.orientation.z
        self.qw = msg.orientation.w
        self.has_orientation = True

        self.publish_tf(msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now())

    def gps_callback(self, msg):
        if math.isnan(msg.latitude) or math.isnan(msg.longitude) or math.isnan(msg.altitude):
            rospy.logwarn_throttle(5.0, "Received NaN in /imu/gps, skip this frame")
            return

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.origin_alt = msg.altitude
            rospy.loginfo("Set GPS origin: lat=%.10f, lon=%.10f, alt=%.3f", self.origin_lat, self.origin_lon, self.origin_alt)

        self.current_x, self.current_y = self.wgs84_to_local_xy(
            msg.latitude,
            msg.longitude,
            self.origin_lat,
            self.origin_lon,
        )
        self.current_z = msg.altitude - self.origin_alt

        stamp = msg.header.stamp if msg.header.stamp != rospy.Time() else rospy.Time.now()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.map_frame
        pose_msg.pose.position.x = self.current_x
        pose_msg.pose.position.y = self.current_y
        pose_msg.pose.position.z = self.current_z

        if self.has_orientation:
            pose_msg.pose.orientation.x = self.qx
            pose_msg.pose.orientation.y = self.qy
            pose_msg.pose.orientation.z = self.qz
            pose_msg.pose.orientation.w = self.qw
        else:
            pose_msg.pose.orientation.w = 1.0

        self.pose_pub.publish(pose_msg)

        self.path_msg.header.stamp = stamp
        self.path_msg.poses.append(pose_msg)
        if self.path_max_len > 0 and len(self.path_msg.poses) > self.path_max_len:
            self.path_msg.poses = self.path_msg.poses[-self.path_max_len :]
        self.path_pub.publish(self.path_msg)

        self.publish_tf(stamp)

    def publish_tf(self, stamp):
        tf_msg = TransformStamped()
        tf_msg.header.stamp = stamp
        tf_msg.header.frame_id = self.map_frame
        tf_msg.child_frame_id = self.child_frame

        tf_msg.transform.translation.x = self.current_x
        tf_msg.transform.translation.y = self.current_y
        tf_msg.transform.translation.z = self.current_z

        tf_msg.transform.rotation.x = self.qx
        tf_msg.transform.rotation.y = self.qy
        tf_msg.transform.rotation.z = self.qz
        tf_msg.transform.rotation.w = self.qw

        self.tf_broadcaster.sendTransform(tf_msg)


def main():
    rospy.init_node("imu_gps_pose_tf_node")
    ImuGpsPoseTfNode()
    rospy.spin()


if __name__ == "__main__":
    main()
