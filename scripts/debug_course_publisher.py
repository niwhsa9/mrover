#!/usr/bin/env python3

import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped

from mrover.msg import Course, Waypoint
import mrover.srv


def send_waypoint(frame_id: str, x: float, y: float, fid_id: int = -1) -> Waypoint:
    tf_broadcaster = tf2_ros.TransformBroadcaster()
    t = TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "odom"
    t.child_frame_id = frame_id
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.rotation.w = 1
    tf_broadcaster.sendTransform(t)
    print(f'sending {frame_id}')
    w = Waypoint()
    w.fiducial_id = fid_id
    w.tf_id = frame_id
    return w


if __name__ == "__main__":
    rospy.init_node("debug_course_publisher")

    publish_course = rospy.ServiceProxy('course_service', mrover.srv.PublishCourse)

    c = Course()
    c.waypoints.append(send_waypoint("course1", -3, -3))
    c.waypoints.append(send_waypoint("course2", -5, -5, 0))
    publish_course(c)
    while not rospy.is_shutdown():
        pass