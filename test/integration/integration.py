#!/usr/bin/env python3

import sys
import unittest

import numpy as np

import rospy
import rostest
import tf2_ros
from mrover.msg import Waypoint, WaypointType
from util.SE3 import SE3
from util.course_service import CourseService

COMPLETION_TOLERANCE = 3.0


class TestIntegration(unittest.TestCase):
    def test_integration(self):
        rospy.logdebug("Integration Test Starting")

        rospy.wait_for_service(CourseService.SERVICE_NAME)  # Wait until we can run our service
        rospy.sleep(0.5)

        rospy.init_node("integration_test")
        publish_course = CourseService()

        rospy.loginfo("Integration Test Ready")

        waypoint_in_world = SE3(position=np.array([-5.5, -5.5, 0.0]))
        publish_course(
            [
                (
                    Waypoint(fiducial_id=0, tf_id="course0", type=WaypointType(val=WaypointType.NO_SEARCH)),
                    waypoint_in_world,
                ),
            ]
        )

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        has_reached_target = False

        while not rospy.is_shutdown():
            try:
                rover_in_world = SE3.from_tf_tree(tf_buffer, parent_frame="map", child_frame="base_link")
                distance_to_target = waypoint_in_world.pos_distance_to(rover_in_world)
                rospy.logdebug(distance_to_target)
                if distance_to_target < COMPLETION_TOLERANCE:
                    has_reached_target = True
                if has_reached_target:
                    rospy.signal_shutdown("Finished test")
                    break
            except tf2_ros.TransformException as exception:
                pass


if __name__ == "__main__":
    rostest.rosrun("mrover", "integration_test", TestIntegration, sys.argv)
