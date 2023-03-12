#!/usr/bin/env python3
"""
	Node for testing service requests for services that are still in development
	Logs the service request to stdout
"""

from typing import Any
import rospy
from mrover.srv import CalibrateMotors

# Change these values for the service name and type definition to test different values
SERVICE_NAME = "/calibrate"
SERVICE_TYPE = CalibrateMotors


def print_service_request(service_request: Any):
    rospy.loginfo(service_request)


def main():
    rospy.init_node("debug_service")
    rospy.Service(SERVICE_NAME, SERVICE_TYPE, print_service_request)
    rospy.spin()


if __name__ == "__main__":
    main()
