#!/usr/bin/env python3
"""
Node for testing service requests for services that are still in development
Logs the service request to stdout
"""

from typing import Any
import rospy
from std_srvs.srv import Trigger, TriggerResponse

# Change these values for the service name and type definition to test different values
SERVICE_NAME = "mcu_board_reset"
SERVICE_TYPE = Trigger


def print_service_request(service_request: Any):
    rospy.loginfo(service_request)
    return TriggerResponse(success=True, message="")


def main():
    rospy.init_node("debug_service")
    rospy.Service(SERVICE_NAME, SERVICE_TYPE, print_service_request)
    rospy.spin()


if __name__ == "__main__":
    main()
