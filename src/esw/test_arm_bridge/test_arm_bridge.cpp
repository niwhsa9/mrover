#include <iostream>

#include <ros/init.h>
#include <ros/node_handle.h>

#include <mrover/Throttle.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_arm_bridge");
    ros::NodeHandle nh;

    ros::Publisher ros_pub = nh.advertise<mrover::Throttle>("arm_throttle_cmd", 1);
    mrover::Throttle throttle_cmd;
    throttle_cmd.names = {"joint_a", "joint_b", "joint_c", "joint_de_0", "joint_de_1", "allen_key", "gripper"};
    throttle_cmd.throttles = {0, 0, 0, 0, 0, 0, 0};

    ros::Rate rate(1);
    while (ros::ok()) {
        ros_pub.publish(throttle_cmd);
        rate.sleep();
        ros::spinOnce();
    }

    return EXIT_SUCCESS;
}
