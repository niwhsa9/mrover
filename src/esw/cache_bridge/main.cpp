#include <ros/ros.h>

#include <motors_manager.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "cache_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    [[maybe_unused]] auto cacheManager = std::make_unique<mrover::MotorsManager>(nh, "cache");

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}