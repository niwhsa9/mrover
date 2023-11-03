#include <ros/ros.h>

#include <motors_manager.hpp>


int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushless_test_bridge");
    ros::NodeHandle nh;

    [[maybe_unused]] auto brushlessController = std::make_unique<mrover::BrushlessController>(nh, "test_brushless_controller");

    // Enter the ROS event loop
    ros::spin();

    return EXIT_SUCCESS;
}
