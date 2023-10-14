#include "can_manager.hpp"
#include <motors_manager.hpp>
#include <ros/ros.h>
#include <std_srvs/SetBool.h>

std::vector<std::string> SANames{"sa_x", "sa_y", "sa_z", "scoop", "drill"};
std::unique_ptr<CANManager> uv_bulb_can_manager;

bool uvBulbCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
    uv_bulb_can_manager->send_data("uv_bulb_cmd", req.data);
    res.success = true;
    res.message = "DONE";
    return true;
}

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "sa_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    uv_bulb_can_manager = std::make_unique<CANManager>(nh, "uv_bulb");
    [[maybe_unused]] auto SAManager = std::make_unique<mrover::MotorsManager>(nh, "sa", SANames);
    nh.advertiseService("sa_enable_uv_bulb", uvBulbCallback);
    // Enter the ROS event loop
    ros::spin();

    return 0;
}