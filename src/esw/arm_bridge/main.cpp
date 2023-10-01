#include "../motor_library/motors_manager.hpp"
#include <ros/ros.h>
#include <mrover/Throttle.h>
#include <std_msgs/Float32.h> // To publish heartbeats

void moveArmThrottle(const mrover::Throttle::ConstPtr& msg);
void heartbeatCallback(const ros::TimerEvent&);

MotorsManager armManager;
std::vector<std::string> armNames =
        {"joint_a", "joint_b", "joint_c", "joint_de", "finger", "gripper"};

std::unordered_map<std::string, float> motorMultipliers; // Store the multipliers for each motor

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "arm_bridge");
    ros::NodeHandle nh;

    // Load motor controllers configuration from the ROS parameter server
    XmlRpc::XmlRpcValue controllersRoot;
    assert(nh.getParam("motors/controllers", controllersRoot));
    assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    armManager = MotorsManager(&nh, armNames, controllersRoot);

    // Load motor multipliers from the ROS parameter server
    XmlRpc::XmlRpcValue armControllers;
    assert(nh.getParam("arm/controllers", armControllers));
    assert(armControllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (const auto& armName: armNames) {
        assert(armControllers.hasMember(armName));
        assert(armControllers[armName].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        if (armControllers[armName].hasMember("multiplier")) {
            motorMultipliers[armName] = static_cast<double>(armControllers[armName]["multiplier"]);
        }
    }

    // Subscribe to the ROS topic for arm commands
    ros::Subscriber moveArmSubscriber = n->subscribe<sensor_msgs::JointState>("ra_throttle_cmd", 1, moveArmThrottle);

    // Create a 0.1 second heartbeat timer
    ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), heartbeatCallback);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveArmThrottle(const mrover::Throttle::ConstPtr& msg) {
    if (msg->name != armNames && msg->name.size() != msg->velocity.size()) {
        ROS_ERROR("Arm request is invalid!");
        return;
    }
    for (size_t i = 0; i < msg->name.size(); ++i) {
        std::string& name = msg->name[i];
        Controller& controller = armManager.get_controller(name);
        float velocity = std::clamp(msg->velocity[i], -1.0, 1.0);
        controller.set_desired_speed_throttle(velocity);
    }

    // Set the messageReceived flag to true when a message is received
    messageReceived = true;
}

void heartbeatCallback(const ros::TimerEvent&) {
    // If no message has been received within the last 0.1 seconds, set desired speed to 0 for all motors
    if (!messageReceived) {
        for (const auto& armName: armNames) {
            Controller& controller = armManager.get_controller(armName);
            controller.set_desired_speed(0.0);
        }
    }

    // Reset the messageReceived flag
    messageReceived = false;
}
