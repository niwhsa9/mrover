#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#include <can_device.hpp>
#include <motors_group.hpp>

using namespace mrover;

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg);

std::unique_ptr<MotorsGroup> driveManager;
std::vector<std::string> driveNames{"front_left", "front_right", "middle_left", "middle_right", "back_left", "back_right"};

std::unordered_map<std::string, Dimensionless> motorMultipliers; // Store the multipliers for each motor

Meters WHEEL_DISTANCE_INNER;
Meters WHEEL_DISTANCE_OUTER;
compound_unit<Radians, inverse<Meters>> WHEEL_LINEAR_TO_ANGULAR;
RadiansPerSecond MAX_MOTOR_SPEED;

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "drive_bridge");
    ros::NodeHandle nh;

    // Load motor multipliers from the ROS parameter server
    XmlRpc::XmlRpcValue driveControllers;
    assert(nh.hasParam("drive/controllers"));
    nh.getParam("drive/controllers", driveControllers);
    assert(driveControllers.getType() == XmlRpc::XmlRpcValue::TypeStruct);
    for (auto const& driveName: driveNames) {
        assert(driveControllers.hasMember(driveName));
        assert(driveControllers[driveName].getType() == XmlRpc::XmlRpcValue::TypeStruct);
        motorMultipliers[driveName] = Dimensionless{xmlRpcValueToTypeOrDefault<double>(driveControllers[driveName], "multiplier", 1.0)};
    }

    // Load rover dimensions and other parameters from the ROS parameter server
    auto roverWidth = requireParamAsUnit<Meters>(nh, "rover/width");
    auto roverLength = requireParamAsUnit<Meters>(nh, "rover/length");
    WHEEL_DISTANCE_INNER = roverWidth / 2;
    WHEEL_DISTANCE_OUTER = sqrt(((roverWidth / 2) * (roverWidth / 2)) + ((roverLength / 2) * (roverLength / 2)));

    auto ratioMotorToWheel = requireParamAsUnit<Dimensionless>(nh, "wheel/gear_ratio");
    auto wheelRadius = requireParamAsUnit<Meters>(nh, "wheel/radius");
    // TODO(quintin) is dividing by ratioMotorToWheel right?
    WHEEL_LINEAR_TO_ANGULAR = Radians{1} / wheelRadius * ratioMotorToWheel;

    auto maxLinearSpeed = requireParamAsUnit<MetersPerSecond>(nh, "rover/max_speed");
    assert(maxLinearSpeed > 0_mps);

    MAX_MOTOR_SPEED = maxLinearSpeed * WHEEL_LINEAR_TO_ANGULAR;

    driveManager = std::make_unique<MotorsGroup>(nh, "drive");

    // Subscribe to the ROS topic for drive commands
    ros::Subscriber moveDriveSubscriber = nh.subscribe<geometry_msgs::Twist>("cmd_vel", 1, moveDrive);

    // Enter the ROS event loop
    ros::spin();

    return 0;
}

void moveDrive(geometry_msgs::Twist::ConstPtr const& msg) {
    // See 11.5.1 in "Controls Engineering in the FIRST Robotics Competition" for the math
    auto forward = MetersPerSecond{msg->linear.x};
    auto turn = RadiansPerSecond{msg->angular.z};
    // TODO(quintin)    Don't ask me to explain perfectly why we need to cancel out a meters unit in the numerator
    //                  I think it comes from the fact that there is a unit vector in the denominator of the equation
    auto delta = turn * WHEEL_DISTANCE_INNER / Meters{1};
    RadiansPerSecond left = forward * WHEEL_LINEAR_TO_ANGULAR - delta;
    RadiansPerSecond right = forward * WHEEL_LINEAR_TO_ANGULAR + delta;

    std::unordered_map<std::string, RadiansPerSecond> driveCommandVelocities{
            {"FrontLeft", left},
            {"FrontRight", right},
            {"MiddleLeft", left},
            {"MiddleRight", right},
            {"BackLeft", left},
            {"BackRight", right},
    };

    for (auto [name, angularVelocity]: driveCommandVelocities) {
        // Set the desired speed for the motor
        Dimensionless multiplier = motorMultipliers[name];
        RadiansPerSecond scaledAngularVelocity = angularVelocity * multiplier; // currently in rad/s

        driveManager->getController(name).setDesiredVelocity(scaledAngularVelocity);
    }
}
