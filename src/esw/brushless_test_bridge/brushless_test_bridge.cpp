#include <ros/rate.h>
#include <ros/ros.h>

#include <iostream>
#include <motors_group.hpp>
#include <units/units.hpp>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "brushless_test_bridge");
    ros::NodeHandle nh;
    ROS_INFO("Running");

    // To get it to run:
    //  On laptop:
    // - ./fdcanusb_daemon -F -v /dev/fdcanusb vcan0
    // - sudo ip link set vcan0 up
    //  On Jetson:
    // - roscore
    // - sudo ip link set can0 up type can bitrate 500000 dbitrate 1000000 berr-reporting on fd on restart-ms 100
    // - rosparam load config/esw_devboard.yaml
    // - rosrun mrover can_driver_node _interface:=can0
    // - roslaunch brushless_test.launch

    // auto brushlessController_de0 = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_0");
    // auto brushlessController_de1 = std::make_unique<mrover::BrushlessController>(nh, "jetson", "joint_de_1");
    
    // fake DE publisher:

    // std::unique_ptr<ros::Publisher> DEPub;
    auto DEPub = std::make_unique<ros::Publisher>(nh.advertise<mrover::Velocity>("arm_velocity_cmd", 1));


    mrover::Velocity msg;
    msg.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll", "allen_key", "gripper"};
    msg.velocities = {0.2, 0, 0, 0, 0, 0, 0};
    
    // ros::Rate rate{102};

    /*
    // Different positions test
    std::array<float, 4> positions = {1.0, 2.0, 3.0, 4.0};
    while (ros::ok()) {
        // Throttle test
        //brushlessController->setDesiredThrottle(mrover::Percent{((float) count) / 500.0});
        brushlessController->setDesiredVelocity(mrover::RadiansPerSecond{5.0});
        // brushlessController->setDesiredPosition(mrover::Radians{positions.at(count / 400)});
        count++;
        ros::spinOnce();
        rate.sleep();
    }

    */
    // brushlessController_de0->setStop();
    // brushlessController_de1->setStop();
    ros::Rate rate{10};

    int count = 0;

    while(ros::ok()){
        // publish DE velocity:
        DEPub->publish(msg);

        count++;

        if(count > 50) {
            msg.velocities[0] *= -1;
            count = 0;
        }   

        ros::spinOnce();
        rate.sleep();
    }

    return EXIT_SUCCESS;
}

