#include <ros/ros.h>
#include <std_srvs/SetBool.h>
#include <vector>
#include <string>

int main(int argc, char** argv) {
    ros::init(argc, argv, "science_test_bridge");
    ros::NodeHandle nh;

    // List of heater names
    std::vector<std::string> heater_names = {"heater_b0", "heater_n0", "heater_b1", "heater_n1", "heater_b2", "heater_n2"};

    // Create a service message
    std_srvs::SetBool srv;

    // Iterate through each heater name and call the service
    for (const auto& heater_name : heater_names) {
        // Create a ROS service client for the current heater
        ros::ServiceClient client = nh.serviceClient<std_srvs::SetBool>("science_enable_" + heater_name);

        // Wait for the service to become available
        ros::service::waitForService("science_enable_" + heater_name);

        // Turn the heater on
        srv.request.data = true;
        if (client.call(srv)) {
            ROS_INFO_STREAM(heater_name << " CAN message sent to turn on. \n\nNow waiting 3 seconds. \n" << heater_name << " should automatically turn off after 1 second in watchdog.\n");
        } else {
            ROS_ERROR_STREAM("Failed to call service to turn " << heater_name << " on");
            return 1;
        }

        ros::Duration(1.0).sleep();

        ROS_INFO_STREAM(heater_name << " should have turned off!! \n");

        // Wait for a few seconds
        ros::Duration(3.0).sleep();

        // Turn the heater off
        srv.request.data = false;
        if (client.call(srv)) {
            ROS_INFO_STREAM(heater_name << " CAN message sent to turn off");
        } else {
            ROS_ERROR_STREAM("Failed to call service to turn " << heater_name << " off");
            return 1;
        }

        ROS_INFO_STREAM("Waiting 2 seconds before starting again. \n");
        ros::Duration(2.0).sleep();
    }

    return 0;
}
