#include "hardware_h265_encoder.hpp"

#include <ros/init.h>
#include <ros/node_handle.h>
#include <ros/subscriber.h>

#include <sensor_msgs/Image.h>

#include <streaming.hpp>

#include <opencv2/imgproc.hpp>

std::optional<StreamServer> streamServer;

std::optional<Encoder> encoder;

auto imageCallback(sensor_msgs::ImageConstPtr const& msg) -> void {
    try {
        cv::Size size{static_cast<int>(msg->width), static_cast<int>(msg->height)};
        if (!encoder) encoder.emplace(size);

        cv::Mat bgraFrame{size, CV_8UC4, const_cast<std::uint8_t*>(msg->data.data()), msg->step};
        cv::Mat i420Frame;
        cvtColor(bgraFrame, i420Frame, cv::COLOR_BGRA2YUV_I420);

        Encoder::BitstreamView view = encoder->feed(i420Frame);
        std::span span{static_cast<std::byte*>(view.lockParams.bitstreamBufferPtr), view.lockParams.bitstreamSizeInBytes};
        streamServer->feed(span);

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception encoding frame: {}", e.what()));
        ros::requestShutdown();
    }
}

auto main(int argc, char** argv) -> int {
    try {
        ros::init(argc, argv, "software_h265_encoder");
        ros::NodeHandle nh;

        streamServer.emplace("0.0.0.0", 8080);

        ros::Subscriber imageSubscriber = nh.subscribe("/camera/left/image", 1, imageCallback);

        ros::spin();
        return EXIT_SUCCESS;

    } catch (std::exception const& e) {
        ROS_ERROR_STREAM(std::format("Exception initializing: {}", e.what()));
        return EXIT_FAILURE;
    }
}