#pragma once

#include <condition_variable>

#include <sl/Camera.hpp>

#include <image_transport/image_transport.h>
#include <image_transport/publisher.h>
#include <nodelet/nodelet.h>
#include <ros/node_handle.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <tag_detector.hpp>

namespace mrover {

    class ZedNode {
    private:
        ros::NodeHandle mNh, mPnh;

        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener;
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        image_transport::ImageTransport mIt;
        ros::Publisher mPcPub, mImuPub;
        image_transport::Publisher mLeftImgPub;

        sensor_msgs::Image mLeftImgMsg;
        sensor_msgs::PointCloud2Ptr mGrabPointCloud = boost::make_shared<sensor_msgs::PointCloud2>();
        sensor_msgs::PointCloud2Ptr mTagPointCloud = boost::make_shared<sensor_msgs::PointCloud2>();

        int mGrabTargetFps{};
        int mImageWidth{};
        int mImageHeight{};
        bool mDirectTagDetection{};

        sl::Camera mZed;
        sl::Mat mLeftImageMat;
        sl::Mat mPointCloudXYZMat;
        sl::Mat mPointCloudNormalMat;

        std::thread mTagThread;
        std::thread mGrabThread;
        bool mIsGrabDone = false;
        std::condition_variable mGrabDone;
        std::mutex mSwapPcMutex;

        boost::shared_ptr<TagDetectorNode> mTagDetectorNode;

        size_t mUpdateTick = 0;

    public:
        ZedNode(ros::NodeHandle const& nh = {}, ros::NodeHandle const& pnh = {"~"});

        ~ZedNode();

        void grabUpdate();

        void tagUpdate();
    };

    class ZedNodelet : public nodelet::Nodelet {
    public:
        ZedNodelet() = default;

        ~ZedNodelet() override = default;

    private:
        void onInit() override;

        boost::shared_ptr<ZedNode> dtl;
    };

} // namespace mrover
