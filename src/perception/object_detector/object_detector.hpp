#pragma once

#include "inference_wrapper.hpp"
#include <opencv2/core/mat.hpp>
#include <ros/publisher.h>
#include "pch.hpp"

namespace mrover {

    //Data type for detection
    struct Detection {
        int class_id{0};
        std::string className;
        float confidence{0.0};
        cv::Rect box{};
    };

    class ObjectDetectorNodelet : public nodelet::Nodelet {

        //Mat for the image from the point cloud
        cv::Mat mImg;

        //List of class names
        std::vector<std::string> classes{"Hammer"};//{"person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck", "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup", "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch", "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse", "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"};

        //ROS Handlers
        ros::NodeHandle mNh, mPnh;

        //Inference inference;
        InferenceWrapper mInferenceWrapper;

        // Publishers
        ros::Publisher mDebugImgPub;

        // Subscribers
        ros::Subscriber mImgSub;

        // Preallocated cv::Mats
        cv::Mat mImageBlob;

        dynamic_reconfigure::Server<ObjectDetectorParamsConfig> mConfigServer;
        dynamic_reconfigure::Server<ObjectDetectorParamsConfig>::CallbackType mCallbackType;

        // Internal state
        cv::dnn::Net mNet;

        // Debug
        LoopProfiler mProfiler{"Object Detector"};

        //TF Member Variables
        tf2_ros::Buffer mTfBuffer;
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        std::string mCameraFrameId;
        std::string mMapFrameId;

        //Hit counter
        int mHitCount;

        //Constants after initialization
        int mObjIncrementWeight;
        int mObjDecrementWeight;
        int mObjHitThreshold;
        int mObjMaxHitcount;

        //Init method
        void onInit() override;

        //Function to get SE3 from the point cloud
        std::optional<SE3> getObjectInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v, size_t width, size_t height);

        std::optional<SE3> spiralSearchInImg(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t xCenter, size_t yCenter, size_t width, size_t height);
    
    public:
        //Node constructor
        ObjectDetectorNodelet() = default;

        //Node Deconstructor
        ~ObjectDetectorNodelet() override = default;

        //Callback for when ZED publishes data
        void imageCallback(sensor_msgs::PointCloud2ConstPtr const& msg);
    };
} // namespace mrover