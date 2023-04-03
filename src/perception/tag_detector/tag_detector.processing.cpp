#include "tag_detector.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <execution>
#include <numeric>

#include <sensor_msgs/image_encodings.h>

namespace mrover {

    struct Point {
        float x, y, z;
        uint8_t r, g, b, a;
        float normal_x, normal_y, normal_z;
        float curvature;
    } __attribute__((packed));

    /**
     * @brief       Retrieve the pose of the tag in camera space
     * @param msg   3D Point Cloud with points stored relative to the camera
     * @param u     X Pixel Position
     * @param v     Y Pixel Position
     */
    std::optional<SE3> TagDetectorNodelet::getTagInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& cloudPtr, size_t u, size_t v) {
        if (u >= cloudPtr->width || v >= cloudPtr->height) {
            NODELET_WARN("Tag center out of bounds: [%zu %zu]", u, v);
            return std::nullopt;
        }

        Point point = reinterpret_cast<Point const*>(cloudPtr->data.data())[u + v * cloudPtr->width];

        if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z)) {
            NODELET_WARN("Tag center point not finite: [%f %f %f]", point.x, point.y, point.z);
            return std::nullopt;
        }

        return std::make_optional<SE3>(R3{point.x, point.y, point.z}, SO3{});
    }

    /**
     * For each tag we have detected so far, fuse point cloud information.
     * This information is where it is in the world.
     *
     * @param msg   Point cloud message
     */
    void TagDetectorNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
        mProfiler.reset();

        if (!mEnableDetections) return;

        NODELET_DEBUG("Got point cloud %d", msg->header.seq);

        if (msg->height == 0 || msg->width == 0) {
            NODELET_WARN("Point cloud has zero size");
            return;
        }

        if (static_cast<int>(msg->height) != mImg.rows || static_cast<int>(msg->width) != mImg.cols) {
            NODELET_INFO("Image size changed from [%d %d] to [%u %u]", mImg.cols, mImg.rows, msg->width, msg->height);
            mImg = cv::Mat{static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC3, cv::Scalar{0, 0, 0}};
        }

        auto* pixelPtr = reinterpret_cast<cv::Vec3b*>(mImg.data);
        auto* pointPtr = reinterpret_cast<Point const*>(msg->data.data());
        std::for_each(std::execution::par_unseq, pixelPtr, pixelPtr + mImg.total(), [&](cv::Vec3b& pixel) {
            size_t i = &pixel - pixelPtr;
            pixel[0] = pointPtr[i].r;
            pixel[1] = pointPtr[i].g;
            pixel[2] = pointPtr[i].b;
        });
        mProfiler.addEpoch("Convert");

        // Detect the tag vertices in screen space and their respective ids
        // {mCorners, mIds} are the outputs from OpenCV
        cv::aruco::detectMarkers(mImg, mDictionary, mCorners, mIds, mDetectorParams);
        NODELET_DEBUG("OpenCV detect size: %zu", mIds.size());
        mProfiler.addEpoch("OpenCV Detect");

        // Update ID, image center, and increment hit count for all detected tags
        for (size_t i = 0; i < mIds.size(); ++i) {
            int id = mIds[i];
            Tag& tag = mTags[id];
            tag.hitCount = std::clamp(tag.hitCount + 1, 0, mMaxHitCount);
            tag.id = id;
            tag.imageCenter = std::accumulate(mCorners[i].begin(), mCorners[i].end(), cv::Point2f{}) / static_cast<float>(mCorners[i].size());
            tag.tagInCam = getTagInCamFromPixel(msg, std::lround(tag.imageCenter.x), std::lround(tag.imageCenter.y));

            if (tag.tagInCam) {
                // Publish tag to immediate
                std::string immediateFrameId = "immediateFiducial" + std::to_string(tag.id);
                SE3::pushToTfTree(mTfBroadcaster, immediateFrameId, mCameraFrameId, tag.tagInCam.value());
            }
        }

        // Handle tags that were not seen this update
        // Decrement their hit count and remove if they hit zero
        auto it = mTags.begin();
        while (it != mTags.end()) {
            auto& [id, tag] = *it;
            if (std::find(mIds.begin(), mIds.end(), id) == mIds.end()) {
                tag.hitCount--;
                if (tag.hitCount <= 0) {
                    it = mTags.erase(it);
                    continue;
                }
            }
            ++it;
        }

        // Publish all tags to the tf tree that have been seen enough times
        for (auto const& [id, tag]: mTags) {
            if (tag.hitCount >= mMinHitCountBeforePublish) {
                if (tag.tagInCam) {
                    try {
                        std::string immediateFrameId = "immediateFiducial" + std::to_string(tag.id);
                        // Publish tag to odom
                        std::string const& parentFrameId = mUseOdom ? mOdomFrameId : mMapFrameId;
                        SE3 tagInParent = SE3::fromTfTree(mTfBuffer, parentFrameId, immediateFrameId);
                        SE3::pushToTfTree(mTfBroadcaster, "fiducial" + std::to_string(id), parentFrameId, tagInParent);
                    } catch (tf2::ExtrapolationException const&) {
                        NODELET_WARN("Old data for immediate tag");
                    } catch (tf2::LookupException const&) {
                        NODELET_WARN("Expected transform for immediate tag");
                    }
                } else {
                    NODELET_DEBUG("Had tag detection but no corresponding point cloud information");
                }
            }
        }

        if (mPublishImages && mImgPub.getNumSubscribers()) {
            cv::aruco::drawDetectedMarkers(mImg, mCorners, mIds);
            mImgMsg.header.seq = mSeqNum;
            mImgMsg.header.stamp = ros::Time::now();
            mImgMsg.header.frame_id = "zed2i_left_camera_frame";
            mImgMsg.height = mImg.rows;
            mImgMsg.width = mImg.cols;
            mImgMsg.encoding = sensor_msgs::image_encodings::BGR8;
            mImgMsg.step = mImg.step;
            mImgMsg.is_bigendian = __BYTE_ORDER__ == __ORDER_BIG_ENDIAN__;
            size_t size = mImgMsg.step * mImgMsg.height;
            mImgMsg.data.resize(size);
            std::copy(std::execution::par_unseq, mImg.data, mImg.data + size, mImgMsg.data.begin());
            mImgPub.publish(mImgMsg);
        }

        size_t detectedCount = mIds.size();
        if (!mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value()) {
            mPrevDetectedCount = detectedCount;
            NODELET_INFO("Detected %zu markers", detectedCount);
        }

        mProfiler.addEpoch("Publish");

        mSeqNum++;
    }

} // namespace mrover
