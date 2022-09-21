#include "aruco_detect.hpp"
 
#include "filter.hpp"
 
// TODO: add cache logic to filter out false positives
 
/**
* Detect fiducials from raw image using OpenCV and calculate their screen space centers.
* Maintain the immediate buffer - holds all fiducials seen currently on screen.
* Later camera space pose information is filled in when we receive point cloud data.
*
* @param msg
*/
void FiducialsNode::imageCallback(sensor_msgs::ImageConstPtr const& msg) {
    if (!mEnableDetections) return;
 
    if (mIsVerbose) {
        ROS_INFO("Got image %d", msg->header.seq);
    }
 
    try {
        ROS_INFO("here");
        mCvPtr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
 
        // Detect the fiducial vertices in screen space and their respective ids
        cv::aruco::detectMarkers(mCvPtr->image, mDictionary, mCorners, mIds, mDetectorParams);
 
        // Remove fiducials in the immediate buffer that are no longer in sight
        //or increment times they've been seen
        auto it = mImmediateFiducials.begin();
        while (it != mImmediateFiducials.end()) {
            if (std::find(mIds.begin(), mIds.end(), it->first) == mIds.end()) {
                it = mImmediateFiducials.erase(it);
            }
        }

        ROS_INFO("%ld", mIds.size());

        // Save fiducials currently on screen to the immediate buffer
        for (size_t i = 0; i < mIds.size(); ++i) {
            int id = mIds[i];
            cv::Point2f imageCenter = std::accumulate(mCorners[i].begin(), mCorners[i].end(), cv::Point2f{}) / 4.0f;
            ImmediateFiducial& immediateFid = mImmediateFiducials[id];
            immediateFid.id = id;
            immediateFid.imageCenter = imageCenter;
        }

        ROS_INFO("%ld", mImmediateFiducials.size());
        // Increment Intermediate Fiducials timesseen and update filters
        // Add readings to the persistent representations of the fiducials if they exist otherwise
        for (auto [id, immediateFid]: mImmediateFiducials) {
            ROS_INFO("something in immediate"); 
            IntermediateFiducial &intermediate_fid = mIntermediateFiducials[id]; 
            PersistentFiducial& persistent_fid = mPersistentFiducials[id];

            if (!immediateFid.fidInCam.has_value()) continue;
            // This is set if the point cloud had no valid reading for this fiducial

            std::string immediateFrameName = "immediateFiducial" + std::to_string(id);
            SE3::pushToTfTree(mTfBroadcaster, immediateFrameName, ROVER_FRAME, immediateFid.fidInCam.value());

            if(intermediate_fid.timesSeen < mMinTimesSeen) { //has not been seen enough times to be persisisent
                ROS_INFO("times seen is less than min times seen");  
                intermediate_fid.id = id; 
                ++intermediate_fid.timesSeen; 

                try {
                    SE3 fidInOdom = SE3::fromTfTree(mTfBuffer, ODOM_FRAME, immediateFrameName);
                    intermediate_fid.fidInOdomXYZ.setFilterParams(mFilterCount, mFilterProportion);
                    intermediate_fid.fidInOdomXYZ.addReading(fidInOdom);
                } catch (tf2::ExtrapolationException const&) {
                    ROS_WARN("Old data for immediate fiducial");
                } catch (tf2::LookupException const&) {
                    ROS_WARN("No transform from immediate fiducial to odom");
                }

                if(intermediate_fid.timesSeen == mMinTimesSeen) // graduate to persistent fiducial 
                {
                    persistent_fid.id = id; 
                    persistent_fid.fidInOdomXYZ = intermediate_fid.fidInOdomXYZ; 
                }                
            } 
            else { // already a persistent fiducial
                ROS_INFO("already persistent"); 
                persistent_fid.id = id;
                try {
                    SE3 fidInOdom = SE3::fromTfTree(mTfBuffer, ODOM_FRAME, immediateFrameName);
                    persistent_fid.fidInOdomXYZ.setFilterParams(mFilterCount, mFilterProportion);
                    persistent_fid.fidInOdomXYZ.addReading(fidInOdom);
                } catch (tf2::ExtrapolationException const&) {
                    ROS_WARN("Old data for immediate fiducial");
                } catch (tf2::LookupException const&) {
                    ROS_WARN("No transform from immediate fiducial to odom");
                }
            }
        }
 
        // Send all transforms of persistent fiducials
        for (auto [id, fid]: mPersistentFiducials) {
            if (!fid.fidInOdomXYZ.ready()) continue; // Wait until the filters have enough readings to become meaningful
 
            SE3::pushToTfTree(mTfBroadcaster, "fiducial" + std::to_string(id), ODOM_FRAME, fid.fidInOdomXYZ.getFidInOdom());
        }

        size_t detectedCount = mIds.size();
        if (mIsVerbose || !mPrevDetectedCount.has_value() || detectedCount != mPrevDetectedCount.value()) {
            mPrevDetectedCount = detectedCount;
            ROS_INFO("Detected %zu markers", detectedCount);
        }
 
        if (!mImmediateFiducials.empty()) {
            cv::aruco::drawDetectedMarkers(mCvPtr->image, mCorners, mIds);
        }
 
        if (mPublishImages) {
           mImgPub.publish(mCvPtr->toImageMsg());
        }
 
        mSeqNum++;
    } catch (cv_bridge::Exception const& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
    } catch (cv::Exception const& e) {
        ROS_ERROR("cv exception: %s", e.what());
    }
}
 
/**
* @brief       Retrieve the pose of the fiducial in camera space
* @param msg   3D Point Cloud with points stored relative to the camera
* @param u     X Pixel Position
* @param v     Y Pixel Position
*/
std::optional<SE3> getFidInCamFromPixel(sensor_msgs::PointCloud2ConstPtr const& msg, size_t u, size_t v) {
    // If PCL ends up being needed, use its camToPoint function instead
    size_t arrayPos = v * msg->row_step + u * msg->point_step;
    size_t arrayPosY = arrayPos + msg->fields[0].offset;
    size_t arrayPosZ = arrayPos + msg->fields[1].offset;
    size_t arrayPosX = arrayPos + msg->fields[2].offset;

    cv::Point3f point;
    std::memcpy(&point.x, &msg->data[arrayPosX], sizeof(point.x));
    std::memcpy(&point.y, &msg->data[arrayPosY], sizeof(point.y));
    std::memcpy(&point.z, &msg->data[arrayPosZ], sizeof(point.z));

    return (std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z))
            ? std::optional<SE3>(SE3({+point.x, -point.y, +point.z}, Eigen::Quaterniond::Identity()))
            : std::nullopt;
}
 
/**
* For each active fiducial image we have detected so far, fuse point cloud information.
* This information is where it is in the world.
*
* @param msg   Point cloud message
*/
void FiducialsNode::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) {
    for (auto& [id, fid]: mImmediateFiducials) {
        size_t u = std::lround(fid.imageCenter.x);
        size_t v = std::lround(fid.imageCenter.y);
 
        try {
            fid.fidInCam = getFidInCamFromPixel(msg, u, v);
        } catch (tf2::TransformException& ex) {
            ROS_WARN("Transform lookup error: %s", ex.what());
        }
    }
}