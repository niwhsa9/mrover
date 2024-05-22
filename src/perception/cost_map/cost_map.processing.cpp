#include "cost_map.hpp"

namespace mrover {

    auto remap(double x, double inMin, double inMax, double outMin, double outMax) -> double {
        return (x - inMin) * (outMax - outMin) / (inMax - inMin) + outMin;
    }

    auto CostMapNodelet::pointCloudCallback(sensor_msgs::PointCloud2ConstPtr const& msg) -> void {
        assert(msg);
        assert(msg->height > 0);
        assert(msg->width > 0);

        if (!mPublishCostMap) return;

        std::uint32_t maxNumPoints = msg->height * msg->width / mDownSamplingFactor / mDownSamplingFactor;
        mPointsInMap.clear();
        mPointsInMap.reserve(maxNumPoints);

        try {
            SE3f cameraToMap = SE3Conversions::fromTfTree(mTfBuffer, "zed_left_camera_frame", "map").cast<float>();
            auto* points = reinterpret_cast<Point const*>(msg->data.data());

            // std::vector<Eigen::Index> rowIndices;
            // rowIndices.resize(msg->width / mDownSamplingFactor);
            // std::ranges::generate(rowIndices, [&, c = 0]() mutable { return c++; });
            //
            // for (Eigen::Index r = 0; r < msg->height / mDownSamplingFactor; r++) {
            //     std::for_each(std::execution::par_unseq, rowIndices.begin(), rowIndices.end(), [&](Eigen::Index c) {
            //         auto* point = points + r * mDownSamplingFactor * msg->width + c * mDownSamplingFactor;
            //         R3f pointInCamera{point->x, point->y, point->z};
            //         R3f normalInCamera{point->normal_x, point->normal_y, point->normal_z};
            //
            //         Eigen::Index i = r * msg->width / mDownSamplingFactor + c;
            //         mPointsInMap.col(i) = cameraToMap.act(pointInCamera);
            //         mNormalsInMap.col(i) = cameraToMap.asSO3().act(normalInCamera); // Normal is a direction as should not be affected by translation
            //     });
            // }

            for (Eigen::Index r = 0; r < msg->height; r += mDownSamplingFactor) {
                for (Eigen::Index c = 0; c < msg->width; c += mDownSamplingFactor) {
                    auto* point = points + r * msg->width + c;

                    // Points with no stereo correspondence are NaN's, so ignore them
                    if (!std::isfinite(point->x) || !std::isfinite(point->y) || !std::isfinite(point->z)) continue;
                    if (!std::isfinite(point->normal_x) || !std::isfinite(point->normal_y) || !std::isfinite(point->normal_z)) continue;
                    if (point->normal_z < -std::numeric_limits<float>::epsilon()) continue;

                    R3f pointInCamera{point->x, point->y, point->z};

                    if (double distSq = pointInCamera.squaredNorm(); distSq < 1 * 1 || distSq > 8 * 8) continue;

                    R3f normalInCamera{point->normal_x, point->normal_y, point->normal_z};
                    normalInCamera.normalize();

                    // Normal is a direction as should not be affected by translation
                    mPointsInMap.emplace_back(cameraToMap.act(pointInCamera), cameraToMap.asSO3().act(normalInCamera));
                }
            }

            std::vector<std::vector<double>> binAngles;
            binAngles.resize(mGlobalGridMsg.data.size());

            for (auto const& [pointInMap, normalInMap]: mPointsInMap) {
                double xInMap = pointInMap.x();
                double yInMap = pointInMap.y();

                // if (xInMap < mGlobalGridMsg.info.origin.position.x || xInMap > mGlobalGridMsg.info.origin.position.x + mDimension ||
                // yInMap < mGlobalGridMsg.info.origin.position.y || yInMap > mGlobalGridMsg.info.origin.position.y + mDimension) continue;

                auto xIndex = static_cast<int>(std::lround((xInMap - mGlobalGridMsg.info.origin.position.x) / mGlobalGridMsg.info.resolution));
                auto yIndex = static_cast<int>(std::lround((yInMap - mGlobalGridMsg.info.origin.position.y) / mGlobalGridMsg.info.resolution));
                int costMapIndex = static_cast<int>(mGlobalGridMsg.info.width) * yIndex + xIndex;

                if (costMapIndex < 0 || costMapIndex >= static_cast<int>(mGlobalGridMsg.data.size())) continue;

                double angleToUp = std::acos(normalInMap.z());

                binAngles[costMapIndex].push_back(angleToUp);

                // auto cost = static_cast<std::int8_t>(std::lround(remap(angleToUp, 0, TAU / 4, FREE_COST, OCCUPIED_COST)));
                //
                // constexpr double alpha = 0.1;
                // mGlobalGridMsg.data[costMapIndex] = static_cast<std::int8_t>(alpha * cost + (1 - alpha) * mGlobalGridMsg.data[costMapIndex]);
            }

            for (std::size_t i = 0; i < mGlobalGridMsg.data.size(); i++) {
                auto& bin = binAngles[i];

                if (bin.empty()) continue;

                std::ranges::sort(bin);
                // take mean of inner 80% of the angles
                auto start = bin.begin() + bin.size() / 10;
                auto end = bin.end() - bin.size() / 10;
                double mean = std::accumulate(start, end, 0.0) / std::distance(start, end);
                auto newCost = static_cast<std::int8_t>(std::lround(remap(mean, 0, TAU / 4, FREE_COST, OCCUPIED_COST)));

                constexpr double alpha = 0.1;
                auto& cost = mGlobalGridMsg.data[i];
                cost = static_cast<std::int8_t>(alpha * newCost + (1 - alpha) * cost);
            }

            mCostMapPub.publish(mGlobalGridMsg);
        } catch (tf2::TransformException const& e) {
            ROS_WARN_STREAM_THROTTLE(1, std::format("TF tree error processing point cloud: {}", e.what()));
        }
    }

} // namespace mrover