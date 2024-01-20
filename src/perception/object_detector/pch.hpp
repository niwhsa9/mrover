#pragma once

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstdint>
#include <execution>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <memory>
#include <numeric>
#include <optional>
#include <sstream>
#include <string>
#include <type_traits>
#include <unordered_map>

#include <boost_cpp23_workaround.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>

#include <dynamic_reconfigure/server.h>
#include <image_transport/image_transport.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/publisher.h>
#include <sensor_msgs/image_encodings.h>
#include <std_srvs/SetBool.h>
#include <tf/exceptions.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <ros/package.h>

#include <mrover/ObjectDetectorParamsConfig.h>
#include <mrover/DetectedObject.h>

#include <loop_profiler.hpp>
#include <se3.hpp>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
