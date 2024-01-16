#pragma once

#include <charconv>
#include <chrono>
#include <execution>
#include <filesystem>
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <source_location>
#include <stdexcept>
#include <thread>
#include <unordered_set>
#include <span>

#include <boost_cpp23_workaround.hpp>

#include <boost/circular_buffer.hpp>
#include <boost/container/static_vector.hpp>
// #include <boost/process.hpp>
#include <boost/range/combine.hpp>
#include <boost/thread.hpp>
#include <boost/thread/future.hpp>

#include <fmt/core.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nodelet/loader.h>
#include <nodelet/nodelet.h>
#include <ros/package.h>
#include <ros/serialization.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <urdf/model.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <glfw3webgpu.h>

#include "webgpu/webgpu.hpp"

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "imgui/backends/imgui_impl_glfw.h"
#include "imgui/backends/imgui_impl_wgpu.h"

#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBodyGearConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointLimitConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyJointMotor.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/Featherstone/btMultiBodyMLCPConstraintSolver.h>
#include <BulletDynamics/MLCPSolvers/btDantzigSolver.h>
#include <BulletDynamics/MLCPSolvers/btMLCPSolver.h>
#include <btBulletDynamicsCommon.h>
// #include <BulletDynamics/Featherstone/btMultiBodyJointFeedback.h>

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <loop_profiler.hpp>
#include <params_utils.hpp>
#include <point.hpp>
#include <se3.hpp>
#include <units.hpp>

#include <mrover/IK.h>
#include <mrover/Position.h>
