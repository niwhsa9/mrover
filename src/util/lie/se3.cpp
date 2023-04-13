#include "se3.hpp"

SE3 SE3::fromTfTree(tf2_ros::Buffer const& buffer, std::string const& fromFrameId, std::string const& toFrameId) {
    geometry_msgs::TransformStamped transform = buffer.lookupTransform(fromFrameId, toFrameId, ros::Time(0));
    return SE3::fromTf(transform.transform);
}

void SE3::pushToTfTree(tf2_ros::TransformBroadcaster& broadcaster, std::string const& childFrameId, std::string const& parentFrameId, SE3 const& tf) {
    broadcaster.sendTransform(tf.toTransformStamped(parentFrameId, childFrameId));
}

SE3::SE3(R3 const& position, SO3 const& rotation) {
    mTransform.translate(position);
    mTransform.rotate(rotation.mAngleAxis);
}

Eigen::Matrix4d SE3::matrix() const {
    return mTransform.matrix();
}

R3 SE3::position() const {
    return mTransform.translation();
}

SO3 SE3::rotation() const {
    return mTransform.rotation();
}

double SE3::distanceTo(SE3 const& other) const {
    return (position() - other.position()).norm();
}

SE3 SE3::operator*(SE3 const& other) const {
    return other.mTransform * mTransform;
}
