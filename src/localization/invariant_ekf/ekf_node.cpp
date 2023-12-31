#include "ekf_node.hpp"

InvariantEKF InvariantEKFNode::init_EKF() {
    // set initial position to zero and initial covariant to very high number
    auto x0 = SE_2_3d::Identity();
    auto P0 = Matrix9d::Identity() * 1e6;

    // load covariance matrices from rosparam
    std::vector<double> Q_vec, R_gps_vec, R_accel_vec, R_mag_vec;
    if (!mPnh.getParam("process_noise_covariance", Q_vec)) {
        throw std::runtime_error("Failed to load process_noise_covariance from rosparam");
    }
    Matrix9d Q = Eigen::Map<Eigen::Matrix<double, 9, 9, Eigen::RowMajor>>(Q_vec.data());
    if (!mPnh.getParam("gps_covariance", R_gps_vec)) {
        throw std::runtime_error("Failed to load gps_covariance from rosparam");
    }
    Matrix3d R_gps = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_gps_vec.data());
    if (!mPnh.getParam("accel_covariance", R_accel_vec)) {
        throw std::runtime_error("Failed to load accel_covariance from rosparam");
    }
    Matrix3d R_accel = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_accel_vec.data());
    if (!mPnh.getParam("mag_covariance", R_mag_vec)) {
        throw std::runtime_error("Failed to load mag_covariance from rosparam");
    }
    Matrix3d R_mag = Eigen::Map<Eigen::Matrix<double, 3, 3, Eigen::RowMajor>>(R_mag_vec.data());

    return {x0, P0, Q, R_gps, R_accel, R_mag};
}

InvariantEKFNode::InvariantEKFNode() : mEKF(init_EKF()) {
    mLastImuTime = std::chrono::system_clock::now();
    mLastGpsTime = std::chrono::system_clock::now();

    // set up subscribers and publishers
    mImuSub = mNh.subscribe("imu", 1, &InvariantEKFNode::imu_callback, this);
    mGpsSub = mNh.subscribe("gps", 1, &InvariantEKFNode::gps_callback, this);

    mOdometryPub = mNh.advertise<nav_msgs::Odometry>("odometry", 1);
}

void InvariantEKFNode::imu_callback(const sensor_msgs::Imu& msg) {
    auto now = std::chrono::system_clock::now();
    double dt = Duration(now - mLastImuTime).count();
    mLastImuTime = now;

    Vector3d accel(msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z);
    Vector3d gyro(msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z);
    mEKF.predict(accel, gyro, dt);
}

void InvariantEKFNode::gps_callback(const geometry_msgs::Pose& msg) {
    auto now = std::chrono::system_clock::now();
    double dt = Duration(now - mLastGpsTime).count();
    mLastGpsTime = now;

    Vector3d z(msg.position.x, msg.position.y, msg.position.z);
    mEKF.update_gps(z);
}

void InvariantEKFNode::publish_odometry() {
    nav_msgs::Odometry msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "map";
    msg.child_frame_id = "base_link";

    auto x = mEKF.get_state();
    msg.pose.pose.position.x = x.translation().x();
    msg.pose.pose.position.y = x.translation().y();
    msg.pose.pose.position.z = x.translation().z();

    Eigen::Quaterniond q(x.rotation());
    msg.pose.pose.orientation.x = q.x();
    msg.pose.pose.orientation.y = q.y();
    msg.pose.pose.orientation.z = q.z();
    msg.pose.pose.orientation.w = q.w();

    // TODO: use row major data instead of manually filling each element
    auto P = mEKF.get_covariance();
    msg.pose.covariance[0] = P(0, 0);
    msg.pose.covariance[1] = P(0, 1);
    msg.pose.covariance[2] = P(0, 2);
    msg.pose.covariance[6] = P(1, 0);
    msg.pose.covariance[7] = P(1, 1);
    msg.pose.covariance[8] = P(1, 2);
    msg.pose.covariance[12] = P(2, 0);
    msg.pose.covariance[13] = P(2, 1);
    msg.pose.covariance[14] = P(2, 2);

    mOdometryPub.publish(msg);
}

void InvariantEKFNode::run() {
    ros::Rate rate(100);
    while (ros::ok()) {
        ros::spinOnce();
        publish_odometry();
        rate.sleep();
    }
}