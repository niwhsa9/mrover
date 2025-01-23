#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController {

        [[maybe_unused]] ros::Subscriber mIkSubscriber;
        [[maybe_unused]] ros::Subscriber mVelSub;
        [[maybe_unused]] ros::Subscriber mJointSub;

        ros::NodeHandle mNh;
        ros::Publisher mPositionPublisher;
        tf2_ros::TransformBroadcaster mTfBroadcaster;
        tf2_ros::Buffer mTfBuffer{};
        tf2_ros::TransformListener mTfListener{mTfBuffer};
        ros::Timer mTimer;
        ros::ServiceServer mModeServ;

        auto ikCalc(SE3d target) -> std::optional<Position>;
        auto timerCallback() -> void;
        auto modeCallback(IkMode::Request& req, IkMode::Response& res) -> bool;

        SE3d mArmPos;
        SE3d mPosTarget;
        R3d mVelTarget = {0, 0, 0};
        ros::Time mLastUpdate;
        
        enum class ArmMode : bool {
            VELOCITY_CONTROL,
            POSITION_CONTROL
        };
        ArmMode mArmMode = ArmMode::POSITION_CONTROL;
        static const ros::Duration TIMEOUT;
    public:
        // From: rover.urdf.xacro
        // A is the prismatic joint, B is the first revolute joint, C is the second revolute joint
        static constexpr double LINK_BC = 0.5344417294;
        static constexpr double LINK_CD = 0.5531735368;
        static constexpr double LINK_DE = 0.044886000454425812;
        static constexpr double JOINT_A_MIN = 0;
        static constexpr double JOINT_A_MAX = 0.4;
        static constexpr double JOINT_B_MIN = -0.86;
        static constexpr double JOINT_B_MAX = 0.1;
        static constexpr double JOINT_C_MIN = 1;
        static constexpr double JOINT_C_MAX = 2.69;
        static constexpr double JOINT_DE_PITCH_MIN = -0.7;
        static constexpr double JOINT_DE_PITCH_MAX = 0.87;
        static constexpr double END_EFFECTOR_LENGTH = 0.13; // measured from blender
        static constexpr double JOINT_C_OFFSET = 0.1608485915;

        ArmController();

        void ik_callback(IK const& new_ik_target);
        void velCallback(geometry_msgs::Vector3 const& ik_vel);
        void fkCallback(sensor_msgs::JointState const& joint_state);
    };

} // namespace mrover