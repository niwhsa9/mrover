#pragma once

#include "pch.hpp"

namespace mrover {

    class ArmController {
        struct ArmPos {
            double x{0}, y{0}, z{0}, pitch{0}, roll{0};
            auto toSE3() const -> SE3d {
                return SE3d{{x, y, z,}, SO3d{Eigen::Quaterniond{Eigen::AngleAxisd{pitch, R3d::UnitY()} * Eigen::AngleAxisd{roll, R3d::UnitX()}}}};
            }

            auto operator+(R3d offset) const -> ArmPos {
                return {x + offset.x(), y + offset.y(), z + offset.z(), pitch, roll};
            }

            auto operator=(IK ik_target) -> ArmPos& {
                x = ik_target.pos[0];
                y = ik_target.pos[1];
                z = ik_target.pos[2];
                pitch = ik_target.pitch;
                roll = ik_target.roll;
                return *this;
            }
        };

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

        auto ikCalc(ArmPos target) -> std::optional<Position>;
        auto timerCallback() -> void;

        ArmPos mArmPos;
        ArmPos mPosTarget = {0, -1, 0}; // THIS IS SCUFFED - set y target to -1 to make sure this is position is not reachable
        R3d mVelTarget = {0, 0, 0};
        // angular velocity target speeds
        double mPitchVel = 0.f;
        double mRollVel = 0.f;
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
        static constexpr double JOINT_DE_ROLL_MIN = -2.36;
        static constexpr double JOINT_DE_ROLL_MAX = 1.44;
        static constexpr double END_EFFECTOR_LENGTH = 0.13; // measured from blender
        static constexpr double JOINT_C_OFFSET = 0.1608485915;

        ArmController();

        void ik_callback(IK const& new_ik_target);
        void velCallback(geometry_msgs::Twist const& ik_vel);
        void fkCallback(sensor_msgs::JointState const& joint_state);
        auto modeCallback(IkMode::Request& req, IkMode::Response& res) -> bool;
    };

} // namespace mrover