#include "arm_controller.hpp"

namespace mrover {
    const ros::Duration ArmController::TIMEOUT = ros::Duration(0.5);
    
    ArmController::ArmController() {
        mIkSubscriber = mNh.subscribe("ee_pos_cmd", 1, &ArmController::ik_callback, this);
        mPositionPublisher = mNh.advertise<Position>("arm_position_cmd", 1);
        mVelSub = mNh.subscribe("ee_vel_cmd", 1, &ArmController::velCallback, this);
        mJointSub = mNh.subscribe("arm_joint_data", 1, &ArmController::fkCallback, this);
        mTimer = mNh.createTimer(ros::Duration(1.0 / 30.0), std::bind(&ArmController::timerCallback, this));
        mModeServ = mNh.advertiseService("ik_mode", &ArmController::modeCallback, this);
        mLastUpdate = ros::Time::now();
    }

    auto ArmController::ikCalc(ArmPos target) -> std::optional<Position> {
        double x = target.x;
        double y = target.y;
        double z = target.z;

        double gamma = -target.pitch;
        double x3 = x - (LINK_DE + END_EFFECTOR_LENGTH) * std::cos(gamma);
        double z3 = z - (LINK_DE + END_EFFECTOR_LENGTH) * std::sin(gamma);

        double C = std::sqrt(x3 * x3 + z3 * z3);
        double alpha = std::acos((LINK_BC * LINK_BC + LINK_CD * LINK_CD - C * C) / (2 * LINK_BC * LINK_CD));
        double beta = std::acos((LINK_BC * LINK_BC + C * C - LINK_CD * LINK_CD) / (2 * LINK_BC * C));
        double thetaA = std::atan(z3 / x3) + beta;
        double thetaB = -1 * (std::numbers::pi - alpha);
        double thetaC = gamma - thetaA - thetaB;

        double q1 = -thetaA;
        double q2 = -thetaB + JOINT_C_OFFSET;
        double q3 = -thetaC - JOINT_C_OFFSET;

        if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3) &&
            y >= JOINT_A_MIN && y <= JOINT_A_MAX &&
            q1 >= JOINT_B_MIN && q1 <= JOINT_B_MAX &&
            q2 >= JOINT_C_MIN && q2 <= JOINT_C_MAX &&
            q3 >= JOINT_DE_PITCH_MIN && q3 <= JOINT_DE_PITCH_MAX &&
            target.roll >= JOINT_DE_ROLL_MIN && target.roll <= JOINT_DE_ROLL_MAX) {
            Position positions;
            positions.names = {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"};
            positions.positions = {
                    static_cast<float>(y),
                    static_cast<float>(q1),
                    static_cast<float>(q2),
                    static_cast<float>(q3),
                    static_cast<float>(target.roll),
            };
            return positions;
        }
        return std::nullopt;

    }

    void ArmController::velCallback(geometry_msgs::Twist const& ik_vel) {
        mVelTarget = {ik_vel.linear.x, ik_vel.linear.y, ik_vel.linear.z};
        mPitchVel = ik_vel.angular.y;
        mRollVel = ik_vel.angular.x;
        if (mArmMode == ArmMode::VELOCITY_CONTROL)
            mLastUpdate = ros::Time::now();
        else
            ROS_WARN_STREAM_THROTTLE(1, "Received velocity command in position mode!");
    }

    void ArmController::fkCallback(sensor_msgs::JointState const& joint_state) {
        double y = joint_state.position[0];
        // joint b position
        double angle = -joint_state.position[1];
        double x = LINK_BC * std::cos(angle);
        double z = LINK_BC * std::sin(angle);
        // joint c position
        angle -= joint_state.position[2] - JOINT_C_OFFSET;
        x += LINK_CD * std::cos(angle);
        z += LINK_CD * std::sin(angle);
        // joint de position
        angle -= joint_state.position[3] + JOINT_C_OFFSET;
        x += (LINK_DE + END_EFFECTOR_LENGTH) * std::cos(angle);
        z += (LINK_DE + END_EFFECTOR_LENGTH) * std::sin(angle);
        mArmPos = {x, y, z, -angle, joint_state.position[4]}; // x, y, z, pitch, roll
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_fk", "arm_base_link", mArmPos.toSE3());
    }

    auto ArmController::ik_callback(IK const& ik_target) -> void {
        mPosTarget = ik_target;
        SE3d target = mPosTarget.toSE3();
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", target);
        if (mArmMode == ArmMode::POSITION_CONTROL)
            mLastUpdate = ros::Time::now();
        else
            ROS_WARN_STREAM_THROTTLE(1, "Received position command in velocity mode!");
    }

    auto ArmController::timerCallback() -> void {
        if (ros::Time::now() - mLastUpdate > TIMEOUT) {
            ROS_WARN_STREAM_THROTTLE(1, "IK Timed Out");
            return;
        }

        ArmPos target;
        if (mArmMode == ArmMode::POSITION_CONTROL) {
            target = mPosTarget;
        } else {
            // every second, this increment will happen 30 times (in theory)
            // so the max velocity is 30 times the constant (in m/s theoretically)
            // note that |mVelTarget| <= 1
            target = mPosTarget + mVelTarget * 0.01;
            target.pitch += mPitchVel * 0.05;
            target.roll += mRollVel * 0.05;
        }
        SE3Conversions::pushToTfTree(mTfBroadcaster, "arm_target", "arm_base_link", target.toSE3());
        auto positions = ikCalc(target);
        if (positions) {
            mPositionPublisher.publish(positions.value());
            // if successful, move the target to the new spot (relevant for velocity control)
            mPosTarget = target;
        } else {
            ROS_WARN_STREAM_THROTTLE(1, "IK Failed");
			// if IK failed in velocity mode, we go back to where the target was pre-increment (this should always be a reachable
			// position because mPosTarget only gets updated in velocity mode if the position is reachable)
			// this should hopefully get us to the (approximately) closest point that is reachable
			if (mArmMode == ArmMode::VELOCITY_CONTROL) {
				positions = ikCalc(mPosTarget);
				if (positions) {
					mPositionPublisher.publish(positions.value());
				} else { // surely this will never happen (because we mPosTarget should always be reachable)
					ROS_WARN_STREAM_THROTTLE(1, "Velocity control closest point failed");
				}
			}
        }
    }

    auto ArmController::modeCallback(IkMode::Request & req, IkMode::Response & resp) -> bool {
        if (req.mode == IkMode::Request::POSITION_CONTROL) {
            ROS_INFO("IK Position Control Mode");
            mArmMode = ArmMode::POSITION_CONTROL;
        } else {
            ROS_INFO("IK Velocity Control Mode");
            mArmMode = ArmMode::VELOCITY_CONTROL;
            // when we switch to velocity control mode, set the target to the current position
            mPosTarget = mArmPos;
        }
        return resp.success = true;
    }
} // namespace mrover

auto main(int argc, char** argv) -> int {
    ros::init(argc, argv, "arm_controller");

    mrover::ArmController armController;

    ros::spin();
}