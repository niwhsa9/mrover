#include "arm_translator.hpp"

#include <ros/duration.h>

#include <params_utils.hpp>
#include <units/units.hpp>
#include <units/units_eigen.hpp>

namespace mrover {

    // Maps pitch and roll values to the DE0 and DE1 motors outputs
    // For example when only pitching the motor, both controllers should be moving in the same direction
    // When rolling, the controllers should move in opposite directions
    auto static const PITCH_ROLL_TO_0_1 = (Matrix2<Dimensionless>{} << -1, -1, -1, 1).finished();
    Dimensionless static constexpr PITCH_ROLL_TO_01_SCALE{40};

    // How often we send an adjust command to the DE motors
    // This corrects the HALL-effect motor source on the Moteus based on the absolute encoder readings
    double static constexpr DE_OFFSET_TIMER_PERIOD = 1;

    ArmTranslator::ArmTranslator(ros::NodeHandle& nh) {
        for (std::string const& hwName: mArmHWNames) {
            [[maybe_unused]] auto [_, was_inserted] = mAdjustClientsByArmHwNames.try_emplace(hwName, nh.serviceClient<AdjustMotor>(std::format("{}_adjust", hwName)));
            assert(was_inserted);
        }

        mThrottleSub = nh.subscribe<Throttle>("arm_throttle_cmd", 1, &ArmTranslator::processThrottleCmd, this);
        mVelocitySub = nh.subscribe<Velocity>("arm_velocity_cmd", 1, &ArmTranslator::processVelocityCmd, this);
        mPositionSub = nh.subscribe<Position>("arm_position_cmd", 1, &ArmTranslator::processPositionCmd, this);
        mJointDataSub = nh.subscribe<sensor_msgs::JointState>("arm_direct_joint_data", 1, &ArmTranslator::processJointState, this);

        mThrottlePub = std::make_unique<ros::Publisher>(nh.advertise<Throttle>("arm_direct_throttle_cmd", 1));
        mVelocityPub = std::make_unique<ros::Publisher>(nh.advertise<Velocity>("arm_direct_velocity_cmd", 1));
        mPositionPub = std::make_unique<ros::Publisher>(nh.advertise<Position>("arm_direct_position_cmd", 1));
        mJointDataPub = std::make_unique<ros::Publisher>(nh.advertise<sensor_msgs::JointState>("arm_joint_data", 1));

        mDeOffsetTimer = nh.createTimer(ros::Duration{DE_OFFSET_TIMER_PERIOD}, &ArmTranslator::updateDeOffsets, this);
    }

    auto findJointByName(std::vector<std::string> const& names, std::string const& name) -> std::optional<std::size_t> {
        auto it = std::ranges::find(names, name);
        return it == names.end() ? std::nullopt : std::make_optional(std::distance(names.begin(), it));
    }

    auto ArmTranslator::processThrottleCmd(Throttle::ConstPtr const& msg) const -> void {
        if (msg->names.size() != msg->throttles.size()) {
            ROS_ERROR("Name count and value count mismatched!");
            return;
        }

        Throttle throttle = *msg;

        // If present, replace the entries for DE0 and DE1 with the pitch and roll values respectively
        std::optional<std::size_t> jointDePitchIndex = findJointByName(msg->names, "joint_de_pitch"), jointDeRollIndex = findJointByName(msg->names, "joint_de_roll");
        if (jointDePitchIndex && jointDeRollIndex) {

            std::size_t pitchIndex = jointDePitchIndex.value(), rollIndex = jointDeRollIndex.value();

            Vector2<Dimensionless> pitchRollThrottles{msg->throttles.at(pitchIndex), msg->throttles.at(rollIndex)};
            Vector2<Dimensionless> motorThrottles = PITCH_ROLL_TO_0_1 * pitchRollThrottles;

            throttle.names[pitchIndex] = "joint_de_0";
            throttle.names[rollIndex] = "joint_de_1";
            throttle.throttles[pitchIndex] = motorThrottles[0].get();
            throttle.throttles[rollIndex] = motorThrottles[1].get();
        }

        mThrottlePub->publish(throttle);
    }

    auto ArmTranslator::processVelocityCmd(Velocity::ConstPtr const& msg) -> void {
        if (msg->names.size() != msg->velocities.size()) {
            ROS_ERROR("Name count and value count mismatched!");
            return;
        }

        Velocity velocity = *msg;

        // TODO(quintin): Decrease code duplication
        // If present, replace the entries for DE0 and DE1 with the pitch and roll values respectively
        std::optional<std::size_t> jointDePitchIndex = findJointByName(msg->names, "joint_de_pitch"), jointDeRollIndex = findJointByName(msg->names, "joint_de_roll");
        if (jointDePitchIndex && jointDeRollIndex) {
            std::size_t pitchIndex = jointDePitchIndex.value(), rollIndex = jointDeRollIndex.value();

            Vector2<RadiansPerSecond> pitchRollVelocities{msg->velocities.at(pitchIndex), msg->velocities.at(rollIndex)};
            Vector2<RadiansPerSecond> motorVelocities = PITCH_ROLL_TO_0_1 * pitchRollVelocities;

            velocity.names[pitchIndex] = "joint_de_0";
            velocity.names[rollIndex] = "joint_de_1";
            velocity.velocities[pitchIndex] = motorVelocities[0].get();
            velocity.velocities[rollIndex] = motorVelocities[1].get();
        }

        mVelocityPub->publish(velocity);
    }

    auto ArmTranslator::processPositionCmd(Position::ConstPtr const& msg) -> void {
        if (msg->names.size() != msg->positions.size()) {
            ROS_ERROR("Name count and value count mismatched!");
            return;
        }

        Position position = *msg;

        for (size_t i = 0; i < position.positions.size(); ++i) {
            if (position.names[i] != "joint_c") {
                continue;
            }
            position.positions[i] = position.positions[i] / (2 * std::numbers::pi_v<float>);
        }

        // TODO(quintin): Decrease code duplication
        // If present, replace the entries for DE0 and DE1 with the pitch and roll values respectively
        std::optional<std::size_t> jointDePitchIndex = findJointByName(msg->names, "joint_de_pitch"), jointDeRollIndex = findJointByName(msg->names, "joint_de_roll");
        if (jointDePitchIndex && jointDeRollIndex) {
            std::size_t pitchIndex = jointDePitchIndex.value(), rollIndex = jointDeRollIndex.value();

            Vector2<RadiansPerSecond> pitchRoll{msg->positions.at(pitchIndex), msg->positions.at(rollIndex)};
            Vector2<RadiansPerSecond> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * pitchRoll;

            position.names[pitchIndex] = "joint_de_0";
            position.names[rollIndex] = "joint_de_1";
            position.positions[pitchIndex] = motorPositions[0].get();
            position.positions[rollIndex] = motorPositions[1].get();
        }

        mPositionPub->publish(position);
    }

    auto wrapAngle(float angle) -> float {
        constexpr float pi = std::numbers::pi_v<float>;
        constexpr float tau = 2 * pi;
        return std::fmod(angle + pi, tau) - pi;
    }

    auto ArmTranslator::processJointState(sensor_msgs::JointState::ConstPtr const& msg) -> void {
        if (mArmHWNames != msg->name || mArmHWNames.size() != msg->position.size() || mArmHWNames.size() != msg->velocity.size() || mArmHWNames.size() != msg->effort.size()) {
            ROS_ERROR("Forwarding joint data for arm is ignored!");
            return;
        }

        sensor_msgs::JointState jointState = *msg;

        std::optional<std::size_t> jointDe0Index = findJointByName(msg->name, "joint_de_0"), jointDe1Index = findJointByName(msg->name, "joint_de_1");
        if (jointDe0Index && jointDe1Index) {
            // The Moteus reports auxiliary motor positions in the range [0, tau) instead of [-pi, pi)
            // Wrap to better align with IK conventions
            auto pitchWrapped = wrapAngle(static_cast<float>(msg->position.at(jointDe0Index.value())));
            auto rollWrapped = wrapAngle(static_cast<float>(msg->position.at(jointDe1Index.value())));
            mJointDePitchRoll = {pitchWrapped, rollWrapped};
            jointState.name[jointDe0Index.value()] = "joint_de_pitch";
            jointState.name[jointDe1Index.value()] = "joint_de_roll";
            jointState.position[jointDe0Index.value()] = pitchWrapped;
            jointState.position[jointDe1Index.value()] = rollWrapped;
        }

        mJointDataPub->publish(jointState);
    }

    auto ArmTranslator::updateDeOffsets(ros::TimerEvent const&) -> void {
        if (!mJointDePitchRoll) return;

        Vector2<Radians> motorPositions = PITCH_ROLL_TO_01_SCALE * PITCH_ROLL_TO_0_1 * mJointDePitchRoll.value();
        {
            AdjustMotor adjust;
            adjust.request.name = "joint_de_0";
            adjust.request.value = motorPositions[0].get();
            mAdjustClientsByArmHwNames["joint_de_0"].call(adjust);
        }
        {
            AdjustMotor adjust;
            adjust.request.name = "joint_de_1";
            adjust.request.value = motorPositions[1].get();
            mAdjustClientsByArmHwNames["joint_de_1"].call(adjust);
        }
    }

} // namespace mrover
