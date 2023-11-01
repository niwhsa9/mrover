#pragma once

#include <array>
#include <variant>

#include "units.hpp"


namespace mrover {

#pragma pack(push, 1)

    struct ConfigLimitSwitchInfo0 {
        std::uint8_t a_present : 1;
        std::uint8_t b_present : 1;
        std::uint8_t c_present : 1;
        std::uint8_t d_present : 1;
        std::uint8_t a_enable : 1;
        std::uint8_t b_enable : 1;
        std::uint8_t c_enable : 1;
        std::uint8_t d_enable : 1;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo0) == 1);

    struct ConfigLimitSwitchInfo1 {
        std::uint8_t a_active_high : 1;
        std::uint8_t b_active_high : 1;
        std::uint8_t c_active_high : 1;
        std::uint8_t d_active_high : 1;
        std::uint8_t a_limits_forward : 1;
        std::uint8_t b_limits_forward : 1;
        std::uint8_t c_limits_forward : 1;
        std::uint8_t d_limits_forward : 1;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo1) == 1);

    struct ConfigLimitSwitchInfo2 {
        std::uint8_t a_use_for_readjustment : 1;
        std::uint8_t b_use_for_readjustment : 1;
        std::uint8_t c_use_for_readjustment : 1;
        std::uint8_t d_use_for_readjustment : 1;
        std::uint8_t a_is_default_enabled : 1;
        std::uint8_t b_is_default_enabled : 1;
        std::uint8_t c_is_default_enabled : 1;
        std::uint8_t d_is_default_enabled : 1;
    };
    static_assert(sizeof(ConfigLimitSwitchInfo2) == 1);

    struct ConfigEncoderInfo {
        [[maybe_unused]] std::uint8_t _ignore : 4; // 8 bits - (4 meaningful bits) = 4 ignored bits
        std::uint8_t quad_present : 1;
        std::uint8_t quad_is_forward_polarity : 1;
        std::uint8_t abs_present : 1;
        std::uint8_t abs_is_forward_polarity : 1;
    };
    static_assert(sizeof(ConfigEncoderInfo) == 1);

    struct ConfigLimitInfo {
        [[maybe_unused]] std::uint8_t _ignore : 6; // 8 bits - (2 meaningful bits) = 6 ignored bits
        std::uint8_t limit_max_forward_position : 1;
        std::uint8_t limit_max_backward_position : 1;
    };

    struct BaseCommand {
        motor_id_t id;
    } PACKED;

    struct IdleCommand : BaseCommand {
    } PACKED;

    struct ThrottleCommand : BaseCommand {
        dimensionless_t throttle;
    } PACKED;

    struct VelocityCommand : BaseCommand {
        RadiansPerSecond velocity;
    } PACKED;

    struct PositionCommand : BaseCommand {
        Radians position;
    } PACKED;

    struct StatusState {

    } PACKED;

    using Message = std::variant<
            IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;
    static_assert(sizeof(Message) <= FRAME_SIZE);

    union FdCanFrame {
        Message message;
        std::array<std::byte, FRAME_SIZE> bytes;
    };

    using InBoundMessage = std::variant<
            ConfigCommand, IdleCommand, ThrottleCommand, VelocityCommand, PositionCommand>;

    using OutBoundMessage = std::variant<
            MotorDataState, StatusState>;

#pragma pack(pop)

} // namespace mrover
