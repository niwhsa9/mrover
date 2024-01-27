#pragma once

#include <array>
#include <concepts>
#include <optional>
#include <type_traits>
#include <variant>

#include <hardware.hpp>
#include <messaging.hpp>
#include <pidf.hpp>
#include <units/units.hpp>

#include "encoders.hpp"
#include "hbridge.hpp"
#include "testing.hpp"

namespace mrover {

    class Controller {
        struct PositionMode {
            PIDF<Radians, Percent> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<Radians, inverse<Seconds>>, Percent> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        /* ==================== Hardware ==================== */
        FDCAN<InBoundMessage> m_fdcan;
        HBridge m_motor_driver;
        TIM_HandleTypeDef* m_watchdog_timer{};
        bool m_watchdog_enabled{};
        TIM_HandleTypeDef* m_quadrature_encoder_timer{};
        TIM_HandleTypeDef* m_quadrature_encoder_vel_timer{};
        I2C_HandleTypeDef* m_absolute_encoder_i2c{};
        std::optional<QuadratureEncoderReader> m_relative_encoder;
        // TODO: implement
        std::optional<AbsoluteEncoderReader> m_absolute_encoder;
        std::array<LimitSwitch, 4> m_limit_switches;

        /* ==================== Internal State ==================== */
        Mode m_mode;
        // "Desired" since it may be overridden.
        // For example if we are trying to drive into a limit switch it will be overriden to zero.
        Percent m_desired_output;
        std::optional<Radians> m_uncalib_position;
        std::optional<RadiansPerSecond> m_velocity;
        BDCMCErrorInfo m_error = BDCMCErrorInfo::DEFAULT_START_UP_NOT_CONFIGURED;

        struct StateAfterConfig {
            Dimensionless gear_ratio;
            Radians max_forward_pos;
            Radians max_backward_pos;
        };

        std::optional<StateAfterConfig> m_state_after_config; // Present if and only if we are configured

        struct StateAfterCalib {
            // Ex: If the encoder reads in 6 Radians and offset is 4 Radians,
            // Then my actual current position should be 2 Radians.
            Radians offset_position;
        };

        std::optional<StateAfterCalib> m_state_after_calib; // Present if and only if we are calibrated

        // Messaging
        InBoundMessage m_inbound = IdleCommand{};
        OutBoundMessage m_outbound = ControllerDataState{.config_calib_error_data = {.error = m_error}};

        /**
         * \brief Updates \link m_uncalib_position and \link m_velocity based on the hardware
         */
        auto update_relative_encoder() -> void {
            std::optional<EncoderReading> reading;

            if (m_relative_encoder) {
                reading = m_relative_encoder->read();
            }

            if (reading) {
                auto const& [position, velocity] = reading.value();
                m_uncalib_position = position;
                m_velocity = velocity;
            } else {
                m_uncalib_position = std::nullopt;
                m_velocity = std::nullopt;
            }
        }

        auto update_limit_switches() -> void {
            // TODO: verify this is correct
            for (LimitSwitch& limit_switch: m_limit_switches) {
                limit_switch.update_limit_switch();
                // Each limit switch may have a position associated with it
                // If we reach there update our offset position since we know exactly where we are

                if (limit_switch.pressed()) {
                    if (std::optional<Radians> readjustment_position = limit_switch.get_readjustment_position()) {
                        if (!m_state_after_calib) m_state_after_calib.emplace();

                        m_state_after_calib->offset_position = m_uncalib_position.value() - readjustment_position.value();
                    }
                }
            }
        }

        auto drive_motor() -> void {
            std::optional<Percent> output;

            if (m_state_after_config) {
                // TODO: verify this is correct
                bool limit_forward = m_desired_output > 0_percent && std::ranges::any_of(m_limit_switches, [](LimitSwitch const& limit_switch) { return limit_switch.limit_forward(); });
                bool limit_backward = m_desired_output < 0_percent && std::ranges::any_of(m_limit_switches, [](LimitSwitch const& limit_switch) { return limit_switch.limit_backward(); });
                if (limit_forward || limit_backward) {
                    m_error = BDCMCErrorInfo::OUTPUT_SET_TO_ZERO_SINCE_EXCEEDING_LIMITS;
                } else {
                    output = m_desired_output;
                }
            }

            m_motor_driver.write(output.value_or(0_percent));
        }

        auto process_command(AdjustCommand const& message) -> void {
            update_relative_encoder();

            // TODO: verify this is correct
            if (m_uncalib_position && m_state_after_config) {
                m_state_after_calib = StateAfterCalib{
                        .offset_position = m_uncalib_position.value() - message.position,
                };
            }
        }

        auto process_command(ConfigCommand const& message) -> void {
            // Initialize values
            StateAfterConfig config{.gear_ratio = message.gear_ratio};

            if (message.quad_abs_enc_info.quad_present) {
                // TODO(quintin): Why TF does this crash without .get() ?
                Ratio multiplier = (message.quad_abs_enc_info.quad_is_forward_polarity ? 1.0f : -1.0f) * message.quad_enc_out_ratio.get();
                if (!m_relative_encoder) m_relative_encoder.emplace(m_quadrature_encoder_timer, multiplier, m_quadrature_encoder_vel_timer);
            }
            if (message.quad_abs_enc_info.abs_present) {
                Ratio multiplier = (message.quad_abs_enc_info.abs_is_forward_polarity ? 1 : -1) * message.abs_enc_out_ratio;
                if (!m_absolute_encoder) m_absolute_encoder.emplace(SMBus<uint8_t, uint16_t>{m_absolute_encoder_i2c}, 0, 0, multiplier);
            }

            m_motor_driver.change_max_pwm(message.max_pwm);

            config.max_forward_pos = message.max_forward_pos;
            config.max_backward_pos = message.max_backward_pos;

            // TODO: verify this is correct
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i)) {
                    bool enabled = GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i);
                    bool active_high = GET_BIT_AT_INDEX(message.limit_switch_info.active_high, i);
                    bool used_for_readjustment = GET_BIT_AT_INDEX(message.limit_switch_info.use_for_readjustment, i);
                    bool limits_forward = GET_BIT_AT_INDEX(message.limit_switch_info.limits_forward, i);
                    auto associated_position = Radians{message.limit_switch_info.limit_readj_pos[i]};

                    m_limit_switches[i].initialize(enabled, active_high, used_for_readjustment, limits_forward, associated_position);
                }
            }
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                if (GET_BIT_AT_INDEX(message.limit_switch_info.present, i) && GET_BIT_AT_INDEX(message.limit_switch_info.enabled, i)) {
                    m_limit_switches[i].enable();
                }
            }

            m_state_after_config = config;

            update_relative_encoder();
        }

        auto process_command(IdleCommand const&) -> void {
            // TODO - what is the expected behavior? just afk?
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            m_desired_output = 0_percent;
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(ThrottleCommand const& message) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            m_desired_output = message.throttle;
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(VelocityCommand const& message, VelocityMode& mode) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }

            update_relative_encoder();

            if (!m_velocity) {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
                return;
            }

            RadiansPerSecond target = message.velocity;
            RadiansPerSecond input = m_velocity.value();
            m_desired_output = mode.pidf.calculate(input, target);
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        auto process_command(PositionCommand const& message, PositionMode& mode) -> void {
            if (!m_state_after_config) {
                m_error = BDCMCErrorInfo::RECEIVING_COMMANDS_WHEN_NOT_CONFIGURED;
                return;
            }
            if (!m_state_after_calib) {
                m_error = BDCMCErrorInfo::RECEIVING_POSITION_COMMANDS_WHEN_NOT_CALIBRATED;
                return;
            }

            update_relative_encoder();

            if (!m_uncalib_position) {
                m_error = BDCMCErrorInfo::RECEIVING_PID_COMMANDS_WHEN_NO_READER_EXISTS;
                return;
            }

            Radians target = message.position;
            Radians input = m_uncalib_position.value();
            m_desired_output = mode.pidf.calculate(input, target);
            m_error = BDCMCErrorInfo::NO_ERROR;
        }

        struct detail {
            template<typename Command, typename V>
            struct command_to_mode;

            template<typename Command, typename ModeHead, typename... Modes>
            struct command_to_mode<Command, std::variant<ModeHead, Modes...>> { // Linear search to find corresponding mode
                using type = std::conditional_t<requires(Controller controller, Command command, ModeHead mode) { controller.process_command(command, mode); },
                                                ModeHead,
                                                typename command_to_mode<Command, std::variant<Modes...>>::type>; // Recursive call
            };

            template<typename Command> // Base case
            struct command_to_mode<Command, std::variant<>> {
                using type = std::monostate;
            };
        };

        template<typename Command, typename T>
        using command_to_mode_t = typename detail::command_to_mode<Command, T>::type;

    public:
        Controller() = default;

        Controller(TIM_HandleTypeDef* hbridge_output, Pin hbridge_forward_pin, Pin hbridge_backward_pin, FDCAN<InBoundMessage> const& fdcan, TIM_HandleTypeDef* watchdog_timer, TIM_HandleTypeDef* quadrature_encoder_timer, TIM_HandleTypeDef* quadrature_encoder_vel_timer, I2C_HandleTypeDef* absolute_encoder_i2c, std::array<LimitSwitch, 4> const& limit_switches)
            : m_motor_driver{HBridge(hbridge_output, hbridge_forward_pin, hbridge_backward_pin)},
              m_fdcan{fdcan},
              m_watchdog_timer{watchdog_timer},
              m_quadrature_encoder_timer{quadrature_encoder_timer},
              m_quadrature_encoder_vel_timer{quadrature_encoder_vel_timer},
              m_absolute_encoder_i2c{absolute_encoder_i2c},
              m_limit_switches{limit_switches} {}

        template<typename Command>
        auto process_command(Command const& command) -> void {
            // Find the "process_command" function that has the right type for the command
            using ModeForCommand = command_to_mode_t<Command, Mode>;

            // If the current mode is not the mode that "process_command" expects, change the mode to a fresh new instance of the correct one
            if (!std::holds_alternative<ModeForCommand>(m_mode)) m_mode.emplace<ModeForCommand>();

            if constexpr (std::is_same_v<ModeForCommand, std::monostate>) {
                process_command(command);
            } else {
                process_command(command, std::get<ModeForCommand>(m_mode));
            }
        }

        /**
         * \brief           Called from the FDCAN interrupt handler when a new message is received, updating \link m_inbound and processing it.
         * \param message   Command message to process.
         *
         * \note            This resets the message watchdog timer.
         */
        auto receive(InBoundMessage const& message) -> void {
            // Ensure watchdog timer is reset and enabled now that we are receiving messages
            __HAL_TIM_SetCounter(m_watchdog_timer, 0);
            if (!m_watchdog_enabled) {
                HAL_TIM_Base_Start_IT(m_watchdog_timer);
                m_watchdog_enabled = true;
            }

            m_inbound = message;
            process_command();
        }

        /**
         * \brief Update all non-blocking readings, process the current command stored in \link m_inbound, update \link m_outbound, and drive the motor.
         *
         * \note Reading the limit switches and encoders is non-blocking since they are memory-mapped.
         */
        auto process_command() -> void {
            update_limit_switches();
            update_relative_encoder();
            std::visit([&](auto const& command) { process_command(command); }, m_inbound);
            drive_motor();
        }

        /**
         * \brief Called after not receiving a message for too long. Responsible for stopping the motor for safety.
         *
         * This disables the watchdog timer until we receive another message.
         */
        auto receive_watchdog_expired() -> void {
            HAL_TIM_Base_Stop_IT(m_watchdog_timer);
            m_watchdog_enabled = false;

            // We lost connection or some other error happened
            // Make sure the motor stops
            m_inbound = IdleCommand{};
            process_command();
        }

        /**
         * \brief Update the quadrature velocity measurement.
         *
         * \note Called more frequently than update position.
         */
        auto calc_quadrature_velocity() -> void {
            m_relative_encoder->update_vel();
        }

        /**
         * \brief Serialize our internal state into an outbound status message
         *
         * \note This does not actually send the message it just updates it. We want to send at a lower rate in \link send()
         */
        auto update_outbound() -> void {
            ControllerDataState state{
                    // Encoding as NaN instead of an optional saves space in the message
                    // It also has a predictable memory layout
                    .position = [&] {
                        if (m_uncalib_position && m_state_after_calib) {
                            return m_uncalib_position.value() - m_state_after_calib->offset_position;
                        }
                        return Radians{std::numeric_limits<float>::quiet_NaN()};
                    }(),
                    .velocity = m_velocity.value_or(RadiansPerSecond{std::numeric_limits<float>::quiet_NaN()}),
                    .config_calib_error_data = ConfigCalibErrorInfo{
                            .configured = m_state_after_config.has_value(),
                            .calibrated = m_state_after_calib.has_value(),
                            .error = m_error,
                    },
            };
            for (std::size_t i = 0; i < m_limit_switches.size(); ++i) {
                SET_BIT_AT_INDEX(state.limit_switches.hit, i, m_limit_switches[i].pressed());
            }
            m_outbound = state;
        }

        /**
         * \brief Process the last received command again and update our outbound status message.
         *
         * We want to process again since the encoder readings may have changed and we need to update our motor output.
         * We also may have new readings that need to be put in the outbound message.
         *
         * This update loop should be called as fast as possible without overloading the MCU.
         */
        auto update() -> void {
            process_command();

            update_outbound();
        }

        /**
         * \brief Send out the current outbound status message.
         *
         * The update rate should be limited to avoid hammering the FDCAN bus.
         */
        auto send() -> void {
            m_fdcan.broadcast(m_outbound);
        }


        auto request_absolute_encoder_data() -> void {
            // Only read the encoder if we are configured
            if (m_absolute_encoder) {
                m_absolute_encoder->request_raw_angle();
            }
        }

        auto read_absolute_encoder_data() -> void {
            if (m_absolute_encoder) {
                m_absolute_encoder->read_raw_angle_into_buffer();
            }
        }

        auto update_absolute_encoder() -> void {
            // TODO: make this work
            // std::optional<EncoderReading> reading;
            //
            // // Only read the encoder if we are configured
            // if (m_absolute_encoder) {
            //     reading = m_absolute_encoder->read();
            // }
            //
            // if (reading) {
            //     auto const& [position, velocity] = reading.value();
            //     if (m_state_after_calib) {
            //         m_uncalib_position = position - m_state_after_calib->offset_position;
            //     }
            //     m_velocity = velocity;
            // } else {
            //     m_uncalib_position = std::nullopt;
            //     m_velocity = std::nullopt;
            // }
        }
    };

} // namespace mrover
