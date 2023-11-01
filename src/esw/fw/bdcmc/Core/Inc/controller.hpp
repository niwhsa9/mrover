#pragma once

#include "main.h"

#include <concepts>
#include <optional>
#include <variant>

#include "config.hpp"
#include "messaging.hpp"
#include "pidf.hpp"
#include "units.hpp"

extern FDCAN_HandleTypeDef hfdcan1;

static inline void check(bool cond, std::invocable auto handler) {
    if (!cond) {
        handler();
    }
}

namespace mrover {

    template<typename T, typename Input, typename TimeUnit = Seconds>
    concept InputReader = requires(T t, Config& config) {
        { t.update(config) };
        { t.read() } -> std::convertible_to<std::pair<Input, compound_unit<Input, inverse<TimeUnit>>>>;
    };

    template<typename T, typename Output>
    concept OutputWriter = requires(T t, Config& config, Output output) {
        { t.target_target_duty_cycle(config, output) };
        { t.write() };
    };

    template<IsUnit InputUnit, IsUnit OutputUnit,
             InputReader<InputUnit> Reader, OutputWriter<OutputUnit> Writer,
             IsUnit TimeUnit = Seconds>
    class Controller {
    private:
        struct PositionMode {
            PIDF<InputUnit, OutputUnit, TimeUnit> pidf;
        };

        struct VelocityMode {
            PIDF<compound_unit<InputUnit, inverse<TimeUnit>>, OutputUnit, TimeUnit> pidf;
        };

        using Mode = std::variant<std::monostate, PositionMode, VelocityMode>;

        Config m_config;
        Reader m_reader;
        Writer m_writer;
        InBoundMessage m_command;
        Mode m_mode;
        FDCAN_TxHeaderTypeDef TxHeader;

        inline void force_configure() {
            if (!m_config.configured()) {
                Error_Handler();
            }
        }

        void feed(IdleCommand const& message) {
            // TODO: Feel like we should be actively calling something here
            // maybe write_output with a throttle of 0?
        }

        void feed(ConfigCommand const& message) {
            m_config.configure(message);
        }

        void feed(ThrottleCommand const& message) {
            force_configure();
            m_writer.target_target_duty_cycle(m_config, message.throttle);
        }

        void feed(VelocityCommand const& message, VelocityMode mode) {
            force_configure();
            (void) message;
            (void) mode;
        }

        void feed(PositionCommand const& message, PositionMode mode) {
            force_configure();
            (void) message;
            (void) mode;
            // InputUnit input = m_reader.read_input();
            // InputUnit target = message.position;
            // OutputUnit output = mode.pidf.calculate(input, target);
            // m_writer.write_output(output);
        }

        struct detail {
            template<typename Command, typename V>
            struct command_to_mode;

            template<typename Command, typename ModeHead, typename... Modes>
            struct command_to_mode<Command, std::variant<ModeHead, Modes...>> { // Linear search to find corresponding mode
                using type = std::conditional_t<requires(Controller controller, Command command,
                                                         ModeHead mode) { controller.feed(command, mode); },
                                                ModeHead,
                                                typename command_to_mode<Command, std::variant<Modes...>>::type>; // Recursive call
            };

            template<typename Command> // Base case
            struct command_to_mode<Command, std::variant<>> {
                using type = std::monostate;
            };
        };

        template<typename Command, typename T>
        using command_to_mode_t = typename detail::template command_to_mode<Command, T>::type;

    public:
        Controller(const uint32_t id, Reader&& reader, Writer&& writer)
            : m_config(id), m_reader(std::move(reader)), m_writer(std::move(writer)), TxHeader{} {}

        template<typename Command>
        void process(Command const& command) {
            // Find the feed function that has the right type for the command
            using ModeForCommand = command_to_mode_t<Command, Mode>;

            // If the current mode is not the mode that the feed function expects, change the mode, providing a new blank mode
            if (!std::holds_alternative<ModeForCommand>(m_mode))
                m_mode.template emplace<ModeForCommand>();

            if constexpr (std::is_same_v<ModeForCommand, std::monostate>) {
                feed(command);
            } else {
                feed(command, std::get<ModeForCommand>(m_mode));
            }
        }

        void process(InBoundMessage const& message) {
            std::visit([&](auto const& command) { process(command); }, message);
        }

        void update() {
            m_reader.update(m_config);
            // TODO: enforce limit switch constraints
            m_writer.write();
        }

        // Transmit motor data out as CAN message
        void transmit() {
            // TODO: add modulo for frequency
            // TODO: send out Controller data as CAN message
            // Use getters to get info from encoder reader, have the bundling into CAN in controller
            //  then controller sends the array as a CAN message
            /* CAN FRAME FORMAT
             * 4 BYTES: Position (0-3)
             * 4 BYTES: Velocity (4-7)
             * 1 BYTES: Is Configured, Is Calibrated, Errors (8)
             * 1 BYTES: Limit a/b/c/d is hit. (9)
             */
            auto [position, velocity] = m_reader.read();
            mrover::FdCanFrameOut frame{
                            .message = mrover::MotorDataState{
                            .velocity = velocity,
                            .position = position,
                            // TODO: Actually fill the config_calib_error_data with data
                            .config_calib_error_data = 0x00,
                            // TODO: Is this going to be right or left aligned?
                            // TODO: actually fill with correct values
                            .limit_switches = 0x00,
                    },
            };

            // TODO: Copied values from somewhere else.
            // TODO: Identifier probably needs to change?
            TxHeader = {
                    .Identifier = 0x11,
                    .IdType = FDCAN_STANDARD_ID,
                    .TxFrameType = FDCAN_DATA_FRAME,
                    .DataLength = FDCAN_DLC_BYTES_12,
                    .ErrorStateIndicator = FDCAN_ESI_ACTIVE,
                    .BitRateSwitch = FDCAN_BRS_OFF,
                    .FDFormat = FDCAN_FD_CAN,
                    .TxEventFifoControl = FDCAN_NO_TX_EVENTS,
                    .MessageMarker = 0,
            };
            // TODO: I think this is all that is required to transmit a CAN message
            check(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &TxHeader, reinterpret_cast<uint8_t*>(&frame.bytes)) == HAL_OK, Error_Handler);
        }
    };

} // namespace mrover
