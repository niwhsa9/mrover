#pragma once

#include <moteus/moteus.h>

#include <can_device.hpp>
#include <controller.hpp>
#include <moteus/moteus_multiplex.h>

namespace mrover {

    using namespace mjbots;

    enum class Mode {
        Stopped = 0,
        Fault = 1,
        PreparingToOperate1 = 2,
        PreparingToOperate2 = 3,
        PreparingToOperate3 = 4,
        PWMMode = 5,
        VoltageMode = 6,
        VoltageFOC = 7,
        VoltageDQ = 8,
        Current = 9,
        Position = 10,
        Timeout = 11,
        ZeroVelocity = 12,
        StayWithin = 13,
        MeasureInductance = 14,
        Brake = 15
    };

    enum class ErrorCode {
        DmaStreamTransferError = 1,
        DmaStreamFifiError = 2,
        UartOverrunError = 3,
        UartFramingError = 4,
        UartNoiseError = 5,
        UartBufferOverrunError = 6,
        UartParityError = 7,
        CalibrationFault = 32,
        MotorDriverFault = 33,
        OverVoltage = 34,
        EncoderFault = 35,
        MotorNotConfigured = 36,
        PwmCycleOverrun = 37,
        OverTemperature = 38,
        StartOutsideLimit = 39,
        UnderVoltage = 40,
        ConfigChanged = 41,
        ThetaInvalid = 42,
        PositionInvalid = 43,
    };

    struct MoteusLimitSwitchInfo {
        bool isForwardPressed{};
        bool isBackwardPressed{};
    };

    template<IsUnit TOutputPosition>
    class BrushlessController final : public ControllerBase<TOutputPosition, BrushlessController<TOutputPosition>> {
        using Base = ControllerBase<TOutputPosition, BrushlessController>;

        // TODO(quintin): this is actually so dumb
        using OutputPosition = typename Base::OutputPosition;
        using OutputVelocity = typename Base::OutputVelocity;

        using Base::mControllerName;
        using Base::mCurrentPosition;
        using Base::mCurrentVelocity;
        using Base::mDevice;
        using Base::mErrorState;
        using Base::mLimitHit;
        using Base::mMasterName;
        using Base::mNh;
        using Base::mState;

    public:
        BrushlessController(ros::NodeHandle const& nh, std::string masterName, std::string controllerName)
            : Base{nh, std::move(masterName), std::move(controllerName)} {

            XmlRpc::XmlRpcValue brushlessMotorData;
            assert(mNh.hasParam(std::format("brushless_motors/controllers/{}", mControllerName)));
            mNh.getParam(std::format("brushless_motors/controllers/{}", mControllerName), brushlessMotorData);
            assert(brushlessMotorData.getType() == XmlRpc::XmlRpcValue::TypeStruct);

            mMinVelocity = OutputVelocity{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "min_velocity", -1.0)};
            mMaxVelocity = OutputVelocity{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_velocity", 1.0)};

            mMinPosition = OutputPosition{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "min_position", -1.0)};
            mMaxPosition = OutputPosition{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_position", 1.0)};

            mMaxTorque = xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "max_torque", 0.3);
            mWatchdogTimeout = xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "watchdog_timeout", 0.25);

            limitSwitch0Present = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_present", false);
            limitSwitch1Present = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_present", false);
            limitSwitch0Enabled = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_enabled", true);
            limitSwitch1Enabled = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_enabled", true);

            limitSwitch0LimitsFwd = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_limits_fwd", false);
            limitSwitch1LimitsFwd = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_limits_fwd", false);
            limitSwitch0ActiveHigh = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_is_active_high", true);
            limitSwitch1ActiveHigh = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_is_active_high", true);
            limitSwitch0UsedForReadjustment = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_0_used_for_readjustment", false);
            limitSwitch1UsedForReadjustment = xmlRpcValueToTypeOrDefault<bool>(brushlessMotorData, "limit_1_used_for_readjustment", false);
            limitSwitch0ReadjustPosition = OutputPosition{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "limit_0_readjust_position", 0.0)};
            limitSwitch1ReadjustPosition = OutputPosition{xmlRpcValueToTypeOrDefault<double>(brushlessMotorData, "limit_1_readjust_position", 0.0)};

            // if active low, we want to make the default value make it believe that
            // the limit switch is NOT pressed.
            // This is because we may not receive the newest query message from the moteus
            // as a result of either testing or startup.
            if (limitSwitch0Present && !limitSwitch0ActiveHigh) {
                mMoteusAux2Info |= 0b01;
            }
            if (limitSwitch1Present) {
                mMoteusAux2Info |= 0b10;
            }

            moteus::Controller::Options options;
            moteus::Query::Format queryFormat{};
            queryFormat.aux1_gpio = moteus::kInt8;
            queryFormat.aux2_gpio = moteus::kInt8;
            if (this->isJointDe()) {
                // DE0 and DE1 have absolute encoders
                // They are not used for their internal control loops
                // Instead we request them at the ROS level and send adjust commands periodically
                // Therefore we do not get them as part of normal messages and must request them explicitly
                queryFormat.extra[0] = moteus::Query::ItemFormat{
                        .register_number = moteus::Register::kEncoder1Position,
                        .resolution = moteus::kFloat,
                };
                queryFormat.extra[1] = moteus::Query::ItemFormat{
                        .register_number = moteus::Register::kEncoder1Velocity,
                        .resolution = moteus::kFloat,
                };
            }
            options.query_format = queryFormat;
            mMoteus.emplace(options);
        }

        auto setDesiredThrottle(Percent throttle) -> void {
            setDesiredVelocity(mapThrottleToVelocity(throttle));
        }

        auto setDesiredPosition(OutputPosition position) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if ((limitSwitch0Enabled && limitSwitch0Present) || (limitSwitch1Enabled && limitSwitch0Present)) {
                sendQuery();

                if (auto [isFwdPressed, isBwdPressed] = getPressedLimitSwitchInfo();
                    (mCurrentPosition < position && isFwdPressed) ||
                    (mCurrentPosition > position && isBwdPressed)) {
                    setBrake();
                    return;
                }
            }

            position = std::clamp(position, mMinPosition, mMaxPosition);

            moteus::PositionMode::Command command{
                    .position = position.rep,
                    .velocity = 0.0,
                    .maximum_torque = mMaxTorque,
                    .watchdog_timeout = mWatchdogTimeout,
            };
            moteus::CanFdFrame positionFrame = mMoteus->MakePosition(command);
            mDevice.publish_moteus_frame(positionFrame);
        }

        auto processCANMessage(CAN::ConstPtr const& msg) -> void {
            assert(msg->source == mControllerName);
            assert(msg->destination == mMasterName);
            auto result = moteus::Query::Parse(msg->data.data(), msg->data.size());
            
            if (this->isJointDe()) {
                mCurrentPosition = OutputPosition{result.extra[0].value}; // Get value of absolute encoder if its joint_de0/1
                mCurrentVelocity = OutputVelocity{result.extra[1].value};
            } else {
                mCurrentPosition = OutputPosition{result.position};
                mCurrentVelocity = OutputVelocity{result.velocity};
            }
            mCurrentEffort = result.torque;

            mErrorState = moteusErrorCodeToErrorState(result.mode, static_cast<ErrorCode>(result.fault));
            mState = moteusModeToState(result.mode);

            mMoteusAux1Info = result.aux1_gpio;
            mMoteusAux2Info = result.aux2_gpio;

            if (result.mode == moteus::Mode::kPositionTimeout || result.mode == moteus::Mode::kFault) {
                setStop();
                ROS_WARN("Position timeout hit");
            }
        }

        auto setDesiredVelocity(OutputVelocity velocity) -> void {
            // Only check for limit switches if at least one limit switch exists and is enabled
            if ((limitSwitch0Enabled && limitSwitch0Present) || (limitSwitch1Enabled && limitSwitch0Present)) {
                sendQuery();

                if (auto [isFwdPressed, isBwdPressed] = getPressedLimitSwitchInfo();
                    (velocity > OutputVelocity{0} && isFwdPressed) ||
                    (velocity < OutputVelocity{0} && isBwdPressed)) {
                    setBrake();
                    return;
                }
            }

            velocity = std::clamp(velocity, mMinVelocity, mMaxVelocity);

            if (abs(velocity) < OutputVelocity{1e-5}) {
                setBrake();
            } else {
                moteus::PositionMode::Command command{
                        .position = std::numeric_limits<double>::quiet_NaN(),
                        .velocity = velocity.rep,
                        .maximum_torque = mMaxTorque,
                        .watchdog_timeout = mWatchdogTimeout,
                };

                moteus::CanFdFrame positionFrame = mMoteus->MakePosition(command);
                mDevice.publish_moteus_frame(positionFrame);
            }
        }

        [[nodiscard]] auto getEffort() const -> double {
            return mCurrentEffort;
        }

        auto setStop() -> void {
            moteus::CanFdFrame setStopFrame = mMoteus->MakeStop();
            mDevice.publish_moteus_frame(setStopFrame);
        }

        auto setBrake() -> void {
            moteus::CanFdFrame setBrakeFrame = mMoteus->MakeBrake();
            mDevice.publish_moteus_frame(setBrakeFrame);
        }

        auto getPressedLimitSwitchInfo() -> MoteusLimitSwitchInfo {
            if (limitSwitch0Present && limitSwitch0Enabled) {
                bool gpioState = 0b01 & mMoteusAux2Info;
                mLimitHit[0] = gpioState == limitSwitch0ActiveHigh;
            }
            if (limitSwitch1Present && limitSwitch1Enabled) {
                bool gpioState = 0b10 & mMoteusAux2Info;
                mLimitHit[1] = gpioState == limitSwitch1ActiveHigh;
            }

            MoteusLimitSwitchInfo result{
                    .isForwardPressed = (mLimitHit[0] && limitSwitch0LimitsFwd) || (mLimitHit[1] && limitSwitch1LimitsFwd),
                    .isBackwardPressed = (mLimitHit[0] && !limitSwitch0LimitsFwd) || (mLimitHit[1] && !limitSwitch1LimitsFwd),
            };

            if (result.isForwardPressed) {
                adjust(limitSwitch0ReadjustPosition);
            } else if (result.isBackwardPressed) {
                adjust(limitSwitch1ReadjustPosition);
            }

            return result;
        }

        auto adjust(OutputPosition position) -> void {
            position = std::clamp(position, mMinPosition, mMaxPosition);
            moteus::OutputExact::Command command{
                    .position = position.get(),
            };
            moteus::OutputExact::Command outputExactCmd{command};
            moteus::CanFdFrame setPositionFrame = mMoteus->MakeOutputExact(outputExactCmd);
            mDevice.publish_moteus_frame(setPositionFrame);
        }

        auto sendQuery() -> void {
            moteus::CanFdFrame queryFrame = mMoteus->MakeQuery();
            mDevice.publish_moteus_frame(queryFrame);
        }

    private:
        std::optional<moteus::Controller> mMoteus;
        bool limitSwitch0Present{};
        bool limitSwitch1Present{};
        bool limitSwitch0Enabled{};
        bool limitSwitch1Enabled{};
        bool limitSwitch0LimitsFwd{};
        bool limitSwitch1LimitsFwd{};
        bool limitSwitch0ActiveHigh{};
        bool limitSwitch1ActiveHigh{};
        bool limitSwitch0UsedForReadjustment{};
        bool limitSwitch1UsedForReadjustment{};
        OutputPosition limitSwitch0ReadjustPosition{};
        OutputPosition limitSwitch1ReadjustPosition{};

        double mMaxTorque{0.5};
        double mWatchdogTimeout{0.1};

        std::int8_t mMoteusAux1Info{}, mMoteusAux2Info{};

        OutputPosition mMinPosition, mMaxPosition;
        OutputVelocity mMinVelocity, mMaxVelocity;
        double mCurrentEffort{std::numeric_limits<double>::quiet_NaN()};

        [[nodiscard]] auto mapThrottleToVelocity(Percent throttle) const -> OutputVelocity {
            throttle = std::clamp(throttle, -1_percent, 1_percent);
            return abs(throttle) * (throttle > 0_percent ? mMaxVelocity : mMinVelocity);
        }

        // Converts moteus error codes and mode codes to std::string descriptions
        static auto moteusErrorCodeToErrorState(moteus::Mode motor_mode, ErrorCode motor_error_code) -> std::string {
            if (motor_mode != moteus::Mode::kFault) return "No Error";
            switch (motor_error_code) {
                case ErrorCode::DmaStreamTransferError:
                    return "DMA Stream Transfer Error";
                case ErrorCode::DmaStreamFifiError:
                    return "DMA Stream FIFO Error";
                case ErrorCode::UartOverrunError:
                    return "UART Overrun Error";
                case ErrorCode::UartFramingError:
                    return "UART Framing Error";
                case ErrorCode::UartNoiseError:
                    return "UART Noise Error";
                case ErrorCode::UartBufferOverrunError:
                    return "UART Buffer Overrun Error";
                case ErrorCode::UartParityError:
                    return "UART Parity Error";
                case ErrorCode::CalibrationFault:
                    return "Calibration Fault";
                case ErrorCode::MotorDriverFault:
                    return "Motor Driver Fault";
                case ErrorCode::OverVoltage:
                    return "Over Voltage";
                case ErrorCode::EncoderFault:
                    return "Encoder Fault";
                case ErrorCode::MotorNotConfigured:
                    return "Motor Not Configured";
                case ErrorCode::PwmCycleOverrun:
                    return "PWM Cycle Overrun";
                case ErrorCode::OverTemperature:
                    return "Over Temperature";
                case ErrorCode::StartOutsideLimit:
                    return "Start Outside Limit";
                case ErrorCode::UnderVoltage:
                    return "Under Voltage";
                case ErrorCode::ConfigChanged:
                    return "Configuration Changed";
                case ErrorCode::ThetaInvalid:
                    return "Theta Invalid";
                case ErrorCode::PositionInvalid:
                    return "Position Invalid";
                default:
                    return "Unknown Error";
            }
        }

        static auto moteusModeToState(moteus::Mode motor_mode) -> std::string {
            switch (motor_mode) {
                case moteus::Mode::kStopped:
                    return "Motor Stopped";
                case moteus::Mode::kFault:
                    return "Motor Fault";
                case moteus::Mode::kEnabling:
                    return "Motor Enabling";
                case moteus::Mode::kCalibrating:
                    return "Motor Calibrating";
                case moteus::Mode::kCalibrationComplete:
                    return "Motor Calibration Complete";
                case moteus::Mode::kPwm:
                    return "Motor Pwm";
                case moteus::Mode::kVoltage:
                    return "Voltage Operating Mode";
                case moteus::Mode::kVoltageFoc:
                    return "Voltage FOC Operating Mode";
                case moteus::Mode::kVoltageDq:
                    return "Voltage DQ Operating Mode";
                case moteus::Mode::kCurrent:
                    return "Current Operating Mode";
                case moteus::Mode::kPosition:
                    return "Position Operating Mode";
                case moteus::Mode::kPositionTimeout:
                    return "Position Timeout";
                case moteus::Mode::kZeroVelocity:
                    return "Zero Velocity";
                case moteus::Mode::kStayWithin:
                    return "Motor Stay Within";
                case moteus::Mode::kMeasureInd:
                    return "Measure Ind";
                case moteus::Mode::kBrake:
                    return "Motor Brake";
                default:
                    return "Unknown Mode";
            }
        }
    };

} // namespace mrover
