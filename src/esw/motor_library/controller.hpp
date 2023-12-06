#pragma once

#include <string>

#include <ros/ros.h>

#include <can_device.hpp>
#include <chrono>
#include <iostream>
#include <units/units.hpp>

#include <mrover/ControllerState.h>
#include <mrover/Position.h>
#include <mrover/Throttle.h>
#include <mrover/Velocity.h>
#include <sensor_msgs/JointState.h>

namespace mrover {

    class Controller {
    public:
        Controller(ros::NodeHandle const& nh, std::string name, std::string controllerName)
            : mNh{nh},
              mName{std::move(name)},
              mControllerName{std::move(controllerName)},
              mDevice{nh, mName, mControllerName},
              mIncomingCANSub{
                      mNh.subscribe<CAN>(
                              std::format("can/{}/in", mControllerName), 16, &Controller::processCANMessage, this)} {
            updateLastConnection();
            mHeartbeatTimer = mNh.createTimer(ros::Duration(0.1), &Controller::heartbeatCallback, this);
            // Subscribe to the ROS topic for commands
            mMoveThrottleSub = mNh.subscribe<Throttle>(std::format("{}_throttle_cmd", mName), 1, &Controller::moveMotorsThrottle, this);
            mMoveVelocitySub = mNh.subscribe<Velocity>(std::format("{}_velocity_cmd", mName), 1, &Controller::moveMotorsVelocity, this);
            mMovePositionSub = mNh.subscribe<Position>(std::format("{}_position_cmd", mName), 1, &Controller::moveMotorsPosition, this);

            mJointDataPub = mNh.advertise<sensor_msgs::JointState>(std::format("{}_joint_data", mName), 1);
            mControllerDataPub = mNh.advertise<ControllerState>(std::format("{}_controller_data", mName), 1);

            mPublishDataTimer = mNh.createTimer(ros::Duration(0.1), &Controller::publishDataCallback, this);
        }

        virtual ~Controller() = default;

        virtual void setDesiredThrottle(Percent throttle) = 0;          // from -1.0 to 1.0
        virtual void setDesiredVelocity(RadiansPerSecond velocity) = 0; // joint output
        virtual void setDesiredPosition(Radians position) = 0;          // joint output
        virtual void processCANMessage(CAN::ConstPtr const& msg) = 0;
        virtual double getEffort() = 0;
        void updateLastConnection() {
            mLastConnection = std::chrono::high_resolution_clock::now();
        }

        void heartbeatCallback(ros::TimerEvent const&) {
            auto duration = std::chrono::high_resolution_clock::now() - mLastConnection;
            if (duration < std::chrono::milliseconds(100)) {
                setDesiredThrottle(0_percent);
            }
        }

        void moveMotorsThrottle(Throttle::ConstPtr const& msg) {
            if (msg->names.size() != 1 || msg->names.at(0) != mName || msg->throttles.size() != 1) {
                ROS_ERROR("Throttle request at topic for %s ignored!", mName.c_str());
                return;
            }
            setDesiredThrottle(msg->throttles.at(0));
        }


        void moveMotorsVelocity(Velocity::ConstPtr const& msg) {
            if (msg->names.size() != 1 || msg->names.at(0) != mName || msg->velocities.size() != 1) {
                ROS_ERROR("Velocity request at topic for %s ignored!", mName.c_str());
                return;
            }
            setDesiredVelocity(RadiansPerSecond{msg->velocities.at(0)});
        }

        void moveMotorsPosition(Position::ConstPtr const& msg) {
            if (msg->names.size() != 1 || msg->names.at(0) != mName || msg->positions.size() != 1) {
                ROS_ERROR("Position request at topic for %s ignored!", mName.c_str());
                return;
            }
            setDesiredPosition(Radians{msg->positions.at(0)});
        }

        void publishDataCallback(ros::TimerEvent const&) {
            sensor_msgs::JointState joint_state;
            ControllerState controller_state;
            joint_state.name.push_back(mName);
            joint_state.position.push_back(mCurrentPosition.get());
            joint_state.velocity.push_back(mCurrentVelocity.get());
            joint_state.effort.push_back(getEffort());

            controller_state.name.push_back(mName);
            controller_state.state.push_back(mState);
            controller_state.error.push_back(mErrorState);
            uint8_t limit_hit;
            for (int i = 0; i < 4; ++i) {
                limit_hit |= mLimitHit.at(i) << i;
            }
            controller_state.limit_hit.push_back(limit_hit);


            mJointDataPub.publish(joint_state);
            mControllerDataPub.publish(controller_state);
        }


    protected:
        ros::NodeHandle mNh;
        std::string mName, mControllerName;
        CanDevice mDevice;
        ros::Subscriber mIncomingCANSub;
        RadiansPerSecond mMinVelocity{}, mMaxVelocity{};
        Radians mMinPosition{}, mMaxPosition{};
        Radians mCurrentPosition{};
        RadiansPerSecond mCurrentVelocity{};
        bool mIsCalibrated{};
        std::string mErrorState;
        std::string mState;
        std::array<bool, 4> mLimitHit{};
        std::chrono::high_resolution_clock::time_point mLastConnection;
        ros::Timer mHeartbeatTimer;

        ros::Subscriber mMoveThrottleSub;
        ros::Subscriber mMoveVelocitySub;
        ros::Subscriber mMovePositionSub;
        ros::Publisher mJointDataPub;
        ros::Publisher mControllerDataPub;
        ros::Timer mPublishDataTimer;
    };

} // namespace mrover
