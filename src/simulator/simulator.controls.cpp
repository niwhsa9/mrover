#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::throttlesCallback(Throttle::ConstPtr const& msg) -> void {
        forEachMotor(msg->names, msg->throttles, [&](btMultiBodyJointMotor* motor, float throttle) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(throttle, 0.5);
        });
    }

    auto SimulatorNodelet::positionsCallback(Position::ConstPtr const& msg) -> void {
        forEachMotor(msg->names, msg->positions, [&](btMultiBodyJointMotor* motor, float position) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(position, 0.05);
            motor->setVelocityTarget(0, 1);
        });
    }

    void SimulatorNodelet::velocitiesCallback(Velocity::ConstPtr const& msg) {
        forEachMotor(msg->names, msg->velocities, [&](btMultiBodyJointMotor* motor, float velocity) {
            motor->setMaxAppliedImpulse(0.5);
            motor->setPositionTarget(0, 0);
            motor->setVelocityTarget(velocity, 0.5);
        });
    }

    auto SimulatorNodelet::centerCursor() const -> void {
        Eigen::Vector2i size;
        glfwGetWindowSize(mWindow.get(), &size.x(), &size.y());
        Eigen::Vector2i center = size / 2;
        glfwSetCursorPos(mWindow.get(), center.x(), center.y());
    }

    auto SimulatorNodelet::keyCallback(int key, [[maybe_unused]] int scancode, int action, [[maybe_unused]] int mods) -> void {
        if (action == GLFW_PRESS) {
            if (key == mQuitKey) ros::requestShutdown();
            if (key == mTogglePhysicsKey) mEnablePhysics = !mEnablePhysics;
            if (key == mToggleRenderModelsKey) mRenderModels = !mRenderModels;
            if (key == mToggleRenderWireframeCollidersKey) mRenderWireframeColliders = !mRenderWireframeColliders;
            if (key == mInGuiKey) {
                mInGui = !mInGui;
                if (!mInGui) centerCursor();
            }
            if (key == mToggleCameraLockKey) {
                if (mCameraInRoverTarget) {
                    mCameraInRoverTarget = std::nullopt;
                    Eigen::Matrix3d rotationMatrix = mCameraInWorld.transform().block<3, 3>(0, 0);

                    Eigen::Vector3d left = rotationMatrix.col(1);
                    Eigen::Vector3d leftWithoutRoll = left;
                    leftWithoutRoll.z() = 0;

                    SO3d q = Eigen::Quaterniond{}.setFromTwoVectors(left, leftWithoutRoll);
                    mCameraInWorld.asSO3() = q * mCameraInWorld.asSO3();
                } else {
                    setCameraInRoverTarget();
                }
            }
        }
    }

    auto SimulatorNodelet::freeLook(Clock::duration dt) -> void {
        auto axis = [this](int positive, int negative) -> double {
            return (glfwGetKey(mWindow.get(), positive) == GLFW_PRESS) - (glfwGetKey(mWindow.get(), negative) == GLFW_PRESS);
        };

        float flySpeed = mFlySpeed * std::chrono::duration_cast<std::chrono::duration<float>>(dt).count();
        R3d keyDelta = R3d{axis(mCamForwardKey, mCamBackwardKey), axis(mCamLeftKey, mCamRightKey), axis(mCamUpKey, mCamDownKey)} * flySpeed;

        Eigen::Vector2i windowSize;
        glfwGetWindowSize(mWindow.get(), &windowSize.x(), &windowSize.y());
        Eigen::Vector2d windowCenter = (windowSize / 2).cast<double>();
        Eigen::Vector2d mouse;
        glfwGetCursorPos(mWindow.get(), &mouse.x(), &mouse.y());
        Eigen::Vector2d mouseDelta = (mouse - windowCenter) * mLookSense;

        SE3d::Tangent cameraRelativeTwist;
        cameraRelativeTwist << keyDelta.x(), keyDelta.y(), 0.0, 0.0, mouseDelta.y(), 0.0;

        SE3d::Tangent worldRelativeTwist;
        worldRelativeTwist << 0.0, 0.0, keyDelta.z(), 0.0, 0.0, 0.0;

        // The plus operator is overloaded and corresponds to lplus and rplus in the micro Lie paper
        // It applies the exponential map to the tangent space element
        // The result can be acted upon a group element (in this case SE3)
        // The side of the plus operator determines word vs. camera space application
        mCameraInWorld = worldRelativeTwist + mCameraInWorld + cameraRelativeTwist;

        SO3d::Tangent worldRelativeAngularVelocity;
        worldRelativeAngularVelocity << 0.0, 0.0, -mouseDelta.x();
        // TODO: Is there a tidy way to combine this with the above?
        mCameraInWorld.asSO3() = worldRelativeAngularVelocity + mCameraInWorld.asSO3();

        centerCursor();
    }

    auto SimulatorNodelet::cameraLock([[maybe_unused]] Clock::duration dt) -> void {
        if (!mCameraInRoverTarget) setCameraInRoverTarget();

        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;
            SE3d roverInWorld = rover.linkInWorld("base_link");
            SE3d cameraInWorldTarget = roverInWorld * mCameraInRoverTarget.value();
            mCameraInWorld = manif::interpolate_slerp(mCameraInWorld, cameraInWorldTarget, mCameraLockSlerp);
        }
        centerCursor();
    }

    auto SimulatorNodelet::setCameraInRoverTarget() -> void {
        if (auto lookup = getUrdf("rover")) {
            URDF const& rover = *lookup;
            SE3d roverInWorld = rover.linkInWorld("base_link");
            mCameraInRoverTarget = roverInWorld.inverse() * mCameraInWorld;
        }
    }

    auto SimulatorNodelet::userControls(Clock::duration dt) -> void {
        if (mPublishIk && mIkMode) {
            IK ik;
            ik.pos[0] = mIkTarget.x();
            ik.pos[1] = mIkTarget.y();
            ik.pos[2] = mIkTarget.z();
            ik.pitch = mIkPitch;
            ik.roll = mIkRoll;
            mIkTargetPub.publish(ik);
        }

        if (!mHasFocus || mInGui) return;

        if (mCameraInRoverTarget)
            cameraLock(dt);
        else
            freeLook(dt);

        geometry_msgs::Twist twist;
        if (glfwGetKey(mWindow.get(), mRoverRightKey) == GLFW_PRESS) {
            twist.angular.z = -mRoverAngularSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverLeftKey) == GLFW_PRESS) {
            twist.angular.z = mRoverAngularSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverForwardKey) == GLFW_PRESS) {
            twist.linear.x = mRoverLinearSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverBackwardKey) == GLFW_PRESS) {
            twist.linear.x = -mRoverLinearSpeed;
        }
        if (glfwGetKey(mWindow.get(), mRoverStopKey) == GLFW_PRESS) {
            twist.linear.x = 0;
            twist.angular.z = 0;
        }
        mCmdVelPub.publish(twist);

        mIkVel.setZero();
        mIkPitchVel = 0;
        mIkRollVel = 0;
        if (glfwGetKey(mWindow.get(), mArmForwardKey) == GLFW_PRESS) {
            mIkVel.x() = 1;
        }
        if (glfwGetKey(mWindow.get(), mArmBackwardKey) == GLFW_PRESS) {
            mIkVel.x() = -1;
        }
        if (glfwGetKey(mWindow.get(), mArmLeftKey) == GLFW_PRESS) {
            mIkVel.y() = 1;
        }
        if (glfwGetKey(mWindow.get(), mArmRightKey) == GLFW_PRESS) {
            mIkVel.y() = -1;
        }
        if (glfwGetKey(mWindow.get(), mArmUpKey) == GLFW_PRESS) {
            mIkVel.z() = 1;
        }
        if (glfwGetKey(mWindow.get(), mArmDownKey) == GLFW_PRESS) {
            mIkVel.z() = -1;
        }
        if (glfwGetKey(mWindow.get(), mArmPitchUpKey) == GLFW_PRESS) {
            mIkPitchVel = -1;
        }
        if (glfwGetKey(mWindow.get(), mArmPitchDownKey) == GLFW_PRESS) {
            mIkPitchVel = 1;
        }
        if (glfwGetKey(mWindow.get(), mArmRollCWKey) == GLFW_PRESS) {
            mIkRollVel = 1;
        }
        if (glfwGetKey(mWindow.get(), mArmRollCCWKey) == GLFW_PRESS) {
            mIkRollVel = -1;
        }
        mIkVel.normalize();
        mIkVel *= mArmSpeed;
        mIkPitchVel *= mArmSpeed;
        mIkRollVel *= mArmSpeed;
        if (mPublishIk && !mIkMode) {
            geometry_msgs::Twist vel;
            vel.linear.x = mIkVel.x();
            vel.linear.y = mIkVel.y();
            vel.linear.z = mIkVel.z();
            vel.angular.x = mIkRollVel;
            vel.angular.y = mIkPitchVel;
            mIkVelPub.publish(vel);
        }
    }

} // namespace mrover
