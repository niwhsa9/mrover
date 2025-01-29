#include "simulator.hpp"

namespace mrover {

    auto SimulatorNodelet::onInit() -> void try {
        mNh = getNodeHandle();
        mPnh = getPrivateNodeHandle();

        mSaveTask = PeriodicTask{mPnh.param<float>("save_rate", 5)};
        mSaveHistory = boost::circular_buffer<SaveData>{static_cast<std::size_t>(mPnh.param<int>("save_history", 4096))};

        mGroundTruthPub = mNh.advertise<nav_msgs::Odometry>("/ground_truth", 1);

        mCmdVelPub = mNh.advertise<geometry_msgs::Twist>("/simulator_cmd_vel", 1);

        mImageTargetsPub = mNh.advertise<ImageTargets>("/objects", 1);

        mIkTargetPub = mNh.advertise<IK>("ee_pos_cmd", 1);

        mIkVelPub = mNh.advertise<geometry_msgs::Twist>("ee_vel_cmd", 1);

        mIkModeClient = mNh.serviceClient<IkMode>("ik_mode");

        mIsHeadless = mPnh.param<bool>("headless", false);
        mEnablePhysics = mIsHeadless;
        {
            XmlRpc::XmlRpcValue gpsLinearization;
            mNh.getParam("gps_linearization", gpsLinearization);
            if (gpsLinearization.getType() != XmlRpc::XmlRpcValue::TypeStruct) throw std::invalid_argument{"GPS lineraization must be a struct. Did you rosparam load a localization config file properly?"};

            mGpsLinearizationReferencePoint = {
                    xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_point_latitude"),
                    xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_point_longitude"),
                    xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_point_altitude"),
            };
            mGpsLinerizationReferenceHeading = xmlRpcValueToTypeOrDefault<double>(gpsLinearization, "reference_heading");
        }

        if (!mIsHeadless) initWindow();

        initPhysics();

        initRender();

        initUrdfsFromParams();

        {
            auto addGroup = [&](std::string_view groupName, std::vector<std::string> const& names) {
                MotorGroup& group = mMotorGroups.emplace_back();
                group.updateTask = PeriodicTask{50};
                group.jointStatePub = mNh.advertise<sensor_msgs::JointState>(std::format("{}_joint_data", groupName), 1);
                group.controllerStatePub = mNh.advertise<ControllerState>(std::format("{}_controller_data", groupName), 1);
                group.names = names;
                group.throttleSub = mNh.subscribe<Throttle>(std::format("{}_throttle_cmd", groupName), 1, &SimulatorNodelet::throttlesCallback, this);
                group.velocitySub = mNh.subscribe<Velocity>(std::format("{}_velocity_cmd", groupName), 1, &SimulatorNodelet::velocitiesCallback, this);
                group.positionSub = mNh.subscribe<Position>(std::format("{}_position_cmd", groupName), 1, &SimulatorNodelet::positionsCallback, this);
            };
            addGroup("arm", {"joint_a", "joint_b", "joint_c", "joint_de_pitch", "joint_de_roll"});
            addGroup("drive_left", {"front_left", "middle_left", "back_left"});
            addGroup("drive_right", {"front_right", "middle_right", "back_right"});
        }

        mRunThread = std::thread{&SimulatorNodelet::run, this};

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    auto spinSleep(Clock::time_point const& until) -> void {
        //  See: https://blat-blatnik.github.io/computerBear/making-accurate-sleep-function/
        // Idea: sleep until 1ms before the desired time, then spin until the desired time
        std::this_thread::sleep_until(until - 1ms);
        while (Clock::now() < until)
            ;
    }

    auto SimulatorNodelet::run() -> void try {
        for (Clock::duration dt{}; ros::ok();) {
            Clock::time_point beginTime = Clock::now();

            mLoopProfiler.beginLoop();

            // Note(quintin):
            // Apple sucks and does not support polling on a non-main thread (which we are currently on)
            // Instead add to ROS's global callback queue which I think will be served by the main thread
            if (!mIsHeadless) {
#ifdef __APPLE__
                struct PollGlfw : ros::CallbackInterface {
                    auto call() -> CallResult override {
                        glfwPollEvents();
                        return Success;
                    }
                };
                ros::getGlobalCallbackQueue()->addCallback(boost::make_shared<PollGlfw>());
#else
                glfwPollEvents();
#endif
                // Comments this out while debugging segfaults, otherwise it captures your cursor
                // glfwSetInputMode(mWindow.get(), GLFW_CURSOR, mInGui ? GLFW_CURSOR_NORMAL : GLFW_CURSOR_DISABLED);
            }
            mLoopProfiler.measureEvent("GLFW Events");

            userControls(dt);
            mLoopProfiler.measureEvent("Controls");

            if (mEnablePhysics) physicsUpdate(dt);
            mLoopProfiler.measureEvent("Physics");

            renderUpdate();
            mLoopProfiler.measureEvent("Render");

            spinSleep(beginTime + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<float>{1.0f / mTargetUpdateRate}));
            dt = Clock::now() - beginTime;

            mLoopProfiler.measureEvent("Sleep");
        }

    } catch (std::exception const& e) {
        NODELET_FATAL_STREAM(e.what());
        ros::shutdown();
    }

    SimulatorNodelet::~SimulatorNodelet() {
        mRunThread.join();

        if (!mIsHeadless) {
            ImGui_ImplWGPU_Shutdown();
            ImGui_ImplGlfw_Shutdown();
            ImGui::DestroyContext();
        }

        for (int i = mDynamicsWorld->getNumConstraints() - 1; i >= 0; --i) {
            mDynamicsWorld->removeConstraint(mDynamicsWorld->getConstraint(i));
        }

        for (int i = mDynamicsWorld->getNumCollisionObjects() - 1; i >= 0; --i) {
            btCollisionObject* object = mDynamicsWorld->getCollisionObjectArray()[i];
            mDynamicsWorld->removeCollisionObject(object);
        }
    }

} // namespace mrover