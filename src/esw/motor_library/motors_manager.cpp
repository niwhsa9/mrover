#include "motors_manager.hpp"

namespace mrover {

    using namespace std::chrono_literals;

    MotorsManager::MotorsManager(ros::NodeHandle const& nh, std::string groupName, std::vector<std::string> controllerNames)
        : mNh{nh},
          mControllerNames{std::move(controllerNames)},
          mGroupName{std::move(groupName)} {

        // Load motor controllers configuration from the ROS parameter server
        XmlRpc::XmlRpcValue controllersRoot;
        assert(nh.hasParam("motors/controllers"));
        nh.getParam("motors/controllers", controllersRoot);
        assert(controllersRoot.getType() == XmlRpc::XmlRpcValue::TypeStruct);

        for (const std::string& name: mControllerNames) {
            assert(controllersRoot[name].hasMember("type") &&
                   controllersRoot[name]["type"].getType() == XmlRpc::XmlRpcValue::TypeString);
            std::string type = static_cast<std::string>(controllersRoot[name]["type"]);
            assert(type == "brushed" || type == "brushless");

            // TODO: avoid hard coding Jetson here
            if (type == "brushed") {
                auto temp = std::make_unique<BrushedController>(nh, "jetson", name);
                mControllers[name] = std::move(temp);
            } else if (type == "brushless") {
                auto temp = std::make_unique<BrushlessController>(nh, "jetson", name);
                mControllers[name] = std::move(temp);
            }

            // TODO(guthrie) make this compile
            //            names[controllers[name]->get_can_manager().get_id()] = name;
        }

        updateLastConnection();

        // Subscribe to the ROS topic for commands
        mMoveThrottleSub = mNh.subscribe<Throttle>(mGroupName + "_throttle_cmd", 1, &MotorsManager::moveMotorsThrottle, this);
        mMoveVelocitySub = mNh.subscribe<Velocity>(mGroupName + "_velocity_cmd", 1, &MotorsManager::moveMotorsVelocity, this);
        mMovePositionSub = mNh.subscribe<Position>(mGroupName + "_position_cmd", 1, &MotorsManager::moveMotorsPosition, this);

        // Create a 0.1 second heartbeat timer
        ros::Timer heartbeatTimer = nh.createTimer(ros::Duration(0.1), &MotorsManager::heartbeatCallback, this);
    }

    Controller& MotorsManager::get_controller(std::string const& name) {
        return *mControllers.at(name);
    }

    void MotorsManager::process_frame(int bus, int id, std::span<std::byte const> frame_data) {
        // TODO: figure out how to send to corresponding controller
    }

    void MotorsManager::moveMotorsThrottle(const Throttle::ConstPtr& msg) {
        if (msg->names != mControllerNames && msg->names.size() != msg->throttles.size()) {
            ROS_ERROR("Throttle request is invalid!");
            return;
        }

        updateLastConnection();

        for (size_t i = 0; i < msg->names.size(); ++i) {
            const std::string& name = msg->names[i];
            Controller& controller = get_controller(name);
            controller.set_desired_throttle(msg->throttles[i]);
        }
    }

    void MotorsManager::moveMotorsVelocity(const Velocity::ConstPtr& msg) {
        if (msg->names != mControllerNames && msg->names.size() != msg->velocities.size()) {
            ROS_ERROR("Velocity request is invalid!");
            return;
        }

        updateLastConnection();

        for (size_t i = 0; i < msg->names.size(); ++i) {
            const std::string& name = msg->names[i];
            Controller& controller = get_controller(name);
            controller.set_desired_velocity(RadiansPerSecond{msg->velocities[i]});
        }
    }

    void MotorsManager::moveMotorsPosition(const Position::ConstPtr& msg) {
        if (msg->names != mControllerNames && msg->names.size() != msg->positions.size()) {
            ROS_ERROR("Arm request is invalid!");
            return;
        }

        updateLastConnection();

        for (std::size_t i = 0; i < msg->names.size(); ++i) {
            const std::string& name = msg->names[i];
            Controller& controller = get_controller(name);
            controller.set_desired_position(Radians{msg->positions[i]});
        }
    }

    void MotorsManager::heartbeatCallback(const ros::TimerEvent&) {
        auto duration = std::chrono::high_resolution_clock::now() - lastConnection;
        if (duration < 100ms) {
            for (const auto& motorName: mControllerNames) {
                Controller& controller = get_controller(motorName);
                controller.set_desired_throttle(0_percent);
            }
        }
    }

    void MotorsManager::updateLastConnection() {
        // Used as an override if we know that all motors have been called recently.
        lastConnection = std::chrono::high_resolution_clock::now();
    }

} // namespace mrover
