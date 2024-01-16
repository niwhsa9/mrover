#include "arm_controller.hpp"

namespace mrover {

    // Constants

    // From: arm.urdf.xacro
    constexpr double LINK_AB = 0.58;
    constexpr double LINK_BC = 0.55;
    constexpr double TAU = std::numbers::pi * 2;
    double const OFFSET = std::atan2(0.09, LINK_BC);

    // Subscribers

    [[maybe_unused]] ros::Subscriber ik_subscriber;

    // Publishers

    ros::Publisher position_publisher;

    // Private state

    IK ik_target;
    Position positions;

    auto run(int argc, char** argv) -> int {
        ros::init(argc, argv, "arm_controller");
        ros::NodeHandle nh;

        double frequency{};
        nh.param<double>("/frequency", frequency, 100.0);

        ik_subscriber = nh.subscribe("arm_ik", 1, ik_callback);
        position_publisher = nh.advertise<Position>("arm_position_cmd", 1);

        positions.names = {"arm_a_link", "arm_b_link", "arm_c_link", "arm_d_link", "arm_e_link"};
        positions.positions.resize(positions.names.size(), 0.f);

        ros::Rate rate{frequency};
        while (ros::ok()) {
            double x = ik_target.pose.position.x;
            double y = ik_target.pose.position.y;
            double z = ik_target.pose.position.z;

            double C = x * x + z * z - LINK_AB * LINK_AB - LINK_BC * LINK_BC;
            double q2 = std::acos(C / (2 * LINK_AB * LINK_BC));
            double q1 = std::atan2(z, x) - std::atan2(LINK_BC * std::sin(q2), LINK_AB + LINK_BC * std::cos(q2));
            q1 = std::clamp(q1, -TAU / 8, 0.0);
            double q3 = -(q1 + q2);

            ROS_INFO("x: %f, y: %f, z: %f, q1: %f, q2: %f, q3: %f", x, y, z, q1, q2, q3);

            if (std::isfinite(q1) && std::isfinite(q2) && std::isfinite(q3)) {
                positions.positions[0] = static_cast<float>(y);
                positions.positions[1] = static_cast<float>(q1);
                positions.positions[2] = static_cast<float>(q2);
                positions.positions[3] = static_cast<float>(q3);
                position_publisher.publish(positions);
            } else {}

            rate.sleep();
            ros::spinOnce();
        }

        return EXIT_SUCCESS;
    }

    void ik_callback(IK const& new_ik_target) {
        ik_target = new_ik_target;
    }

} // namespace mrover

int main(int argc, char** argv) {
    return mrover::run(argc, argv);
}
