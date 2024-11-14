#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>


int main(int argc, char **argv)
{
    // Initialize ROS2 node
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("trajectory_visualization");

    // Create a publisher for joint states
    auto publisher = node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

    // Set joint names
    std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

    // Initialize joint state message
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = joint_names;
    joint_state.position.resize(joint_names.size(), 0.0);
    joint_state.velocity.resize(joint_names.size(), 0.0);
    joint_state.effort.resize(joint_names.size(), 0.0);

    int frequency = 100;  // Hz
    double period = 1/static_cast<double>(frequency);  // s
    double max_time = 2.0;  // s
    double factor = 0.5; // rad/s
    double time = 0.0;

    rclcpp::Rate loop_rate(frequency); // publishing rate

    // Loop to create a trajectory where each joint has value q(t) = t * factor
    while (rclcpp::ok()) {
        for (size_t i = 0; i < joint_state.position.size(); ++i) {
            joint_state.position[i] = time * factor;  
        }

        // Set the current time for the joint state
        joint_state.header.stamp = node->get_clock()->now();

        // Publish the joint state
        publisher->publish(joint_state);

        // Update time and sleep
        time += period;

        // Loop time
        if (time > max_time) time = 0.0;

        loop_rate.sleep();
    }

    // Shutdown ROS2
    rclcpp::shutdown();
    return 0;
}
