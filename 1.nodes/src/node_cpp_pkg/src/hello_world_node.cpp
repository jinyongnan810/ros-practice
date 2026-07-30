#include "rclcpp/rclcpp.hpp"

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a node named "hello_world_node"
    auto node = std::make_shared<rclcpp::Node>("hello_world_node");
    // Log a message to indicate that the node has started
    RCLCPP_INFO(node->get_logger(), "Hello, World! Node has started.");
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
