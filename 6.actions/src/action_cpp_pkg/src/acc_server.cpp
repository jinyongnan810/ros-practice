#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class AccServerNode : public rclcpp::Node
{
public:
    AccServerNode() : Node("acc_server_node")
    {
        RCLCPP_INFO(this->get_logger(), "Accumulate Action Server Node has been started!");
        // Create an action server
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<AccServerNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
