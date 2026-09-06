#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TurtleActionClientNode : public rclcpp::Node
{
public:
    TurtleActionClientNode() : Node("turtle_action_client")
    {
        RCLCPP_INFO(this->get_logger(), "Turtle Action Client Node has been started (no actions yet)!");
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<TurtleActionClientNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
