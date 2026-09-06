#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

class TurtleActionServerNode : public rclcpp::Node
{
public:
    TurtleActionServerNode() : Node("turtle_action_server")
    {
        RCLCPP_INFO(this->get_logger(), "Turtle Action Server Node has been started (no actions yet)!");
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<TurtleActionServerNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
