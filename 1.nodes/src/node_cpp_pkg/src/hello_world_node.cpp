#include "rclcpp/rclcpp.hpp"

class HelloWorldNode : public rclcpp::Node
{
public:
    HelloWorldNode() : Node("hello_world_node")
    {
        // Initialize the timer
        timer_ = nullptr;
        log_startup_message();
        create_periodic_timer();
    }

private:
    // Timer to periodically log a message
    rclcpp::TimerBase::SharedPtr timer_;
    // Counter to keep track of the number of periodic messages logged
    int counter_ = 0;
    // Log a message to indicate that the node has started
    void log_startup_message()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, World! Node has started.");
    }

    // Create a timer to periodically log a message
    void create_periodic_timer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                log_periodic_message();
            });
    }

    // Log a periodic message
    void log_periodic_message()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, World! This is a periodic message. Counter: %d", counter_++);
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<HelloWorldNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
