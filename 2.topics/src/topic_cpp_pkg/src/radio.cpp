#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/news.hpp"
// #include "std_msgs/msg/string.hpp"

class RadioNode : public rclcpp::Node
{
public:
    RadioNode() : Node("radio_node")
    {
        log_startup_message();
        // Create a subscription to receive messages
        subscription_ = this->create_subscription<custom_interfaces::msg::News>(
            "news", 10,
            [this](const custom_interfaces::msg::News::SharedPtr message)
            {
                receive_periodic_message(message);
            });
        // subscription_ = this->create_subscription<std_msgs::msg::String>(
        //     "news", 10,
        //     [this](const std_msgs::msg::String::SharedPtr message)
        //     {
        //         receive_periodic_message(message);
        //     });
    }

private:
    // Subscription to receive messages
    rclcpp::Subscription<custom_interfaces::msg::News>::SharedPtr subscription_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // Log a message to indicate that the node has started
    void log_startup_message()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, World! Radio Node has started.");
    }
    // Receive a periodic message
    void receive_periodic_message(const custom_interfaces::msg::News::SharedPtr message)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received [%s] %s: %s",
            message->datetime.c_str(),
            message->title.c_str(),
            message->content.c_str());
    }
    // void receive_periodic_message(const std_msgs::msg::String::SharedPtr message)
    // {
    //     RCLCPP_INFO(this->get_logger(), "Received: '%s'", message->data.c_str());
    // }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<RadioNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
