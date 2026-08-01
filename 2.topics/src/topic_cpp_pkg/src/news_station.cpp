#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class NewsStationNode : public rclcpp::Node
{
public:
    NewsStationNode() : Node("news_station_node")
    {
        // Initialize the timer
        timer_ = nullptr;
        // Initialize the publisher
        log_startup_message();
        // Create a periodic timer to publish a message every second
        create_periodic_timer();
    }

private:
    // Timer to periodically publish a message
    rclcpp::TimerBase::SharedPtr timer_;
    // Publisher to publish messages
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = this->create_publisher<std_msgs::msg::String>("news", 10);
    // Counter for message published
    int counter_ = 0;
    // Log a message to indicate that the node has started
    void log_startup_message()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, World! News Station Node has started.");
    }

    // Create a timer to periodically publish a message
    void create_periodic_timer()
    {
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]()
            {
                publish_periodic_message();
            });
    }

    // publish a periodic message
    void publish_periodic_message()
    {
        // Create a message to publish
        auto message = std::make_shared<std_msgs::msg::String>();
        message->data = "News update! Counter: " + std::to_string(counter_++);
        // Publish the message
        publisher_->publish(*message);
        RCLCPP_INFO(this->get_logger(), "Published: '%s'", message->data.c_str());
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<NewsStationNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
