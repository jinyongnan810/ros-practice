#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/news.hpp"
// #include "std_msgs/msg/string.hpp"

#include <chrono>
#include <ctime>
#include <iomanip>
#include <stdexcept>
#include <sstream>

class NewsStationNode : public rclcpp::Node
{
public:
    NewsStationNode() : Node("news_station_node")
    {
        const auto timer_interval = this->declare_parameter<double>("timer_interval", 1.0);
        if (timer_interval <= 0.0)
        {
            throw std::invalid_argument("timer_interval must be greater than 0 seconds");
        }
        // Initialize the timer
        timer_ = nullptr;
        // Initialize the publisher
        log_startup_message();
        // Create a periodic timer to publish messages
        create_periodic_timer(timer_interval);
    }

private:
    // Timer to periodically publish a message
    rclcpp::TimerBase::SharedPtr timer_;
    // Publisher to publish messages
    rclcpp::Publisher<custom_interfaces::msg::News>::SharedPtr publisher_ = this->create_publisher<custom_interfaces::msg::News>("news", 10);
    // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_ = this->create_publisher<std_msgs::msg::String>("news", 10);
    // Counter for message published
    int counter_ = 0;
    // Log a message to indicate that the node has started
    void log_startup_message()
    {
        RCLCPP_INFO(this->get_logger(), "Hello, World! News Station Node has started.");
    }

    // Create a timer to periodically publish a message
    void create_periodic_timer(double timer_interval)
    {
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_interval),
            [this]()
            {
                publish_periodic_message();
            });
    }

    // publish a periodic message
    void publish_periodic_message()
    {
        // Create a message to publish
        auto message = std::make_shared<custom_interfaces::msg::News>();
        // auto message = std::make_shared<std_msgs::msg::String>();
        const auto now = std::time(nullptr);
        std::ostringstream datetime;
        datetime << std::put_time(std::localtime(&now), "%Y-%m-%dT%H:%M:%S");
        message->datetime = datetime.str();
        message->title = "News update";
        message->content = "Counter: " + std::to_string(counter_++);
        // message->data = "News update! Counter: " + std::to_string(counter_++);
        // Publish the message
        publisher_->publish(*message);
        RCLCPP_INFO(
            this->get_logger(),
            "Published [%s] %s: %s",
            message->datetime.c_str(),
            message->title.c_str(),
            message->content.c_str());
        // RCLCPP_INFO(this->get_logger(), "Published: '%s'", message->data.c_str());
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
