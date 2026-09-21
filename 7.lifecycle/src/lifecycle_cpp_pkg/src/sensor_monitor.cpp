#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class SensorMonitor : public rclcpp::Node
{
public:
  SensorMonitor() : Node("sensor_monitor"), received_count_(0)
  {
    this->declare_parameter<std::string>("topic_name", "sensor_data");
    std::string topic_name = this->get_parameter("topic_name").as_string();

    sub_ = this->create_subscription<std_msgs::msg::String>(
      topic_name,
      10,
      std::bind(&SensorMonitor::topic_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "SensorMonitor started. Listening to '%s' for lifecycle publisher stream...",
      topic_name.c_str());
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    received_count_++;
    RCLCPP_INFO(
      get_logger(),
      "[MONITOR #%lu] Live message received: '%s'",
      received_count_, msg->data.c_str());
  }

  uint64_t received_count_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
