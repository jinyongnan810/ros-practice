#include <chrono>
#include <memory>
#include <string>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class SensorStation : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit SensorStation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : rclcpp_lifecycle::LifecycleNode("sensor_station", options), count_(0)
  {
    // Declare parameters in constructor
    this->declare_parameter<std::string>("sensor_name", "sensor_station");
    this->declare_parameter<std::string>("topic_name", "sensor_data");
    this->declare_parameter<double>("publish_frequency", 1.0);

    RCLCPP_INFO(
      get_logger(),
      "[%s] Constructed in UNCONFIGURED state.",
      get_name());
  }

  // 1. Transition: Unconfigured -> Configuring -> Inactive
  CallbackReturn on_configure(const rclcpp_lifecycle::State & /*state*/) override
  {
    sensor_name_ = this->get_parameter("sensor_name").as_string();
    topic_name_ = this->get_parameter("topic_name").as_string();
    publish_frequency_ = this->get_parameter("publish_frequency").as_double();

    if (publish_frequency_ <= 0.0) {
      RCLCPP_ERROR(get_logger(), "[%s] Publish frequency must be > 0.0!", get_name());
      return CallbackReturn::FAILURE;
    }

    // Create Lifecycle Publisher (starts in inactive state by default)
    pub_ = this->create_publisher<std_msgs::msg::String>(topic_name_, 10);

    // Create periodic timer
    auto timer_period = std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = this->create_wall_timer(
      timer_period,
      std::bind(&SensorStation::timer_callback, this));

    RCLCPP_INFO(
      get_logger(),
      "[%s] on_configure: Initialized publisher on topic '%s' at %.1f Hz. State is now INACTIVE.",
      get_name(), topic_name_.c_str(), publish_frequency_);

    return CallbackReturn::SUCCESS;
  }

  // 2. Transition: Inactive -> Activating -> Active
  CallbackReturn on_activate(const rclcpp_lifecycle::State & /*state*/) override
  {
    // Explicitly activate the lifecycle publisher so it starts transmitting
    if (pub_) {
      pub_->on_activate();
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] on_activate: LifecyclePublisher is now ACTIVE and broadcasting messages.",
      get_name());

    return CallbackReturn::SUCCESS;
  }

  // 3. Transition: Active -> Deactivating -> Inactive
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & /*state*/) override
  {
    // Explicitly deactivate lifecycle publisher so it stops transmitting
    if (pub_) {
      pub_->on_deactivate();
    }

    RCLCPP_INFO(
      get_logger(),
      "[%s] on_deactivate: LifecyclePublisher is now INACTIVE (messages suppressed).",
      get_name());

    return CallbackReturn::SUCCESS;
  }

  // 4. Transition: Inactive -> CleaningUp -> Unconfigured
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & /*state*/) override
  {
    timer_.reset();
    pub_.reset();
    count_ = 0;

    RCLCPP_INFO(
      get_logger(),
      "[%s] on_cleanup: Destroyed publisher and timer. State is now UNCONFIGURED.",
      get_name());

    return CallbackReturn::SUCCESS;
  }

  // 5. Transition: Any state -> ShuttingDown -> Finalized
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & /*state*/) override
  {
    timer_.reset();
    pub_.reset();

    RCLCPP_INFO(
      get_logger(),
      "[%s] on_shutdown: Resources cleaned up. State is now FINALIZED.",
      get_name());

    return CallbackReturn::SUCCESS;
  }

  // 6. Transition: Error occurred -> ErrorProcessing
  CallbackReturn on_error(const rclcpp_lifecycle::State & /*state*/) override
  {
    RCLCPP_ERROR(get_logger(), "[%s] on_error: Error detected, recovering...", get_name());
    timer_.reset();
    pub_.reset();
    return CallbackReturn::SUCCESS;
  }

private:
  void timer_callback()
  {
    std_msgs::msg::String msg;
    msg.data = "[" + sensor_name_ + "] reading #" + std::to_string(count_++);

    // LifecyclePublisher::is_activated() returns true only when in ACTIVE state
    if (pub_ && pub_->is_activated()) {
      RCLCPP_INFO(get_logger(), "[ACTIVE] Publishing -> '%s'", msg.data.c_str());
      pub_->publish(msg);
    } else {
      RCLCPP_INFO_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "[%s - INACTIVE] Timer firing, but LifecyclePublisher is disabled (messages suppressed)",
        get_name());
    }
  }

  std::string sensor_name_;
  std::string topic_name_;
  double publish_frequency_{1.0};
  uint64_t count_{0};

  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SensorStation>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
