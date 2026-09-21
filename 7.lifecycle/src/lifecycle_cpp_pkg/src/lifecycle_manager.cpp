#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

class LifecycleManager : public rclcpp::Node
{
public:
  LifecycleManager() : Node("lifecycle_manager")
  {
    this->declare_parameter<std::vector<std::string>>(
      "managed_nodes",
      std::vector<std::string>{"sensor_station_1", "sensor_station_2"});

    managed_nodes_ = this->get_parameter("managed_nodes").as_string_array();

    // Sanitize node names (strip whitespace, quotes, and brackets)
    std::vector<std::string> sanitized_nodes;
    for (const auto & raw_name : managed_nodes_) {
      auto start = raw_name.find_first_not_of(" \t\n\r\'\"[]");
      if (start != std::string::npos) {
        auto end = raw_name.find_last_not_of(" \t\n\r\'\"[]");
        sanitized_nodes.push_back(raw_name.substr(start, end - start + 1));
      }
    }
    managed_nodes_ = std::move(sanitized_nodes);

    RCLCPP_INFO(
      get_logger(),
      "LifecycleManager initialized. Managing %zu nodes:",
      managed_nodes_.size());
    for (const auto & name : managed_nodes_) {
      RCLCPP_INFO(get_logger(), "  - /%s", name.c_str());
    }

    // Launch management sequence in worker thread to avoid blocking ROS executor
    manager_thread_ = std::thread(&LifecycleManager::run_orchestration, this);
  }

  ~LifecycleManager()
  {
    if (manager_thread_.joinable()) {
      manager_thread_.join();
    }
  }

private:
  void run_orchestration()
  {
    // Wait for nodes and services to become available
    RCLCPP_INFO(get_logger(), "==> Step 0: Waiting for lifecycle services for all nodes...");
    for (const auto & node_name : managed_nodes_) {
      auto client_change = this->create_client<lifecycle_msgs::srv::ChangeState>(
        "/" + node_name + "/change_state");
      auto client_get = this->create_client<lifecycle_msgs::srv::GetState>(
        "/" + node_name + "/get_state");

      while (!client_change->wait_for_service(2s)) {
        if (!rclcpp::ok()) {return;}
        RCLCPP_INFO(get_logger(), "Waiting for /%s/change_state...", node_name.c_str());
      }
      while (!client_get->wait_for_service(2s)) {
        if (!rclcpp::ok()) {return;}
        RCLCPP_INFO(get_logger(), "Waiting for /%s/get_state...", node_name.c_str());
      }
    }

    // Initial inspection
    print_current_states("Initial State");
    std::this_thread::sleep_for(2s);

    // Step 1: Configure all nodes
    RCLCPP_INFO(
      get_logger(),
      "==> Step 1: CONFIGURE all nodes (allocating publishers, timers, parameters)...");
    change_state_all(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure");
    print_current_states("After Configure");

    RCLCPP_INFO(
      get_logger(),
      "[NOTE] Nodes are now INACTIVE. Timers are running, but LifecyclePublishers are inactive.");
    RCLCPP_INFO(get_logger(), "[NOTE] Monitor receives NO messages during this 4-second period.");
    std::this_thread::sleep_for(4s);

    // Step 2: Synchronously activate all nodes
    RCLCPP_INFO(
      get_logger(),
      "==> Step 2: SIMULTANEOUS ACTIVATION of all managed nodes!");
    change_state_all(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate");
    print_current_states("After Simultaneous Activation");

    RCLCPP_INFO(
      get_logger(),
      "[NOTE] All nodes are now ACTIVE. Both stations are actively publishing messages simultaneously!");
    std::this_thread::sleep_for(8s);

    // Step 3: Synchronously deactivate all nodes
    RCLCPP_INFO(
      get_logger(),
      "==> Step 3: SIMULTANEOUS DEACTIVATION of all managed nodes!");
    change_state_all(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE, "deactivate");
    print_current_states("After Simultaneous Deactivation");

    RCLCPP_INFO(
      get_logger(),
      "[NOTE] Nodes returned to INACTIVE. Message streams stop immediately while nodes remain alive.");
    std::this_thread::sleep_for(4s);

    // Step 4: Cleanup
    RCLCPP_INFO(get_logger(), "==> Step 4: CLEANUP all nodes (freeing publishers and timers)...");
    change_state_all(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP, "cleanup");
    print_current_states("After Cleanup");
    std::this_thread::sleep_for(2s);

    // Step 5: Shutdown
    RCLCPP_INFO(get_logger(), "==> Step 5: SHUTDOWN all nodes to finalized state...");
    change_state_all(
      lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN,
      "shutdown");
    print_current_states("After Shutdown");

    RCLCPP_INFO(
      get_logger(),
      "==> Coordinated multi-node lifecycle orchestration completed successfully!");
  }

  bool change_state(const std::string & node_name, uint8_t transition_id, const std::string & transition_name)
  {
    auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
      "/" + node_name + "/change_state");
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition_id;
    request->transition.label = transition_name;

    auto future = client->async_send_request(request);
    if (future.wait_for(5s) == std::future_status::ready) {
      auto result = future.get();
      if (result->success) {
        RCLCPP_INFO(
          get_logger(),
          "  [SUCCESS] Node /%s transitioned via '%s'",
          node_name.c_str(), transition_name.c_str());
        return true;
      }
    }
    RCLCPP_ERROR(
      get_logger(),
      "  [FAILED] Node /%s failed to transition via '%s'",
      node_name.c_str(), transition_name.c_str());
    return false;
  }

  void change_state_all(uint8_t transition_id, const std::string & transition_name)
  {
    for (const auto & node_name : managed_nodes_) {
      change_state(node_name, transition_id, transition_name);
    }
  }

  std::string get_state(const std::string & node_name)
  {
    auto client = this->create_client<lifecycle_msgs::srv::GetState>(
      "/" + node_name + "/get_state");
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    auto future = client->async_send_request(request);
    if (future.wait_for(3s) == std::future_status::ready) {
      return future.get()->current_state.label;
    }
    return "UNKNOWN";
  }

  void print_current_states(const std::string & header)
  {
    RCLCPP_INFO(get_logger(), "--- State Check [%s] ---", header.c_str());
    for (const auto & node_name : managed_nodes_) {
      std::string state = get_state(node_name);
      RCLCPP_INFO(get_logger(), "    /%s is in state: [%s]", node_name.c_str(), state.c_str());
    }
  }

  std::vector<std::string> managed_nodes_;
  std::thread manager_thread_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<LifecycleManager>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
