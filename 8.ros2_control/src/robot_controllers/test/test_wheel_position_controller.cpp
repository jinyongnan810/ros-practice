#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "gtest/gtest.h"
#include "hardware_interface/handle.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "robot_controllers/wheel_position_controller.hpp"

using CallbackReturn = controller_interface::CallbackReturn;
using ReturnType = controller_interface::return_type;
using Message = std_msgs::msg::Float64MultiArray;
using namespace std::chrono_literals;

class WheelPositionControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  void SetUp() override
  {
    controller_ = std::make_unique<robot_controllers::WheelPositionController>();
    // The five-argument init overload is supported across Jazzy releases.
    ASSERT_EQ(controller_->init("wheel_joint_controller",
      "<robot name=\"test\"><link name=\"base_link\"/></robot>", 50, "", rclcpp::NodeOptions()),
      ReturnType::OK);
    ASSERT_TRUE(controller_->get_node()->set_parameter(
      rclcpp::Parameter("joints", std::vector<std::string>{"wheel_joint"})).successful);
    ASSERT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
    command_handle_ = std::make_shared<hardware_interface::CommandInterface>(
      "wheel_joint", "position", &command_);
    state_handle_ = std::make_shared<hardware_interface::StateInterface>(
      "wheel_joint", "position", &position_);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    commands.emplace_back(command_handle_, nullptr);
    states.emplace_back(state_handle_, nullptr);
    controller_->assign_interfaces(std::move(commands), std::move(states));
    publisher_node_ = std::make_shared<rclcpp::Node>("controller_test_publisher");
    publisher_ = publisher_node_->create_publisher<Message>("/wheel_joint_controller/commands", 1);
    executor_.add_node(publisher_node_);
    executor_.add_node(controller_->get_node()->get_node_base_interface());
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (publisher_->get_subscription_count() == 0 && std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(10ms);
    }
    ASSERT_GT(publisher_->get_subscription_count(), 0u);
  }

  void TearDown() override
  {
    controller_->on_deactivate(rclcpp_lifecycle::State{});
    controller_->on_cleanup(rclcpp_lifecycle::State{});
    controller_->release_interfaces();
    executor_.remove_node(controller_->get_node()->get_node_base_interface());
    executor_.remove_node(publisher_node_);
    controller_.reset();
  }

  void publish(const std::vector<double> & data)
  {
    Message message;
    message.data = data;
    publisher_->publish(message);
    const auto deadline = std::chrono::steady_clock::now() + 200ms;
    while (std::chrono::steady_clock::now() < deadline)
    {
      executor_.spin_some();
      std::this_thread::sleep_for(1ms);
    }
  }

  ReturnType update()
  {
    return controller_->update(rclcpp::Time(0), rclcpp::Duration::from_seconds(0.02));
  }

  // Handles and backing values must outlive the controller's loaned interfaces.
  double position_{0.3};
  double command_{-1.0};
  std::shared_ptr<hardware_interface::CommandInterface> command_handle_;
  std::shared_ptr<hardware_interface::StateInterface> state_handle_;
  std::unique_ptr<robot_controllers::WheelPositionController> controller_;
  rclcpp::Node::SharedPtr publisher_node_;
  rclcpp::Publisher<Message>::SharedPtr publisher_;
  rclcpp::executors::SingleThreadedExecutor executor_;
};

TEST_F(WheelPositionControllerTest, PluginIsDiscoverable)
{
  pluginlib::ClassLoader<controller_interface::ControllerInterface> loader(
    "controller_interface", "controller_interface::ControllerInterface");
  EXPECT_TRUE(loader.isClassAvailable("robot_controllers/WheelPositionController"));
  auto plugin = loader.createSharedInstance("robot_controllers/WheelPositionController");
  EXPECT_NE(plugin, nullptr);
}

TEST_F(WheelPositionControllerTest, HoldsMeasuredPositionAndForwardsValidCommands)
{
  EXPECT_EQ(controller_->command_interface_configuration().names,
    std::vector<std::string>{"wheel_joint/position"});
  EXPECT_EQ(controller_->state_interface_configuration().names,
    std::vector<std::string>{"wheel_joint/position"});
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  EXPECT_DOUBLE_EQ(command_, position_);
  ASSERT_EQ(update(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(command_, position_);
  publish({0.8});
  ASSERT_EQ(update(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(command_, 0.8);
  ASSERT_EQ(update(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(command_, 0.8);
}

TEST_F(WheelPositionControllerTest, RejectsMalformedCommandsWithoutChangingTarget)
{
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  publish({0.8});
  ASSERT_EQ(update(), ReturnType::OK);
  for (const auto & data : std::vector<std::vector<double>>{
      {}, {0.1, 0.2}, {std::numeric_limits<double>::quiet_NaN()},
      {std::numeric_limits<double>::infinity()}})
  {
    publish(data);
    EXPECT_EQ(update(), ReturnType::OK);
    EXPECT_DOUBLE_EQ(command_, 0.8);
  }
}

TEST_F(WheelPositionControllerTest, ReactivationDiscardsPreviousAndInactiveTargets)
{
  publish({1.0});  // Inactive commands are ignored.
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  ASSERT_EQ(update(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(command_, position_);
  publish({0.8});
  ASSERT_EQ(update(), ReturnType::OK);
  ASSERT_EQ(controller_->on_deactivate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  publish({-0.8});
  position_ = -0.2;
  ASSERT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::SUCCESS);
  ASSERT_EQ(update(), ReturnType::OK);
  EXPECT_DOUBLE_EQ(command_, -0.2);
}

TEST_F(WheelPositionControllerTest, RejectsInvalidConfigurationOrMissingFeedback)
{
  position_ = std::numeric_limits<double>::quiet_NaN();
  EXPECT_EQ(controller_->on_activate(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  for (const auto & joints : std::vector<std::vector<std::string>>{
      {}, {""}, {"wheel_joint", "arm_joint_1"}})
  {
    controller_->get_node()->set_parameter(rclcpp::Parameter("joints", joints));
    EXPECT_EQ(controller_->on_configure(rclcpp_lifecycle::State{}), CallbackReturn::ERROR);
  }
}
