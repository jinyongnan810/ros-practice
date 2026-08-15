#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include "chasing_interfaces/msg/target.hpp"
#include "chasing_interfaces/msg/target_positions.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"

class RandomTurtleSpawner : public rclcpp::Node
{
public:
    RandomTurtleSpawner()
        : Node("random_turtle_spawner"), random_engine_(std::random_device{}()),
          position_distribution_(1.0F, 10.0F), angle_distribution_(0.0F, 6.2831853F)
    {
        const auto duration = declare_parameter<double>("duration", 2.0);
        if (duration <= 0.0)
        {
            throw std::invalid_argument("duration must be greater than zero");
        }

        spawn_client_ = create_client<turtlesim::srv::Spawn>("/spawn");
        kill_client_ = create_client<turtlesim::srv::Kill>("/kill");
        positions_publisher_ =
            create_publisher<chasing_interfaces::msg::TargetPositions>(
                "/spawned_target_positions", 10);
        pose_subscription_ = create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10,
            std::bind(&RandomTurtleSpawner::handle_pose, this, std::placeholders::_1));
        timer_ = create_wall_timer(
            std::chrono::duration<double>(duration),
            std::bind(&RandomTurtleSpawner::spawn_turtle, this));

        RCLCPP_INFO(get_logger(), "Spawning a random turtle every %.2f seconds", duration);
    }

private:
    void spawn_turtle()
    {
        if (!spawn_client_->service_is_ready())
        {
            RCLCPP_WARN(get_logger(), "/spawn service is not available yet");
            return;
        }

        if (request_pending_)
        {
            RCLCPP_WARN(get_logger(), "Previous spawn request is still pending");
            return;
        }

        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->x = position_distribution_(random_engine_);
        request->y = position_distribution_(random_engine_);
        request->theta = angle_distribution_(random_engine_);
        request_pending_ = true;

        spawn_client_->async_send_request(
            request,
            [this, request](rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future)
            {
                request_pending_ = false;
                const auto response = future.get();
                chasing_interfaces::msg::Target target;
                target.name = response->name;
                target.position.x = request->x;
                target.position.y = request->y;
                target_positions_.targets.push_back(target);
                positions_publisher_->publish(target_positions_);
                RCLCPP_INFO(
                    get_logger(), "Spawned %s at (%.2f, %.2f)", response->name.c_str(), request->x,
                    request->y);
            });
    }

    void handle_pose(const turtlesim::msg::Pose::SharedPtr pose)
    {
        if (!kill_client_->service_is_ready())
        {
            return;
        }

        for (const auto &target : target_positions_.targets)
        {
            const auto delta_x = pose->x - target.position.x;
            const auto delta_y = pose->y - target.position.y;
            if (delta_x * delta_x + delta_y * delta_y > 0.2 * 0.2 ||
                pending_kills_.count(target.name) > 0)
            {
                continue;
            }

            auto request = std::make_shared<turtlesim::srv::Kill::Request>();
            request->name = target.name;
            pending_kills_.insert(target.name);
            kill_client_->async_send_request(
                request,
                [this, name = target.name](rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future)
                {
                    try
                    {
                        future.get();
                    }
                    catch (const std::exception &error)
                    {
                        pending_kills_.erase(name);
                        RCLCPP_ERROR(
                            get_logger(), "Failed to clear target %s: %s", name.c_str(),
                            error.what());
                        return;
                    }

                    pending_kills_.erase(name);
                    auto &targets = target_positions_.targets;
                    targets.erase(
                        std::remove_if(
                            targets.begin(), targets.end(),
                            [&name](const auto &target)
                            { return target.name == name; }),
                        targets.end());
                    positions_publisher_->publish(target_positions_);
                    RCLCPP_INFO(get_logger(), "Cleared target %s", name.c_str());
                });
        }
    }

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
    rclcpp::Publisher<chasing_interfaces::msg::TargetPositions>::SharedPtr positions_publisher_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    chasing_interfaces::msg::TargetPositions target_positions_;
    std::unordered_set<std::string> pending_kills_;
    std::mt19937 random_engine_;
    std::uniform_real_distribution<float> position_distribution_;
    std::uniform_real_distribution<float> angle_distribution_;
    bool request_pending_{false};
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RandomTurtleSpawner>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}