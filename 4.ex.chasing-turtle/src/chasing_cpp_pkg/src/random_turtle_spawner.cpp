#include <chrono>
#include <functional>
#include <memory>
#include <random>
#include <stdexcept>

#include "chasing_interfaces/msg/target.hpp"
#include "chasing_interfaces/msg/target_positions.hpp"
#include "rclcpp/rclcpp.hpp"
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
        positions_publisher_ =
            create_publisher<chasing_interfaces::msg::TargetPositions>(
                "/spawned_target_positions", 10);
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

    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Publisher<chasing_interfaces::msg::TargetPositions>::SharedPtr positions_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    chasing_interfaces::msg::TargetPositions target_positions_;
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