#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/acc.hpp"

class AccServerNode : public rclcpp::Node
{
public:
    AccServerNode() : Node("acc_server_node")
    {
        // Create a service
        service_ = this->create_service<custom_interfaces::srv::Acc>(
            "accumulate",
            std::bind(&AccServerNode::handle_accumulate, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    void handle_accumulate(
        const std::shared_ptr<custom_interfaces::srv::Acc::Request> request,
        std::shared_ptr<custom_interfaces::srv::Acc::Response> response)
    {
        response->sum = request->a + request->b + request->c;
        RCLCPP_INFO(this->get_logger(), "Incoming request: a=%ld, b=%ld, c=%ld", request->a, request->b, request->c);
    }

    rclcpp::Service<custom_interfaces::srv::Acc>::SharedPtr service_;
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<AccServerNode>();
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
