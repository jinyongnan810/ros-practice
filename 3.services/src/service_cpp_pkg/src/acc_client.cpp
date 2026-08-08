#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/srv/acc.hpp"

class AccClientNode : public rclcpp::Node
{
public:
    AccClientNode() : Node("acc_client_node")
    {
        // Create a client for the "accumulate" service
        client_ = this->create_client<custom_interfaces::srv::Acc>("accumulate");
    }
    rclcpp::Client<custom_interfaces::srv::Acc>::SharedPtr client_;
    void add(int64_t a, int64_t b, int64_t c)
    {
        // Create a request
        auto request = std::make_shared<custom_interfaces::srv::Acc::Request>();
        request->a = a;
        request->b = b;
        request->c = c;

        // Wait for the service to be available
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
        }

        // Process the response after the executor completes the request.
        client_->async_send_request(
            request,
            [this, request](rclcpp::Client<custom_interfaces::srv::Acc>::SharedFuture future)
            {
                response_callback(future, request);
            });
    }

private:
    void response_callback(
        rclcpp::Client<custom_interfaces::srv::Acc>::SharedFuture future,
        custom_interfaces::srv::Acc::Request::SharedPtr request)
    {
        try
        {
            auto response = future.get();
            RCLCPP_INFO(
                this->get_logger(),
                "Result of add(%ld, %ld, %ld): %ld",
                request->a,
                request->b,
                request->c,
                response->sum);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
        }
    }
};

int main(int argc, char *argv[])
{
    // Initialize the ROS 2 client library
    rclcpp::init(argc, argv);

    // Create a custom node
    auto node = std::make_shared<AccClientNode>();
    // Call the service with some example values
    node->add(1, 2, 3);
    node->add(4, 5, 6);
    node->add(7, 8, 9);
    // Keep the node alive until it is shut down
    rclcpp::spin(node);
    // Destroy the node
    node.reset();

    // Shutdown the ROS 2 client library
    rclcpp::shutdown();
    return 0;
}
