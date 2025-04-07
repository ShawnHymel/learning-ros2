#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumCounterNode : public rclcpp::Node
{
public:
    NumCounterNode() : Node("number_counter")
    {
        counter_ = 0;
        subscriber_ = this->create_subscription<example_interfaces::msg::Int64>(
            "number", 
            10,
            std::bind(&NumCounterNode::numberCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number_count", 10);
        RCLCPP_INFO(this->get_logger(), "NumCounter Node has been started.");
    }

private:
    void numberCallback(const example_interfaces::msg::Int64::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%li", msg->data);

        // Update counter
        counter_ += msg->data;

        // Wrap counter number in message interface
        auto msg_count = example_interfaces::msg::Int64();
        msg_count.data = counter_;

        // Publish message
        publisher_->publish(msg_count);
    }

    int64_t counter_;
    rclcpp::Subscription<example_interfaces::msg::Int64>::SharedPtr subscriber_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumCounterNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
