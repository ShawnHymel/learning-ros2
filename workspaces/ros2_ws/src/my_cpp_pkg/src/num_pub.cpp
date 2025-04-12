#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisherNode : public rclcpp::Node
{
public:
NumberPublisherNode() : Node("number_publisher")
    {

        // Get parameters
        this->declare_parameter("number", 2);
        this->declare_parameter("timer_period", 1.0);

        // Set parameters to member variables
        number_ = this->get_parameter("number").as_int();
        timer_period_ = this->get_parameter("timer_period").as_double();

        // Set parameter callback
        param_cb_handle_ = this->add_post_set_parameters_callback(
            std::bind(&NumberPublisherNode::parametersCallback, this, std::placeholders::_1)
        );

        // Create publisher
        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_),
            std::bind(&NumberPublisherNode::publishNumber, this)
        );
        RCLCPP_INFO(this->get_logger(), "Num Pub Node has been started.");
    }

private:
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        publisher_->publish(msg);
    }

    void parametersCallback(const std::vector<rclcpp::Parameter> &parameters)
    {
        for (const auto &param: parameters) {
            if (param.get_name() == "number") {
                number_ = param.as_int();
            }
            if (param.get_name() == "timer_period") {
                timer_period_ = param.as_double();
                timer_->cancel();
                timer_ = this->create_wall_timer(
                    std::chrono::duration<double>(timer_period_),
                    std::bind(&NumberPublisherNode::publishNumber, this)
                );
            }
        }
    }

    int number_;
    double timer_period_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
