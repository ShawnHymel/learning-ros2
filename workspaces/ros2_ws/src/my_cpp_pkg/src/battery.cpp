#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery")
    {
        client_ = this->create_client<my_robot_interfaces::srv::SetLed>("set_led");
        timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&BatteryNode::updateLedPanel, this)
        );
    }

private:
    void updateLedPanel()
    {
        // Check if server is ready
        if (!client_->service_is_ready())
        {
            RCLCPP_WARN(this->get_logger(), "LED Panel server not ready");
            return;
        }

        // Update LED state and restart timer
        led_state_ = !led_state_;
        timer_->cancel();
        if (led_state_) {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(6),
                std::bind(&BatteryNode::updateLedPanel, this)
            );
        } else {
            timer_ = this->create_wall_timer(
                std::chrono::seconds(2),
                std::bind(&BatteryNode::updateLedPanel, this)
            );
        }

        // Debug
        RCLCPP_INFO(this->get_logger(), "Setting LED to: %d", led_state_);

        // Construct request
        auto request = std::make_shared<my_robot_interfaces::srv::SetLed::Request>();
        request->led_number = 3;
        request->state = led_state_;

        // Send request
        client_->async_send_request(
            request,
            std::bind(
                &BatteryNode::callbackSetLed,
                this,
                std::placeholders::_1
            )
        );
    }

    void callbackSetLed(rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Success: %d", (bool)response->success);
    }

    rclcpp::Client<my_robot_interfaces::srv::SetLed>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool led_state_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
