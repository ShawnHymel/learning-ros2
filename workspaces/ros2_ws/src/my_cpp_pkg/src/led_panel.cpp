#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"
#include "my_robot_interfaces/msg/led_panel_state.hpp"

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        server_ = this->create_service<my_robot_interfaces::srv::SetLed>(
            "set_led",
            std::bind(
                &LedPanelNode::callbackSetLed,
                this,
                std::placeholders::_1,
                std::placeholders::_2
            )
        );
        publisher_ = this->create_publisher<my_robot_interfaces::msg::LedPanelState>(
            "led_panel_state",
            10
        );
        RCLCPP_INFO(this->get_logger(), "LED Panel service started");
    }

private:
    void callbackSetLed(
        const my_robot_interfaces::srv::SetLed::Request::SharedPtr request,
        const my_robot_interfaces::srv::SetLed::Response::SharedPtr response)
    {
        int led = request->led_number;
        bool state = request->state;

        // Set the appropriate LED
        response->success = true;
        switch (led) {
            case 1:
                led_state_1 = state;
                break;
            case 2:
                led_state_2 = state;
                break;
            case 3:
                led_state_3 = state;
                break;
            default:
                response->success = false;
                break;
        }

        // Print some debugging info
        if (response->success)
        {
            RCLCPP_INFO(this->get_logger(), "LED %d set to %d", led, state);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unsupported LED number");
        }

        // Publish panel state
        auto msg = my_robot_interfaces::msg::LedPanelState();
        msg.led_1 = led_state_1;
        msg.led_2 = led_state_2;
        msg.led_3 = led_state_3;
        publisher_->publish(msg);
    }

    rclcpp::Service<my_robot_interfaces::srv::SetLed>::SharedPtr server_;
    rclcpp::Publisher<my_robot_interfaces::msg::LedPanelState>::SharedPtr publisher_;
    bool led_state_1 = false;
    bool led_state_2 = false;
    bool led_state_3 = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
