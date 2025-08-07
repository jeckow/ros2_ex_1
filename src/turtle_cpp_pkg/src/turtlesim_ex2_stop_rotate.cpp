#include "rclcpp/rclcpp.hpp"
#include "turtle_interfaces/srv/rotate_status.hpp"
#include "turtle_interfaces/msg/rotate_status.hpp"

class turtlesim_ex2_stop_rotate_Node : public rclcpp::Node
{

public:
    turtlesim_ex2_stop_rotate_Node() : Node("turtlesim_ex2_stop_rotate")
    {
        server_ = this->create_service<turtle_interfaces::srv::RotateStatus>("turtlesim_ex2_stop_rotate", 
            [this](const turtle_interfaces::srv::RotateStatus::Request::SharedPtr request,
                   const turtle_interfaces::srv::RotateStatus::Response::SharedPtr response)
                {
                        this->callbackRotateStatus(request, response);
                }
        );
        
        publisher_ = this->create_publisher<turtle_interfaces::msg::RotateStatus>("/rotate_status", 10);
        RCLCPP_INFO(this->get_logger(), "Turtle rotation toggler on..");
    }


private:
    rclcpp::Service<turtle_interfaces::srv::RotateStatus>::SharedPtr server_;
    rclcpp::Publisher<turtle_interfaces::msg::RotateStatus>::SharedPtr publisher_;

    void callbackRotateStatus(const turtle_interfaces::srv::RotateStatus::Request::SharedPtr request,
                              const turtle_interfaces::srv::RotateStatus::Response::SharedPtr response)
    {
        auto msg = turtle_interfaces::msg::RotateStatus();

        if (request->rotation_status == true)
        {
            response->toggle = false;
            response->message = "Turtle rotation on";
        }

        else if (request->rotation_status == false)
        {
            response->toggle = true;
            response->message = "Turtle rotation off";
        }

        // publish the service response to a topic that we can subscribe to
        msg.toggle = response->toggle;
        msg.message = response->message;
        publisher_->publish(msg);

        RCLCPP_INFO(this->get_logger(), "Rotation: %d (%s)", response->toggle, response->message.c_str());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtlesim_ex2_stop_rotate_Node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}