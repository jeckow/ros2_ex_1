#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/set_pen.hpp"                // for toggling the pen
#include "turtlesim/srv/teleport_absolute.hpp"      // for drawing the lines
#include "turtlesim/msg/pose.hpp"                   // for checking position of turtle1
#include "geometry_msgs/msg/twist.hpp"              // for rotating turtle2 in place

using namespace std::chrono_literals;       // para rekta sulat na lang kung ilang seconds

class turtlesim_ex1_node : public rclcpp::Node
{
public:
    turtlesim_ex1_node() : Node("turtlesim_ex1")
    {
        // create pen client that will execute the toggling status of the pen (kung nakaangat ba or hindi pag magddrawing)
        pen_client_ = this->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");

        // create a teleport client that will execute the drawing of the lines when the pen is down
        teleport_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        // publish to rotate turtle
        rotate_turt_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle2/cmd_vel", 10);
        rotate_turt.angular.z = 0.0;
        rotate_turt_pub_->publish(rotate_turt);

        // subscribe to turtle2's position via pose
        turtle2_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle2/pose",
            10,
            [this](turtlesim::msg::Pose::SharedPtr turtle2_pose)
            {
                rotate_turtle2_callback(turtle2_pose);                                          // call back for detecting and rotating
            }
        );

        // subscribe to turtle1's position via pose
        turtle1_pose_sub_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",
            10,
            [this](turtlesim::msg::Pose::SharedPtr turtle1_pose)
            {
                get_turtle2_bounds(turtle1_pose);                                          // call back for detecting and rotating
            }
        );

        // Wait for following services to be available, which comes from the turtle1
        while (!pen_client_->wait_for_service(1s) ||
               !teleport_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for turtlesim services...");
        }
    }


    // draw the quadrant lines by defining the coordinates
    void draw_quadrants()
    {
        // draw horizontal line
        toggle_pen(false);
        teleport_pen(0.0, 5.5, 0.0);
        toggle_pen(true);
        teleport_pen(11.0, 5.5, 0.0);
        
        // draw vertical line
        toggle_pen(false);
        teleport_pen(5.5, 0.0, 0.0);
        toggle_pen(true);
        teleport_pen(5.5, 11.0, 0.0);

        // turn off pen
        toggle_pen(0.0);

        RCLCPP_INFO(get_logger(), "Drawing quadrant drawn...");
    }


    // position turtle1 to quadrant 3
    void position_turtle1()
    {
        set_position(2.75, 8.25, 0.0);
    }

    
private:
    // initialize private variables
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr teleport_client_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle1_pose_sub_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr turtle2_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr rotate_turt_pub_;
    geometry_msgs::msg::Twist rotate_turt;
    int quadrant;


    // toggle the pen client to turn on and off
    void toggle_pen(bool status)
    {
        // create a shared pointer named pen_status that points to the instance of the message template for defining the properties of the pen
        auto pen_status = std::make_shared<turtlesim::srv::SetPen::Request>();

        // white color axes lines
        pen_status->r = 255;                                                                    // uint8
        pen_status->g = 255;                                                                    // uint8
        pen_status->b = 255;                                                                    // uint8

        // pen width
        pen_status->width = 3;                                                                  // uint8

        // pen toggle: val ? true : false
        pen_status->off = status ? 0 : 1;                                                       // use ternary operator to avoid if/else condition. If 1, off. if 0, on (draw).

        //  send a request to the client and wait for the service response
        auto pen_response = pen_client_->async_send_request(pen_status);                        // hold the acknowledgement status from the service response
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), pen_response);      // use the acknowledgement status to wait

        // note to self: get_node_base_interface() is a method inside the class responsible for spinning. that is what is being monitored
    }

    
    // draw using teleport_absolute based on the status of the pen
    void teleport_pen(float x, float y, float theta)
    {
        // create a shared pointer the instane of the message template for defining the coordinates when drawing
        auto teleport_params = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        teleport_params->x = x;                                                                  // float32
        teleport_params->y = y;                                                                  // float32
        teleport_params->theta = theta;                                                          // float32

        // execute drawing and spin the node until response is recieved
        auto teleport_response = teleport_client_->async_send_request(teleport_params);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), teleport_response);
    }


    // set the position of turtle1 to Q2
    void set_position(float x, float y, float theta)
    {
        toggle_pen(false);
        teleport_pen(x, y, theta);
    }


    // get the bounds based on turtle2's position
    void get_turtle2_bounds(turtlesim::msg::Pose::SharedPtr msg)
    {
        if (msg->x >= 5.5 && msg->y < 5.5)
        {
            quadrant = 4;
        }

        else if (msg->x >= 5.5 && msg->y >= 5.5)
        {
            quadrant = 1;
        }

        else if (msg->x < 5.5 && msg->y >= 5.5)
        {
            quadrant = 2;
        }

        else if (msg->x < 5.5 && msg->y < 5.5)
        {
            quadrant = 3;
        }
    }

    
    // rotate turtle2 based on which quadrant it is located w.r.t turtle1's position
    void rotate_turtle2_callback(turtlesim::msg::Pose::SharedPtr msg)
    {
        // quadrant 4
        if (quadrant == 4 && (msg->x > 5.5 && msg->x <= 11) && (msg->y < 5.5 && msg->y >= 0.0))
        {
            rotate_turt.angular.z = 1.0;
            rotate_turt_pub_->publish(rotate_turt);
            RCLCPP_INFO(this->get_logger(), "rotating turtle2");
        }

        // quadrant 1
        if (quadrant == 1 && (msg->x > 5.5 && msg->x <= 11) && (msg->y > 5.5 && msg->y <= 11))
        {
            rotate_turt.angular.z = 1.0;
            rotate_turt_pub_->publish(rotate_turt);
            RCLCPP_INFO(this->get_logger(), "rotating turtle2");
        }

        // quadrant 2
        if (quadrant == 2 && (msg->x <= 5.5 && msg->x >= 0.0) && (msg->y >= 5.5 && msg->y <= 11))
        {
            rotate_turt.angular.z = 1.0;
            rotate_turt_pub_->publish(rotate_turt);
            RCLCPP_INFO(this->get_logger(), "rotating turtle2");
        }

        // quadrant 3
        if (quadrant == 3 && (msg->x <= 5.5 && msg->x >= 0.0) && (msg->y < 5.5 && msg->y >= 0.0))
        {
            rotate_turt.angular.z = 1.0;
            rotate_turt_pub_->publish(rotate_turt);
            RCLCPP_INFO(this->get_logger(), "rotating turtle2");
        }
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtlesim_ex1_node>(); // MODIFY NAME
    node->draw_quadrants();
    std::this_thread::sleep_for(1s);
    node->position_turtle1();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}