#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"                  // for creating turtle2

using namespace std::chrono_literals;       // para rekta sulat na lang kung ilang seconds

class turtlesim_ex1_node : public rclcpp::Node
{
public:
    turtlesim_ex1_node() : Node("turtlesim_ex1_spawn")
    {
        // declare parameters as default values
        this->declare_parameter("x", 8.5);                                                     // turtle2 x coordinate
        this->declare_parameter("y", 2.75);                                                    // turtle2 y coordinate
        this->declare_parameter("theta", 0.0);                                                 // turtle2 z rotation

        this->declare_parameter("v_linear_x", 1.0);                                              // linear velocity
        this->declare_parameter("v_angular_z", 0.0);                                             // angular velocity         

        // use declared parameters within the code
        x = this->get_parameter("x").as_double();
        y = this->get_parameter("y").as_double();
        theta = this->get_parameter("theta").as_double();
        
        v_linear = this->get_parameter("v_linear_x").as_double();
        v_angular = this->get_parameter("v_angular_z").as_double();

        // create turtle2
        spawn_client_= this->create_client<turtlesim::srv::Spawn>("/spawn");

        // return a message when v_linear and v_angular is out of bounds
        param_callback_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &params)->rcl_interfaces::msg::SetParametersResult
            {
                auto result = check_param_values(params);
                return result;
            }
        );

        // Wait for following services to be available, which comes from the turtle1
        while (!spawn_client_->wait_for_service(1s))
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for turtlesim services...");
        }
    }

    
    // spawn turtle 2
    void spawn_turtle2()
    {
        spawn_and_set_position(x, y, theta, "turtle2");
    }


private:
    // initialize private variables
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_;
    rcl_interfaces::msg::SetParametersResult param_result;

    float x;
    float y;
    float theta;
    float v_linear;
    float v_angular;


    // spawn the 2nd turtle and position in correct quadrant
    void spawn_and_set_position(float x, float y, float theta, const std::string &name)
    {
        auto turtle_params = std::make_shared<turtlesim::srv::Spawn::Request>();
        turtle_params->x = x;
        turtle_params->y = y;
        turtle_params->theta = theta;
        turtle_params->name = name;

        auto turtle_response = spawn_client_->async_send_request(turtle_params);
        rclcpp::spin_until_future_complete(this->get_node_base_interface(), turtle_response);
    }


    // create a callback for the linear and angular velocity
    rcl_interfaces::msg::SetParametersResult check_param_values(const std::vector<rclcpp::Parameter> params)
    {
        param_result.successful = true;
        param_result.reason = "All parameters valid";
        for (const auto &param : params) 
        {
            if (param.get_name() == "v_linear_x") 
            {
                double param_value = param.as_double();
                if (param_value <= 0.0 || param_value >= 3.0)
                {
                    param_result.successful = false;
                    param_result.reason = "linear velocity must be between 0.0 - 3.0";
                    RCLCPP_ERROR(this->get_logger(), "%s", param_result.reason.c_str());
                }
            }

            else if (param.get_name() == "v_angular_z") {
                double param_value = param.as_double();
                if (param_value <= 0.0 || param_value >= 1.0)
                {
                    param_result.successful = false;
                    param_result.reason = "angular velocity must be between 0.0 - 1.0";
                    RCLCPP_ERROR(this->get_logger(), "%s", param_result.reason.c_str());
                }
            }
        }

        return param_result;
    } 
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<turtlesim_ex1_node>(); // MODIFY NAME;
    node->spawn_turtle2();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}