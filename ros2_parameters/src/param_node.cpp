#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class ParamNode : public rclcpp::Node
{
public:
    ParamNode() : Node("parameters_node")

    {
        // Declare parameters with default values
        this->declare_parameter<std::string>("robot_name", "DefaultBot");
        this->declare_parameter<double>("max_speed", 1.0);
        this->declare_parameter<bool>("enabled", true);

        // Create timer to print parameters every 2 seconds
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&ParamNode::print_parameters, this));
    }

private:
    void print_parameters()
    {
        std::string robot_name = this->get_parameter("robot_name").as_string();
        double max_speed = this->get_parameter("max_speed").as_double();
        bool enabled = this->get_parameter("enabled").as_bool();

        RCLCPP_INFO(this->get_logger(), "Robot Name: %s", robot_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Max Speed: %f", max_speed);
        RCLCPP_INFO(this->get_logger(), "Enabled: %s", enabled ? "true" : "false");
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamNode>());
    rclcpp::shutdown();
    return 0;
}

