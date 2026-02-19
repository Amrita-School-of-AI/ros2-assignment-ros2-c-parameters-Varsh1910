#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

/*
 * TODO: Create a Class named 'ParamNode' that inherits from rclcpp::Node.
 * Requirements:
 * 1. The constructor should name the node "param_node".
 * 2. Declare these parameters with default values:
 *    - "robot_name" (string): default "ROS2Bot"
 *    - "max_speed" (double): default 1.5
 *    - "enabled" (bool): default true
 * 3. Create a timer that triggers every 2000ms.
 * 4. In timer callback, read parameters and log:
 *    "Robot: <name>, Max Speed: <speed>, Enabled: <enabled>"
 */

class ParamNode : public rclcpp::Node
{
public:
    ParamNode()
        : Node("param_node")
    {
        // TODO: Declare parameters here

        // TODO: Create timer here
    }

private:
    // TODO: Define timer_callback function here

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ParamNode>());
    rclcpp::shutdown();
    return 0;
}
