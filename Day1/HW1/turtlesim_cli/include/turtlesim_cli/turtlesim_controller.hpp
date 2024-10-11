// include/turtlesim_cli/turtlesim_controller.hpp
#ifndef TURTLESIM_CONTROLLER_HPP_
#define TURTLESIM_CONTROLLER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_srvs/srv/empty.hpp>
#include <turtlesim/srv/set_pen.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <turtlesim/msg/color.hpp>
#include <termios.h>
#include <memory>

class TurtlesimController : public rclcpp::Node
{
public:
    TurtlesimController();
    void run();

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr set_pen_client_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_client_;
    
    void controlMode();
    void backgroundColorMode();
    void penSettingsMode();
    void showMenu();
    
    void enableRawMode();
    void disableRawMode();
    char readKey();
    struct termios orig_termios_;
};

#endif // TURTLESIM_CONTROLLER_HPP_
