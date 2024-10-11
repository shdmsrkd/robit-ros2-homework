#ifndef DRAW_NODE_HPP
#define DRAW_NODE_HPP

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include <geometry_msgs/msg/twist.hpp>

class TeleopTurtle
{
public:
    TeleopTurtle(std::shared_ptr<rclcpp::Node> node_);
    void spawnTurtle(std::string name);
    void setPen(int r, int g, int b, int width);
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publication;

private:
    std::shared_ptr<rclcpp::Node> node_;
    std::string current_turtle_name_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
    rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
};

#endif // DRAW_NODE_HPP
