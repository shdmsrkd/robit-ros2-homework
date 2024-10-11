#include <rclcpp/rclcpp.hpp>
#include <QApplication>
#include <QTimer>
#include "turtle_controller_gui.hpp"
#include "turtle_control/msg/turtle_command.hpp"
#include "geometry_msgs/msg/twist.hpp"

class TurtleControlNode : public rclcpp::Node
{
public:
    TurtleControlNode() : Node("turtle_control_node")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel", 10);
        subscription_ = this->create_subscription<turtle_control::msg::TurtleCommand>
        (
            "turtle_command", 10,
            std::bind(&TurtleControlNode::commandCallback, this, std::placeholders::_1)
        );
    }

private:
    void commandCallback(const turtle_control::msg::TurtleCommand::SharedPtr msg)
    {
        auto twist = geometry_msgs::msg::Twist();
        twist.linear.x = msg->linear;
        twist.angular.z = msg->angular;
        publisher_->publish(twist);
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::Subscription<turtle_control::msg::TurtleCommand>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    QApplication app(argc, argv);

    auto node = std::make_shared<TurtleControlNode>();
    TurtleControllerGUI gui(node);
    gui.show();

    QTimer timer;
    QObject::connect(&timer, &QTimer::timeout, [&]() 
    {
        rclcpp::spin_some(node);
    });
    timer.start(10);

    int result = app.exec();
    rclcpp::shutdown();
    return result;
}
