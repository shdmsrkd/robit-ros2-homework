#ifndef TURTLE_CONTROLLER_GUI_HPP
#define TURTLE_CONTROLLER_GUI_HPP

#include <QMainWindow>
#include <QPushButton>
#include <rclcpp/rclcpp.hpp>
#include "turtle_control/msg/turtle_command.hpp"

class TurtleControllerGUI : public QMainWindow
{
    Q_OBJECT

public:
    TurtleControllerGUI(rclcpp::Node::SharedPtr node);

private slots:
    void moveForward();
    void moveBackward();
    void turnLeft();
    void turnRight();

private:
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<turtle_control::msg::TurtleCommand>::SharedPtr publisher_;
    QPushButton *upButton_, *downButton_, *leftButton_, *rightButton_;

    void setupUI();
    void publishCommand(double linear, double angular);
};

#endif // TURTLE_CONTROLLER_GUI_HPP
