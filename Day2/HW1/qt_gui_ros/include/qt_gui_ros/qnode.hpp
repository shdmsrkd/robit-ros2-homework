#ifndef QT_GUI_ROS_QNODE_HPP
#define QT_GUI_ROS_QNODE_HPP

#include <QThread>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>
#include <turtlesim/srv/set_pen.hpp>

class QNode : public QThread
{
  Q_OBJECT

public:
  QNode();
  virtual ~QNode();
  void run();
  void publishVelocity(double linear_x, double angular_z);
  void setBackgroundColor(int r, int g, int b);
  void setPenColorLength(int r, int g, int b, int width);
  void PrintCmd_vel(double linear, double angular);

Q_SIGNALS:
  void rosShutDown();

private:
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::Client<rcl_interfaces::srv::SetParameters>::SharedPtr param_client;
  rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client;
};

#endif // QT_GUI_ROS_QNODE_HPP
