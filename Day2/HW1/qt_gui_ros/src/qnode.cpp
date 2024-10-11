#include "../include/qt_gui_ros/qnode.hpp"
#include <rcl_interfaces/msg/parameter.hpp>
#include <rcl_interfaces/srv/set_parameters.hpp>

QNode::QNode()
{
  int argc = 0;
  char** argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("qt_gui_ros");
  
  cmd_vel_pub = node->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
  param_client = node->create_client<rcl_interfaces::srv::SetParameters>("/turtlesim/set_parameters");
  pen_client = node->create_client<turtlesim::srv::SetPen>("/turtle1/set_pen");
  
  this->start();
}

QNode::~QNode()
{
  rclcpp::shutdown();
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::publishVelocity(double linear_x, double angular_z)
{
  auto twist_msg = geometry_msgs::msg::Twist();
  twist_msg.linear.x = linear_x;
  twist_msg.angular.z = angular_z;
  cmd_vel_pub->publish(twist_msg);
}

void QNode::setBackgroundColor(int r, int g, int b)
{
    auto request = std::make_shared<rcl_interfaces::srv::SetParameters::Request>();
    
    rcl_interfaces::msg::Parameter b_r_param;
    b_r_param.name = "background_r";
    b_r_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    b_r_param.value.integer_value = r;
    
    rcl_interfaces::msg::Parameter b_g_param;
    b_g_param.name = "background_g";
    b_g_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    b_g_param.value.integer_value = g;
    
    rcl_interfaces::msg::Parameter b_b_param;
    b_b_param.name = "background_b";		
    b_b_param.value.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    b_b_param.value.integer_value = b;
    
    request->parameters = {b_r_param, b_g_param, b_b_param};
    auto result_future = param_client->async_send_request(request);
}

void QNode::setPenColorLength(int r, int g, int b, int width)
{
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = 0;
    
    auto result_future = pen_client->async_send_request(request);
}



