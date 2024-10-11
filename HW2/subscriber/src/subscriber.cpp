#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/input_type.hpp"

using std::placeholders::_1;

class InputTypeSubscriber : public rclcpp::Node
{
public:
  InputTypeSubscriber()
  : Node("InputTypesubscriber")
  {
    subscription_ = this->create_subscription<custom_interfaces::msg::InputType>(
      "input_type", 10, std::bind(&InputTypeSubscriber::topic_callback, this, _1));
  }

private:
  void topic_callback(const custom_interfaces::msg::InputType::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "Received Input\nName: %s  Grade: %s  Student Number: %s",
                msg->name.c_str(), msg->grade.c_str(), 
                msg->student_number.c_str());
  }
  rclcpp::Subscription<custom_interfaces::msg::InputType>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputTypeSubscriber>());
  rclcpp::shutdown();
  return 0;
}
