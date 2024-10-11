#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "custom_interfaces/msg/input_type.hpp"

using namespace std::chrono_literals;

class InputTypePublisher : public rclcpp::Node
{
public:
  InputTypePublisher()
  : Node("input_type_publisher")
  {
    publisher_ =
      this->create_publisher<custom_interfaces::msg::InputType>("input_type", 10);
    
    // Get user input
    std::cout << "Enter your name: ";
    std::getline(std::cin, name_);
    
    std::cout << "Enter your grade: ";
    std::getline(std::cin, grade_);
    
    std::cout << "Enter your student number: ";
    std::getline(std::cin, student_number_);

    auto publish_msg = [this]() -> void 
    {
      auto message = custom_interfaces::msg::InputType();
      message.name = name_;
      message.grade = grade_;
      message.student_number = student_number_;
      
      RCLCPP_INFO(this->get_logger(), "Publishing Input\nName: %s  Grade: %s  Student Number: %s", 
                  message.name.c_str(), message.grade.c_str(),
                  message.student_number.c_str());
      
      this->publisher_->publish(message);
    };
    
    timer_ = this->create_wall_timer(1s, publish_msg);
  }

private:
  rclcpp::Publisher<custom_interfaces::msg::InputType>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::string name_;
  std::string grade_;
  std::string student_number_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputTypePublisher>());
  rclcpp::shutdown();
  return 0;
}
