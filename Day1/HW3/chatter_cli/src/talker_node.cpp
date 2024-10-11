#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("publisher"), count_(1) 
  {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);  

    auto publish_message =
      [this]() -> void
      {
        string_msg_ = std::make_unique<std_msgs::msg::String>();
        count_msg_ = std::make_unique<std_msgs::msg::Int64>();

        std::cout << "Enter message to publish: ";
        std::string input;
        std::getline(std::cin, input);

        string_msg_->data = input;
        count_msg_->data = count_++;

        RCLCPP_INFO(this->get_logger(), "Publishing: '%s', Count: %ld", 
                    input.c_str(), count_ - 1);

        string_publisher_->publish(std::move(string_msg_));
        count_publisher_->publish(std::move(count_msg_));
      };

    rclcpp::QoS qos(rclcpp::KeepLast{7});
    
    string_publisher_ = this->create_publisher<std_msgs::msg::String>("topic", qos);
    count_publisher_ = this->create_publisher<std_msgs::msg::Int64>("count", qos);
    
    timer_ = this->create_wall_timer(1s, publish_message);
  }

private:
  size_t count_;
  std::unique_ptr<std_msgs::msg::String> string_msg_;
  std::unique_ptr<std_msgs::msg::Int64> count_msg_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr string_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr count_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}
