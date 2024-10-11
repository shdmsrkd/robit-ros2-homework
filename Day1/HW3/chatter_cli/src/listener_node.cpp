#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int64.hpp"

class Subscriber : public rclcpp::Node
{
public:
  Subscriber()
  : Node("subscriber")
  {
    string_subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) 
      {
        RCLCPP_INFO(this->get_logger(), "Subscribed : %s, Count: %ld",
                    msg->data.c_str(), received_count);
      });

    countsubscription_ = this->create_subscription<std_msgs::msg::Int64>(
      "count", 10,
      [this](const std_msgs::msg::Int64::SharedPtr msg) 
      {
        received_count = msg->data;
      });
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr string_subscription_;
  rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr countsubscription_;
  int64_t received_count = 1;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Subscriber>());
  rclcpp::shutdown();
  return 0;
}
