#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>

class TeleopSubscriber : public rclcpp::Node
{
public:
    TeleopSubscriber() : Node("myteleop_subscriber")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/turtle1/cmd_vel", 10, std::bind(&TeleopSubscriber::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::Twist::SharedPtr msg) const
    {
        RCLCPP_INFO(this->get_logger(), "Received Twist command - linear.x: %f, angular.z: %f", msg->linear.x, msg->angular.z);
        // Here you can add additional logic to process or react to the received command
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopSubscriber>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
