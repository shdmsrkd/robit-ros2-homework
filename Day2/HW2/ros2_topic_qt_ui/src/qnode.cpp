#include "../include/ros2_topic_qt_ui/qnode.hpp"

QNode::QNode(int argc, char** argv) :
    init_argc(argc),
    init_argv(argv),
    count_(0)
{
}

QNode::~QNode() {
    if(rclcpp::ok()) {
        rclcpp::shutdown();
    }
    wait();
}

bool QNode::init() {
    rclcpp::init(init_argc, init_argv);
    
    node = rclcpp::Node::make_shared("qt_pub_sub_node");
    
    publisher_ = node->create_publisher<std_msgs::msg::String>("counter_topic", 10);
    
    subscriber_ = node->create_subscription<std_msgs::msg::String>(
        "counter_topic", 10,
        std::bind(&QNode::subscriptionCallback, this, std::placeholders::_1));
    
    start();
    return true;
}

void QNode::run() {
    while(rclcpp::ok()) {
        rclcpp::spin_some(node);
        msleep(100);
    }
    Q_EMIT rosShutdown();
}

void QNode::publishCounter() {
    count_++;
    auto msg = std_msgs::msg::String();
    msg.data = std::to_string(count_);
    publisher_->publish(msg);
}

void QNode::subscriptionCallback(const std_msgs::msg::String::SharedPtr msg) {
    Q_EMIT messageReceived(QString::fromStdString(msg->data));
}
