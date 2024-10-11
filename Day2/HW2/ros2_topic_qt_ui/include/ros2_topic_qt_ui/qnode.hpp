#ifndef Q_NODE_HPP
#define Q_NODE_HPP

#include <QThread>
#include <QStringListModel>
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

class QNode : public QThread {
    Q_OBJECT

public:
    QNode(int argc, char** argv);
    virtual ~QNode();
    bool init();
    void run() override;
    void publishMessage(const std::string& message);
    void publishCounter();

Q_SIGNALS:
    void rosShutdown();
    void messageReceived(const QString &message);

private:
    int init_argc;
    char** init_argv;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
    int count_;
    
    void subscriptionCallback(const std_msgs::msg::String::SharedPtr msg);

};

#endif // Q_NODE_HPP
