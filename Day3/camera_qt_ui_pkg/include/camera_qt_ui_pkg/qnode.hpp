#ifndef QNODE_HPP
#define QNODE_HPP

// Qt 헤더
#include <QThread>
#include <QImage>
#include <QPixmap>
#include <QString>

// ROS2 헤더
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

Q_DECLARE_METATYPE(QImage)  // QImage 타입을 Qt의 메타 시스템에 등록

class QNode : public QThread {
    Q_OBJECT
public:
    QNode();
    virtual ~QNode();

protected:
    void run() override;

Q_SIGNALS:
    void rosShutDown();
    void imageReceived(const QImage &image);

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    
    rclcpp::Node::SharedPtr node;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
};

#endif // QNODE_HPP
