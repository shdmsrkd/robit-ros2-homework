#include "../include/camera_qt_ui_pkg/qnode.hpp"

QNode::QNode()
{
    int argc = 0;
    char** argv = NULL;
    rclcpp::init(argc, argv);
    
    node = rclcpp::Node::make_shared("camera_qt_ui_pkg");
    RCLCPP_INFO(node->get_logger(), "Node created");
    
    image_sub_ = node->create_subscription<sensor_msgs::msg::Image>(
        "/camera1/camera/image_raw", 10, 
        std::bind(&QNode::imageCallback, this, std::placeholders::_1));
    
    RCLCPP_INFO(node->get_logger(), "Image subscription created for topic: /image_raw");
    
    this->start();
    RCLCPP_INFO(node->get_logger(), "QNode thread started");
}

QNode::~QNode()
{
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
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

void QNode::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try {

        RCLCPP_INFO(node->get_logger(), "Image received");
        
        cv::Mat cv_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
        
        RCLCPP_INFO(node->get_logger(), "Image size: %d x %d", cv_image.cols, cv_image.rows);
        
        QImage qimg(cv_image.data, cv_image.cols, cv_image.rows, 
                   cv_image.step, QImage::Format_BGR888);
        
        RCLCPP_INFO(node->get_logger(), "QImage created: %d x %d", qimg.width(), qimg.height());
        
        Q_EMIT imageReceived(qimg);
        RCLCPP_INFO(node->get_logger(), "Image signal emitted");
    }
    catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(node->get_logger(), "cv_bridge exception: %s", e.what());
    }
    catch (std::exception& e) {
        RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    }
}
