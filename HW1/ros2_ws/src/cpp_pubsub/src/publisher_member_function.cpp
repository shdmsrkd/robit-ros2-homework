#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/srv/teleport_absolute.hpp>
#include <ncurses.h>
#include <memory>
#include <functional>
#include <chrono>
#include <thread>

class TeleopPublisher : public rclcpp::Node
{
public:
    TeleopPublisher() : Node("myteleop_publisher")
    {
        publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&TeleopPublisher::timer_callback, this));
        reset_client_ = this->create_client<turtlesim::srv::TeleportAbsolute>("/turtle1/teleport_absolute");

        // 서비스가 준비될 때까지 기다립니다.
        while (!reset_client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }
    }

private:
    void timer_callback() // 타이머 돌때마다 이 함수 호출
    {
        int ch = getch(); // 입력받기 
        
        auto twist = geometry_msgs::msg::Twist();
        
        switch (ch) // 받은 입력값 판별해서 동작
        {
            case KEY_UP:
                twist.linear.x = 1.0;
                break;
            case KEY_DOWN:
                twist.linear.x = -1.0;
                break;
            case KEY_LEFT:
                twist.angular.z = 1.0;
                break;
            case KEY_RIGHT:
                twist.angular.z = -1.0;
                break;
            case 'q':
                endwin();
                rclcpp::shutdown();
                return;
            case 'r':
                reset_turtle_position();
                break;
            case 's':
            {
                // 직진 후 90도 회전 
                for (int i = 0; i < 4; ++i) 
                {
                    twist.linear.x = 1.0;   
                    twist.angular.z = 0.0;  
                    publisher_->publish(twist); 
                    std::this_thread::sleep_for(std::chrono::seconds(2));  
                    
                    twist.linear.x = 0.0;    
                    twist.angular.z = 1.57; // 1초에 90도 회전 가능 
                    publisher_->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }

                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                publisher_->publish(twist); 
                break;
            }
            case 'c':
            {
                for(int i = 0; i < 4; i++)
                {
                    twist.linear.x = 0.5;   
                    twist.angular.z = 1.57;  
                    publisher_->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(1)); 
                }
                
                twist.linear.x = 0.0;   
                twist.angular.z = 0.0;
                publisher_->publish(twist);
                
                break;
            }
            case 't':
            {
                for(int i = 0; i < 3; i++)
                {
                    twist.linear.x = 1.0;  
                    twist.angular.z = 0.0;
                    publisher_->publish(twist); 
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    
                    twist.linear.x = 0.0;    
                    twist.angular.z = 2.094; 
                    publisher_->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                publisher_->publish(twist); 
                break;
            }
        }
        
        publisher_->publish(twist);
    }

    void reset_turtle_position()
    {
        auto request = std::make_shared<turtlesim::srv::TeleportAbsolute::Request>();
        request->x = 5.544445;  
        request->y = 5.544445;  
        request->theta = 0.0;   

        auto result_future = reset_client_->async_send_request(request);
        

        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully reset turtle position");
        } else 
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to reset turtle position");
        }
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Client<turtlesim::srv::TeleportAbsolute>::SharedPtr reset_client_;
};

int main(int argc, char** argv)
{
    initscr();
    cbreak();
    noecho();
    keypad(stdscr, TRUE);
    nodelay(stdscr, TRUE);

    rclcpp::init(argc, argv);
    auto node = std::make_shared<TeleopPublisher>();
    rclcpp::spin(node);
    
    endwin();
    rclcpp::shutdown();
    return 0;
}

