#include "turtlesim_draw/draw_node.hpp"
#include <chrono>
#include <thread>

TeleopTurtle::TeleopTurtle(std::shared_ptr<rclcpp::Node> node): node_(node)
{
    spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("/spawn");
}

void TeleopTurtle::setPen(int r, int g, int b, int width)
{
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>(current_turtle_name_ + "/set_pen");
    
    auto request = std::make_shared<turtlesim::srv::SetPen::Request>();
    
    request->r = r;
    request->g = g;
    request->b = b;
    request->width = width;
    request->off = 0;
    
    pen_client_->async_send_request(request);
}

void TeleopTurtle::spawnTurtle(std::string name) 
{
    current_turtle_name_ = name;
    
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    
    request->x = 5.5;
    request->y = 5.5;
    request->theta = 0.0;
    request->name = name;
    
    twist_publication = node_->create_publisher<geometry_msgs::msg::Twist>(name + "/cmd_vel", 1);
    pen_client_ = node_->create_client<turtlesim::srv::SetPen>(name + "/set_pen");
    
    spawn_client_->async_send_request(request);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("teleop_turtle");
    TeleopTurtle teleop_turtle(node);
    
    int select_shape;
    bool running = true;
    int pen_r, pen_g, pen_b;
    int pen_width;
    int length;
    geometry_msgs::msg::Twist twist;

    teleop_turtle.spawnTurtle("turtle1");

    while(running)
    {
        std::cout << "크기 선택 (1~50) : " << std::endl;
        std::cin >> length;
        std::cout << "펜 색상 RGB 값을 차례로 입력하세요 (0-255): " << std::endl;
        std::cin >> pen_r >> pen_g >> pen_b;
        std::cout << "펜 굵기를 입력하세요 (1~10): ";
        std::cin >> pen_width;
        teleop_turtle.setPen(pen_r, pen_g, pen_b, pen_width);
        std::cout << "그릴 도형을 선택하세요(1. 삼각형, 2. 사각형, 3. 원) " << std::endl;
        std::cin >> select_shape;
	
        switch(select_shape)
        {
            case 1: // 삼각형
                for(int i = 0; i < 3; i++)
                {
                    twist.linear.x = (float)length/10;  
                    twist.angular.z = 0.0;
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    
                    twist.linear.x = 0.0;
                    twist.angular.z = 2.094;  
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                teleop_turtle.twist_publication->publish(twist);
                break;
                
            case 2: // 사각형
                for (int i = 0; i < 4; ++i) 
                {
                    twist.linear.x = (float)length/10; 
                    twist.angular.z = 0.0;
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    
                    twist.linear.x = 0.0;
                    twist.angular.z = 1.57; 
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                teleop_turtle.twist_publication->publish(twist);
                break;
                
            case 3: // 원
                for(int i = 0; i < 360; i++) 
                {
                    twist.linear.x = (float)length/10;
                    twist.angular.z = 1.0; 
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::milliseconds(50)); 
                }
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                teleop_turtle.twist_publication->publish(twist);
                break;
            
            default:
                std::cout << "오류입니다. 1, 2, 3 중 하나를 입력하세요." << std::endl;
                for(int i = 0; i < 3; i++)
                {
                    twist.linear.x = (float)length/10;
                    twist.angular.z = 0.0;
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(2));
                    
                    twist.linear.x = 0.0;
                    twist.angular.z = 2.094;
                    teleop_turtle.twist_publication->publish(twist);
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
                twist.linear.x = 0.0;
                twist.angular.z = 0.0;
                teleop_turtle.twist_publication->publish(twist);
                break;
        }

        char continue_answer;
        std::cout << "계속진행할거면 'y'를 그만할거면 'n'을 입력하세요 : ";
        std::cin >> continue_answer;
        if(continue_answer == 'n')
        {
            running = false;
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
