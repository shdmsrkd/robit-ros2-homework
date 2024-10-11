#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/set_pen.hpp"
#include "turtlesim/srv/kill.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <termios.h>
#include <stdio.h>
#include <memory.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <vector>

#define KEYCODE_W 0x77
#define KEYCODE_A 0x61
#define KEYCODE_S 0x73
#define KEYCODE_D 0x64

class TeleopTurtle
{
public:
   TeleopTurtle(std::shared_ptr<rclcpp::Node> node);
   void keyLoop();
   void setPen(int r, int g, int b, int width);
   void setBackground(int r, int g, int b);
   void killTurtle(std::string name);
   void initializeTurtlesim();
   void spawnAllTurtles();
   void selectTurtle(int choice);
private:
   std::shared_ptr<rclcpp::Node> node_;
   double linear_, angular_, l_scale_, a_scale_;
   rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
   rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr spawn_client_;
   rclcpp::Client<turtlesim::srv::SetPen>::SharedPtr pen_client_;
   rclcpp::Client<turtlesim::srv::Kill>::SharedPtr kill_client_;
   std::string current_turtle_name_;
   std::vector<std::string> turtle_names;
   
   int kfd;
   struct termios cooked;
   struct termios raw;
};

TeleopTurtle::TeleopTurtle(std::shared_ptr<rclcpp::Node> node):
   node_(node),
   linear_(0),
   angular_(0),
   l_scale_(2.0),
   a_scale_(2.0),
   kfd(0)
{
   spawn_client_ = node_->create_client<turtlesim::srv::Spawn>("/spawn");
}

void TeleopTurtle::initializeTurtlesim()
{
   killTurtle("turtle1");
   std::this_thread::sleep_for(std::chrono::seconds(1));
}

void TeleopTurtle::spawnAllTurtles()
{
   turtle_names.clear();
   
   double positions[9][2] = {
       {3.0, 3.0},   
       {5.5, 3.0},   
       {8.0, 3.0},   
       {3.0, 5.5},  
       {5.5, 5.5},   
       {8.0, 5.5}, 
       {3.0, 8.0},  
       {5.5, 8.0}, 
       {8.0, 8.0}   
   };

   for(int i = 1; i <= 9; i++) {
       std::string name = "turtle" + std::to_string(i);
       turtle_names.push_back(name);

       while (!spawn_client_->wait_for_service(std::chrono::seconds(1))) {
           if (!rclcpp::ok()) {
               return;
           }
       }

       auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
       request->x = positions[i-1][0];
       request->y = positions[i-1][1];
       request->theta = 0.0;
       request->name = name;
       
       auto result_future = spawn_client_->async_send_request(request);
       std::this_thread::sleep_for(std::chrono::milliseconds(100));
   }
}

void TeleopTurtle::selectTurtle(int choice)
{
   std::string selected_name = "turtle" + std::to_string(choice);
   current_turtle_name_ = selected_name;

   for(const auto& name : turtle_names) {
       if(name != selected_name) {
           killTurtle(name);
           std::this_thread::sleep_for(std::chrono::milliseconds(100));
       }
   }

   twist_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>(selected_name + "/cmd_vel", 1);
   pen_client_ = node_->create_client<turtlesim::srv::SetPen>(selected_name + "/set_pen");
}

void TeleopTurtle::killTurtle(std::string name)
{
   kill_client_ = node_->create_client<turtlesim::srv::Kill>("/kill");
   
   while (!kill_client_->wait_for_service(std::chrono::seconds(1))) {
       if (!rclcpp::ok()) {
           return;
       }
   }
   
   auto request = std::make_shared<turtlesim::srv::Kill::Request>();
   request->name = name;
   
   auto result_future = kill_client_->async_send_request(request);
   std::this_thread::sleep_for(std::chrono::milliseconds(500));
}

void TeleopTurtle::setBackground(int r, int g, int b)
{
   auto parameters_client = std::make_shared<rclcpp::SyncParametersClient>(
       node_, "/turtlesim");
   
   parameters_client->set_parameters({
       rclcpp::Parameter("background_r", r),
       rclcpp::Parameter("background_g", g),
       rclcpp::Parameter("background_b", b)
   });
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

void TeleopTurtle::keyLoop()
{
   char c;
   bool dirty = false;

   tcgetattr(kfd, &cooked);
   memcpy(&raw, &cooked, sizeof(struct termios));
   raw.c_lflag &=~ (ICANON | ECHO);
   raw.c_cc[VEOL] = 1;
   raw.c_cc[VEOF] = 2;
   tcsetattr(kfd, TCSANOW, &raw);

   while(1)
   {
       linear_ = 0;
       angular_ = 0;
       if(::read(kfd, &c, 1) < 0)
       {
           perror("read():");
           exit(-1);
       }

       switch(c)
       {
           case KEYCODE_W:
               linear_ = 1.0;
               dirty = true;
               break;
           case KEYCODE_S:
               linear_ = -1.0;
               dirty = true;
               break;
           case KEYCODE_A:
               angular_ = 1.0;
               dirty = true;
               break;
           case KEYCODE_D:
               angular_ = -1.0;
               dirty = true;
               break;
           case 'q':
               tcsetattr(kfd, TCSANOW, &cooked);
               return;
       }

       geometry_msgs::msg::Twist twist;
       twist.angular.z = a_scale_ * angular_;
       twist.linear.x = l_scale_ * linear_;

       if(dirty == true)
       {
           twist_pub_->publish(twist);    
           dirty = false;
       }
   }
}

int main(int argc, char** argv)
{
   rclcpp::init(argc, argv);
   auto node = rclcpp::Node::make_shared("teleop_turtle");
   TeleopTurtle teleop_turtle(node);
   
   teleop_turtle.initializeTurtlesim();
   
   int mode_choice;
   bool running = true;
   int turtle_shape;
   int R, G, B;
   int pen_r, pen_g, pen_b;
   int pen_width;

   while(running)
   {
       rclcpp::spin_some(node);
       std::cout << "모드를 선택하세요(1. 시작, 2. 배경색 설정, 3. 거북이 모양 설정, 4. 펜 설정) : " << std::endl;
       std::cin >> mode_choice;

       switch(mode_choice)
       {
           case 1:
               teleop_turtle.keyLoop();
               break;

           case 2: 
               std::cin >> R >> G >> B;
               teleop_turtle.setBackground(R, G, B);
               break;

           case 3:
               teleop_turtle.spawnAllTurtles();
               std::this_thread::sleep_for(std::chrono::seconds(1));
               
               std::cout << "조종할 거북이를 선택하세요 (1~9): ";
               std::cin >> turtle_shape;
               if(turtle_shape >= 1 && turtle_shape <= 9) {
                   teleop_turtle.selectTurtle(turtle_shape);
               } else {
                   teleop_turtle.selectTurtle(1);
               }
               break;

           case 4:
               std::cin >> pen_r >> pen_g >> pen_b;
               std::cin >> pen_width;
               teleop_turtle.setPen(pen_r, pen_g, pen_b, pen_width);
               break;

           default:
               break;
       }
   }
   
   rclcpp::shutdown();
   return 0;
}

