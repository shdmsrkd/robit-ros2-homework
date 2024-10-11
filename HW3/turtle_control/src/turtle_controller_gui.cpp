#include "turtle_controller_gui.hpp"
#include <QGridLayout>

TurtleControllerGUI::TurtleControllerGUI(rclcpp::Node::SharedPtr node)
    : node_(node)
{
    publisher_ = node_->create_publisher<turtle_control::msg::TurtleCommand>("turtle_command", 10);
    setupUI();
}

void TurtleControllerGUI::setupUI()
{
    setWindowTitle("Turtle Controller");
    setFixedSize(240, 240);

    auto mainLayout = new QGridLayout;

    upButton_ = new QPushButton("↑");
    downButton_ = new QPushButton("↓");
    leftButton_ = new QPushButton("←");
    rightButton_ = new QPushButton("→");


    int buttonSize = 70;
    upButton_->setFixedSize(buttonSize, buttonSize);
    downButton_->setFixedSize(buttonSize, buttonSize);
    leftButton_->setFixedSize(buttonSize, buttonSize);
    rightButton_->setFixedSize(buttonSize, buttonSize);


    mainLayout->addWidget(upButton_, 0, 1);
    mainLayout->addWidget(leftButton_, 1, 0);
    mainLayout->addWidget(rightButton_, 1, 2);
    mainLayout->addWidget(downButton_, 1, 1);

    auto centralWidget = new QWidget(this);
    centralWidget->setLayout(mainLayout);
    setCentralWidget(centralWidget);

    connect(upButton_, &QPushButton::clicked, this, &TurtleControllerGUI::moveForward);
    connect(downButton_, &QPushButton::clicked, this, &TurtleControllerGUI::moveBackward);
    connect(leftButton_, &QPushButton::clicked, this, &TurtleControllerGUI::turnLeft);
    connect(rightButton_, &QPushButton::clicked, this, &TurtleControllerGUI::turnRight);
}

void TurtleControllerGUI::moveForward()
{
    publishCommand(1.0, 0.0);
}

void TurtleControllerGUI::moveBackward()
{
    publishCommand(-1.0, 0.0);
}

void TurtleControllerGUI::turnLeft()
{
    publishCommand(0.0, 1.0);
}

void TurtleControllerGUI::turnRight()
{
    publishCommand(0.0, -1.0);
}

void TurtleControllerGUI::publishCommand(double linear, double angular)
{
    auto msg = turtle_control::msg::TurtleCommand();
    msg.linear = linear;
    msg.angular = angular;
    publisher_->publish(msg);
}
