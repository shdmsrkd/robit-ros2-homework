#include "../include/ros2_topic_qt_ui/main_window.hpp"
#include "ui_mainwindowdesign.h" 

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindowDesign)
{
    ui->setupUi(this);
    
    qnode = new QNode(argc, argv);
    qnode->init();

    ui->label->setText("Publisher");
    ui->label_2->setText("Subscriber");
    ui->label_3->setText("Count: 0");

    connect(qnode, &QNode::messageReceived, this, &MainWindow::updateReceivedMessage);
    connect(qnode, &QNode::rosShutdown, this, &MainWindow::close);
    connect(ui->pushButton, &QPushButton::clicked, this, &MainWindow::on_pushButton_clicked);
}

MainWindow::~MainWindow()
{
    delete qnode;
    delete ui;
}

void MainWindow::on_pushButton_clicked()
{
    qnode->publishCounter();
}

void MainWindow::updateReceivedMessage(const QString &message)
{
    ui->label_3->setText("Count: " + message);
}
