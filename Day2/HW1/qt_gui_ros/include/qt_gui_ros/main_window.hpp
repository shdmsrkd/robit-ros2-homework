#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QString>
#include <QSpinBox>
#include "ui_mainwindow.h"
#include "qnode.hpp"
#include <chrono>
#include <thread>
#include <QTimer>

namespace Ui {
class MainWindowDesign;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = nullptr);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event);

private Q_SLOTS:
    void on_pushButton_clicked();
    void on_pushButton_2_clicked();
    void on_pushButton_3_clicked();
    void on_pushButton_4_clicked();
    
    void on_pushButton_9_clicked();
    void on_pushButton_10_clicked();
    void on_pushButton_11_clicked();
    
    void on_spinBox_8_valueChanged(int value);
    void on_spinBox_9_valueChanged(int value);
    void on_spinBox_10_valueChanged(int value);
    
    void on_spinBox_valueChanged(int value);
    void on_spinBox_2_valueChanged(int value);
    void on_spinBox_3_valueChanged(int value);
    void on_spinBox_11_valueChanged(int value);
    
    void on_spinBox_12_valueChanged(int value);
    
    void updatePenProperties();
    void updateTurtleSimBackground();
    
    void PrintCmd_vel();
private:
    Ui::MainWindowDesign* ui;
    QNode* qnode;
    QTimer *timer;
    double linear;
    double angular; 
};

#endif // MAIN_WINDOW_HPP
