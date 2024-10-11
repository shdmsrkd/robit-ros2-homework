#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QImage>
#include <QPixmap>
#include <QCloseEvent>
#include <QDebug>  // QDebug 헤더 추가
#include "ui_mainwindow.h"
#include "qnode.hpp"

namespace Ui {
class MainWindowDesign;
}

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget* parent = 0);
    ~MainWindow();

protected:
    void closeEvent(QCloseEvent* event);

private Q_SLOTS:
    void updateCameraImage(const QImage &image);

private:
    Ui::MainWindowDesign* ui;
    QNode* qnode;
    void setupLabel() 
    {
        ui->label->setFixedSize(640, 480);  // QLabel 크기 고정
    }
};

#endif // MAIN_WINDOW_HPP
