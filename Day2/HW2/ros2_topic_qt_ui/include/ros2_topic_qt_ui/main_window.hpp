#ifndef MAIN_WINDOW_HPP
#define MAIN_WINDOW_HPP

#include <QMainWindow>
#include <QLabel>
#include <QPushButton>
#include "qnode.hpp"

// UI 클래스 전방 선언
namespace Ui {
    class MainWindowDesign;  // 클래스 이름 수정
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(int argc, char** argv, QWidget *parent = nullptr);
    virtual ~MainWindow();

private Q_SLOTS:
    void on_pushButton_clicked();
    void updateReceivedMessage(const QString &message);

private:
    Ui::MainWindowDesign *ui;  // 클래스 이름 수정
    QNode* qnode;
};

#endif // MAIN_WINDOW_HPP
