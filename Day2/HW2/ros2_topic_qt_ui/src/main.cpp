#include <QApplication>
#include "../include/ros2_topic_qt_ui/main_window.hpp"

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);
    MainWindow w(argc, argv);
    w.show();
    return app.exec();
}
