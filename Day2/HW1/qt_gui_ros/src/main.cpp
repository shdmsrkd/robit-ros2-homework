#include <QApplication>
#include <iostream>

#include "../include/qt_gui_ros/main_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
