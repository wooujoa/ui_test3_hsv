#include <QApplication>
#include <iostream>

#include "../include/ui_test3_hsv/main_window.hpp"

int main(int argc, char* argv[])
{
  QApplication a(argc, argv);
  MainWindow w;
  w.show();
  return a.exec();
}
