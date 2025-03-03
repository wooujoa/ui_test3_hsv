/**
 * @file /include/ui_test3_hsv/main_window.hpp
 *
 * @brief Qt based gui for %(package)s.
 *
 * @date January 2025
 **/

#ifndef ui_test3_hsv_MAIN_WINDOW_H
#define ui_test3_hsv_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QMainWindow>
#include "QIcon"
#include "qnode.hpp"
#include "ui_mainwindow.h"
#include <cv_bridge/cv_bridge.h>

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow
{
  Q_OBJECT

public:
  MainWindow(QWidget *parent = nullptr);
  ~MainWindow();
  QNode *qnode;

  cv::Mat clone_mat;\
  int hsv_value[6];

public Q_SLOTS:
  void slotUpdateImg();
  void Find_binary_img(cv::Mat& img);
  void Find_hsv_img(cv::Mat& img);
  cv::Mat get_binary(cv::Mat& img, int val[]);

private:
  Ui::MainWindowDesign *ui;
  void closeEvent(QCloseEvent *event);
};

#endif // ui_test3_hsv_MAIN_WINDOW_H
