/**
 * @file /src/main_window.cpp
 *
 * @brief Implementation for the qt gui.
 *
 * @date January 2025
 **/
/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ui_test3_hsv/main_window.hpp"
#include <opencv2/opencv.hpp>

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindowDesign)
{
  ui->setupUi(this);

  QIcon icon("://ros-icon.png");
  this->setWindowIcon(icon);

  qnode = new QNode();

  QObject::connect(qnode, SIGNAL(rosShutDown()), this, SLOT(close()));
  QObject::connect(qnode, SIGNAL(sigNewFrame()), this, SLOT(slotUpdateImg()));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
  QMainWindow::closeEvent(event);
}

MainWindow::~MainWindow()
{
  delete ui;
}

void MainWindow::slotUpdateImg() // qnode로 부터 받은 이미지를 복제하여 UI(original_labe)에 표시
{
  RCLCPP_INFO(rclcpp::get_logger("ui_test2"), "label 수신 중...");

  clone_mat = qnode->img_raw_->clone(); // qnode에 있는 포인터 이미지를 main_window내에서 복제함(나눠서 해결하면 효율적이니까)
  cv::resize(clone_mat, clone_mat, cv::Size(640, 360), 0, 0, cv::INTER_CUBIC);

  //Find_binary_img(clone_mat);
  Find_hsv_img(clone_mat);

  QImage RGB_Img((const unsigned char *)(clone_mat.data), clone_mat.cols, clone_mat.rows, QImage::Format_RGB888);
  ui->original_img->setPixmap(QPixmap::fromImage(RGB_Img));

  delete qnode->img_raw_;
  qnode->img_raw_ = NULL;
  qnode->isreceived = false;
}
void MainWindow::Find_hsv_img(cv::Mat& img){
  int user_hsv_value[6] = {70, 98, 139, 255, 255, 255}; //지금 노란색
  cv::Mat user_hsv_img = get_binary(img, user_hsv_value);

  QImage binaryQImage(user_hsv_img.data, user_hsv_img.cols, user_hsv_img.rows, user_hsv_img.step, QImage::Format_Grayscale8);
  ui->hsv_img->setPixmap(QPixmap::fromImage(binaryQImage));
}
void MainWindow::Find_binary_img(cv::Mat &img)
{
  // hsv값에 따라 이진화 하는 과정 필요
  cv::Mat binary_img = get_binary(img, hsv_value);

  // hsv 찾을때 써야하는 구문
  QImage binaryQImage(binary_img.data, binary_img.cols, binary_img.rows, binary_img.step, QImage::Format_Grayscale8);
  ui->hsv_img->setPixmap(QPixmap::fromImage(binaryQImage));

  // hsv_value에 .ui의 슬라이더에 따른 값을 저장
  hsv_value[0] = ui->low_h_slider->value();
  hsv_value[1] = ui->low_s_slider->value();
  hsv_value[2] = ui->low_v_slider->value();
  hsv_value[3] = ui->high_h_slider->value();
  hsv_value[4] = ui->high_s_slider->value();
  hsv_value[5] = ui->high_v_slider->value();

  // hsv_value의 값을 LCD에 표시
  ui->low_h_LCD->display(hsv_value[0]);
  ui->low_s_LCD->display(hsv_value[1]);
  ui->low_v_LCD->display(hsv_value[2]);
  ui->high_h_LCD->display(hsv_value[3]);
  ui->high_s_LCD->display(hsv_value[4]);
  ui->high_v_LCD->display(hsv_value[5]);
}

// 출력값으로 이진화된 이미지 출력해야 하기 때문에 cv::Mat 반환값 설정
cv::Mat MainWindow::get_binary(cv::Mat &img, int val[])
{

  cv::Mat hsvimg;
  cv::cvtColor(img, hsvimg, cv::COLOR_BGR2HSV); // BGR->HSV로 아는거다

  // 디버깅용 hsv 이미지 보기
  //cv::imshow("HSV Image", hsvimg);

  // 가우시안 블러 써서 반사율 좀 줄이기
  cv::Mat gausianblurimg;
  cv::GaussianBlur(hsvimg, gausianblurimg, cv::Size(5, 5), 0);

  // 디버깅용 가우시안블러 이미지 보기
  //cv::imshow("GaussianBlur Image", gausianblurimg);

  // 3x3 kernel 생성
  cv::Mat mask = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(1, 1));
  // 생성된 kernel로 erode써서 노이즈 제거
  cv::erode(gausianblurimg, gausianblurimg, mask, cv::Point(-1, -1), 1);

  // hsv 범위내 색상을 threshold로 설정하여 마스킹
  cv::Scalar lower(val[0], val[1], val[2]);
  cv::Scalar upper(val[3], val[4], val[5]);
  cv::Mat res_bin_img;
  cv::inRange(gausianblurimg, lower, upper, res_bin_img);

  return res_bin_img;
}
