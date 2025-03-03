/**
 * @file /src/qnode.cpp
 *
 * @brief Ros communication central!
 *
 * @date January 2025
 **/

/*****************************************************************************
** Includes
*****************************************************************************/

#include "../include/ui_test3_hsv/qnode.hpp"

QNode::QNode()
{
  int argc = 0;
  char **argv = NULL;
  rclcpp::init(argc, argv);
  node = rclcpp::Node::make_shared("ui_test3_hsv");

  // this -> start(); 이전에 PUB, SUB하는 부분
  sub_image_ = node->create_subscription<sensor_msgs::msg::Image>(
      "/camera/camera/color/image_raw", 10,
      std::bind(&QNode::callbackImage, this, std::placeholders::_1));

  this->start();
}

QNode::~QNode()
{
  if (rclcpp::ok())
  {
    rclcpp::shutdown();
  }
}

void QNode::run()
{
  rclcpp::WallRate loop_rate(20);
  while (rclcpp::ok())
  {
    rclcpp::spin_some(node);
    loop_rate.sleep();
  }
  rclcpp::shutdown();
  Q_EMIT rosShutDown();
}

void QNode::callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img)
{
  RCLCPP_INFO(node->get_logger(), "✅ 이미지 데이터 수신 중...");
  if (img_raw_ == NULL && !isreceived)
  {
    try
    {
      img_raw_ = new cv::Mat(cv_bridge::toCvCopy(msg_img, sensor_msgs::image_encodings::RGB8)->image);

      if (img_raw_ != NULL)
      {
        Q_EMIT sigNewFrame();
        isreceived = true;
      }
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(node->get_logger(), "cv_bridge 변환 실패: %s", e.what());
    }
  }
}
