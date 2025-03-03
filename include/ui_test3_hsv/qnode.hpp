/**
 * @file /include/ui_test3_hsv/qnode.hpp
 *
 * @brief Communications central!
 *
 * @date January 2025
 **/
/*****************************************************************************
** Ifdefs
*****************************************************************************/

#ifndef ui_test3_hsv_QNODE_HPP_
#define ui_test3_hsv_QNODE_HPP_

/*****************************************************************************
** Includes
*****************************************************************************/
#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/camera_info.hpp>
#endif
#include <QThread>

/*****************************************************************************
** Class
*****************************************************************************/
class QNode : public QThread
{
  Q_OBJECT
public:
  QNode();
  ~QNode();

  cv::Mat* img_raw_ = NULL;
  bool isreceived = false;

protected:
  void run();

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_image_;
  void callbackImage(const sensor_msgs::msg::Image::ConstSharedPtr &msg_img);

Q_SIGNALS:
  void rosShutDown();
  void sigNewFrame();
};

#endif /* ui_test3_hsv_QNODE_HPP_ */
