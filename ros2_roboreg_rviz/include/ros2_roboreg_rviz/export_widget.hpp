#ifndef ROS2_ROBOREG_RVIZ__EXPORT_SAMPLES_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__EXPORT_SAMPLES_WIDGET_HPP_

#include <QBoxLayout>
#include <QDir>
#include <QFileDialog>
#include <QPushButton>
#include <QWidget>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_roboreg_idl/srv/export_samples.hpp"

namespace ros2_roboreg_rviz {
class ExportSamplesWidget : public QWidget {
public:
  ExportSamplesWidget(const rclcpp::Node::SharedPtr node_ptr, QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_nodename);

protected:
  void onExportSamples_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::ExportSamples>::SharedPtr export_samples_client_ptr_;

  QPushButton *export_samples_button_;
};
} // end of namespace ros2_roboreg_rviz

#endif // ROS2_ROBOREG_RVIZ__EXPORT_SAMPLES_WIDGET_HPP_