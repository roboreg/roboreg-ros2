#ifndef ROS2_ROBOREG_RVIZ__IO_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__IO_WIDGET_HPP_

#include <QBoxLayout>
#include <QDir>
#include <QFileDialog>
#include <QPushButton>
#include <QWidget>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_roboreg_idl/srv/export.hpp"
#include "ros2_roboreg_idl/srv/import.hpp"

namespace ros2_roboreg_rviz {
class IOWidget : public QWidget {
public:
  IOWidget(const rclcpp::Node::SharedPtr node_ptr, const std::string &roboreg_node_name = "roboreg",
           QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_node_name);

protected:
  void onExportData_();
  void onExportTF_();
  void onImportTF_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::Export>::SharedPtr export_data_client_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::Export>::SharedPtr export_tf_client_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::Import>::SharedPtr import_tf_client_ptr_;

  QPushButton *export_data_button_;
  QPushButton *export_tf_button_;
  QPushButton *import_tf_button_;
};
} // end of namespace ros2_roboreg_rviz

#endif // ROS2_ROBOREG_RVIZ__IO_WIDGET_HPP_
