#ifndef ROS2_ROBOREG_RVIZ__SAVE_DATA_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__SAVE_DATA_WIDGET_HPP_

#include <QBoxLayout>
#include <QDir>
#include <QFileDialog>
#include <QPushButton>
#include <QWidget>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_roboreg_idl/srv/save_data.hpp"

namespace ros2_roboreg_rviz {
class SaveDataWidget : public QWidget {
public:
  SaveDataWidget(const rclcpp::Node::SharedPtr node_ptr, QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_nodename);

protected:
  void onSaveData_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::SaveData>::SharedPtr save_data_client_ptr_;

  QPushButton *save_data_button_;
};
} // end of namespace ros2_roboreg_rviz

#endif // ROS2_ROBOREG_RVIZ__SAVE_DATA_WIDGET_HPP_