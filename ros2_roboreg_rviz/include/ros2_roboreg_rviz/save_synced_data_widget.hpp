#ifndef ROS2_ROBOREG_RVIZ__SAVE_SYNCED_DATA_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__SAVE_SYNCED_DATA_WIDGET_HPP_

#include <QBoxLayout>
#include <QDir>
#include <QFileDialog>
#include <QPushButton>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"

#include "ros2_roboreg_idl/srv/save_synced_data.hpp"

namespace ros2_roboreg_rviz {
class SaveSyncedDataWidget : public QWidget {
public:
  SaveSyncedDataWidget(const rclcpp::Node::SharedPtr node_ptr, QWidget *parent = nullptr);

protected:
  void onSaveSyncedData_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::SaveSyncedData>::SharedPtr save_synced_data_client_ptr_;

  QPushButton *save_synced_data_button_;
};
} // end of namespace ros2_roboreg_rviz

#endif // ROS2_ROBOREG_RVIZ__SAVE_SYNCED_DATA_WIDGET_HPP_