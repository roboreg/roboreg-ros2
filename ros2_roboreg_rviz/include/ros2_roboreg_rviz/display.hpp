#ifndef ROS2_ROBOREG_RVIZ__DISPLAY_HPP_
#define ROS2_ROBOREG_RVIZ__DISPLAY_HPP_

#include <chrono>
#include <memory>

#include <QVBoxLayout>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"

#include "ros2_roboreg_rviz/collect_data_widget.hpp"
#include "ros2_roboreg_rviz/register_widget.hpp"
#include "ros2_roboreg_rviz/save_synced_data_widget.hpp"

namespace ros2_roboreg_rviz {
class Display : public rviz_common::Display {
public:
  Display() = default;

  void onInitialize() override;

protected:
  rclcpp::Node::SharedPtr node_ptr_;

  rclcpp::Client<ros2_roboreg_idl::srv::SaveSyncedData>::SharedPtr save_synced_data_client_ptr_;
};
} // end of namespace ros2_roboreg_rviz
#endif // ROS2_ROBOREG_RVIZ__DISPLAY_HPP_
