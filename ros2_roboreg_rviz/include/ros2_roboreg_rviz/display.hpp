#ifndef ROS2_ROBOREG_RVIZ__DISPLAY_HPP_
#define ROS2_ROBOREG_RVIZ__DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <chrono>
#include <memory>

#include <QVBoxLayout>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "std_msgs/msg/string.hpp"

#include "ros2_roboreg_rviz/collect_data_widget.hpp"
#include "ros2_roboreg_rviz/register_widget.hpp"
#include "ros2_roboreg_rviz/save_data_widget.hpp"
#endif

namespace ros2_roboreg_rviz {
class Display : public rviz_common::Display {
  Q_OBJECT

public:
  Display();

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateRobotDescriptionTopic();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rviz_common::properties::RosTopicProperty *description_topic_property_;
};
} // end of namespace ros2_roboreg_rviz
#endif // ROS2_ROBOREG_RVIZ__DISPLAY_HPP_
