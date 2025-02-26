#ifndef ROBOREG_RVIZ__DISPLAY_HPP_
#define ROBOREG_RVIZ__DISPLAY_HPP_

#ifndef Q_MOC_RUN
#include <QVBoxLayout>
#include <QWidget>
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rviz_common/display.hpp"
#include "rviz_common/display_context.hpp"
#include "rviz_common/properties/ros_topic_property.hpp"
#include "rviz_common/properties/string_property.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/string.hpp"

#include "roboreg_rviz/collect_data_widget.hpp"
#include "roboreg_rviz/formatting.hpp"
#include "roboreg_rviz/io_widget.hpp"
#include "roboreg_rviz/register_widget.hpp"
#endif

namespace roboreg_rviz {
class Display : public rviz_common::Display {
  Q_OBJECT

public:
  Display();

protected:
  void onInitialize() override;

private Q_SLOTS:
  void updateRobotDescriptionTopic();
  void updateJointStateTopic();
  void updateDepthTopic();
  void updateRoboregNode();

private:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::AsyncParametersClient::UniquePtr parameters_client_;
  std::string roboreg_namespace_;
  std::string roboreg_node_name_;

  CollectDataWidget *collect_data_widget_;
  RegisterWidget *register_widget_;
  IOWidget *io_widget_;

  // properties
  rviz_common::properties::RosTopicProperty *robot_description_topic_property_;
  rviz_common::properties::RosTopicProperty *joint_state_topic_property_;
  rviz_common::properties::StringProperty *roboreg_namespace_property_;
};
} // end of namespace roboreg_rviz
#endif // ROBOREG_RVIZ__DISPLAY_HPP_
