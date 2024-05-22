#include "ros2_roboreg_rviz/display.hpp"

namespace ros2_roboreg_rviz {
Display::Display() {
  description_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Description Topic", "robot_description",
      rosidl_generator_traits::name<std_msgs::msg::String>(),
      "Topic under which the robot description is published.", this,
      SLOT(updateRobotDescriptionTopic()), this);
}

void Display::onInitialize() {
  rviz_common::Display::onInitialize();
  auto node_abstraction = this->context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    throw std::runtime_error("Failed to lock node abstraction.");
  }
  node_ptr_ = node_abstraction->get_raw_node();

  description_topic_property_->initialize(node_abstraction);

  // add a collect data and save synced data widgets
  auto widget = new QWidget();
  auto collect_data_widget = new CollectDataWidget(node_ptr_, widget);
  auto register_widget = new RegisterWidget(node_ptr_, widget);
  auto save_data_widget = new SaveDataWidget(node_ptr_, widget);
  setAssociatedWidget(widget);

  // set layout
  auto layout = new QVBoxLayout(widget);
  layout->addWidget(collect_data_widget);
  layout->addWidget(register_widget);
  layout->addWidget(save_data_widget);
}

void Display::updateRobotDescriptionTopic() {
  RCLCPP_INFO(node_ptr_->get_logger(), "Robot description topic changed.");

  // parameter cb
}
} // end of namespace ros2_roboreg_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_roboreg_rviz::Display, rviz_common::Display)
