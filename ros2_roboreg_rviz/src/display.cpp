#include "ros2_roboreg_rviz/display.hpp"

namespace ros2_roboreg_rviz {
Display::Display() {
  robot_description_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Description Topic", "robot_description",
      rosidl_generator_traits::name<std_msgs::msg::String>(),
      "Topic under which the robot description is published.", this,
      SLOT(updateRobotDescriptionTopic()), this);
  roboreg_node_name_property_ = new rviz_common::properties::StringProperty(
      "Roboreg Node Name", "roboreg", "The node name under which the roboreg server lives.", this,
      SLOT(updateRoboregNodeNode()), this);
}

void Display::onInitialize() {
  rviz_common::Display::onInitialize();
  auto node_abstraction = this->context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    throw std::runtime_error("Failed to lock node abstraction.");
  }
  node_ptr_ = node_abstraction->get_raw_node();

  robot_description_topic_property_->initialize(node_abstraction);

  // add a collect data and save synced data widgets
  auto widget = new QWidget();
  collect_data_widget_ = new CollectDataWidget(node_ptr_, widget);
  register_widget_ = new RegisterWidget(node_ptr_, widget);
  save_data_widget_ = new SaveDataWidget(node_ptr_, widget);
  setAssociatedWidget(widget);

  // set layout
  auto layout = new QVBoxLayout(widget);
  layout->addWidget(collect_data_widget_);
  layout->addWidget(register_widget_);
  layout->addWidget(save_data_widget_);
}

void Display::updateRobotDescriptionTopic() {
  RCLCPP_INFO(node_ptr_->get_logger(), "Robot description topic changed.");
}

void Display::updateRoboregNodeNode() {
  collect_data_widget_->setupClient(roboreg_node_name_property_->getStdString());
  register_widget_->setupClient(roboreg_node_name_property_->getStdString());
  save_data_widget_->setupClient(roboreg_node_name_property_->getStdString());
}
} // end of namespace ros2_roboreg_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_roboreg_rviz::Display, rviz_common::Display)
