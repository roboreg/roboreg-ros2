#include "ros2_roboreg_rviz/display.hpp"

namespace ros2_roboreg_rviz {
Display::Display() {
  robot_description_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Description Topic", "robot_description",
      rosidl_generator_traits::name<std_msgs::msg::String>(),
      "Topic under which the robot description is published.", this,
      SLOT(updateRobotDescriptionTopic()), this);

  joint_state_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Joint State Topic", "joint_states",
      rosidl_generator_traits::name<sensor_msgs::msg::JointState>(),
      "Topic under which the joint states are published.", this, SLOT(updateJointStateTopic()),
      this);
  point_cloud_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Point Cloud Topic", "point_cloud/cloud_registered",
      rosidl_generator_traits::name<sensor_msgs::msg::PointCloud2>(),
      "Topic under which the point cloud is published.", this, SLOT(updatePointCloudTopic()), this);
  roboreg_node_name_property_ = new rviz_common::properties::StringProperty(
      "Roboreg Node Name", "roboreg", "The node name under which the roboreg server lives.", this,
      SLOT(updateRoboregNodeNode()), this);
}

void Display::onInitialize() {
  rviz_common::Display::onInitialize();
  // get node
  auto node_abstraction = this->context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    throw std::runtime_error("Failed to lock node abstraction.");
  }
  node_ptr_ = node_abstraction->get_raw_node();

  // initialize properties
  robot_description_topic_property_->initialize(node_abstraction);
  joint_state_topic_property_->initialize(node_abstraction);
  point_cloud_topic_property_->initialize(node_abstraction);

  // add a collect data and save synced data widgets
  auto widget = new QWidget();
  collect_sample_widget_ = new CollectSampleWidget(node_ptr_, widget);
  register_widget_ = new RegisterWidget(node_ptr_, widget);
  export_samples_widget_ = new ExportSamplesWidget(node_ptr_, widget);
  setAssociatedWidget(widget);

  // set layout
  auto layout = new QVBoxLayout(widget);
  layout->addWidget(collect_sample_widget_);
  layout->addWidget(register_widget_);
  layout->addWidget(export_samples_widget_);
}

void Display::updateRobotDescriptionTopic() {
  RCLCPP_INFO(node_ptr_->get_logger(), "Robot description topic changed.");

  // set robot description topic for roboreg server (this requires a parameter callback to update
  // the subscriber in roboreg server)
}

void Display::updateJointStateTopic() {
  RCLCPP_INFO(node_ptr_->get_logger(), "Joint state topic changed.");
}

void Display::updatePointCloudTopic() {
  RCLCPP_INFO(node_ptr_->get_logger(), "Point cloud topic changed.");
}

void Display::updateRoboregNodeNode() {
  collect_sample_widget_->setupClient(roboreg_node_name_property_->getStdString());
  register_widget_->setupClient(roboreg_node_name_property_->getStdString());
  export_samples_widget_->setupClient(roboreg_node_name_property_->getStdString());
}
} // end of namespace ros2_roboreg_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_roboreg_rviz::Display, rviz_common::Display)
