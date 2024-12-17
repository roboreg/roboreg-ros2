#include "ros2_roboreg_rviz/display.hpp"

namespace ros2_roboreg_rviz {
Display::Display() : roboreg_namespace_("") {
  robot_description_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Description Topic", "/robot_description",
      rosidl_generator_traits::name<std_msgs::msg::String>(),
      "Topic under which the robot description is published.", this,
      SLOT(updateRobotDescriptionTopic()), this);
  joint_state_topic_property_ = new rviz_common::properties::RosTopicProperty(
      "Joint State Topic", "/joint_states",
      rosidl_generator_traits::name<sensor_msgs::msg::JointState>(),
      "Topic under which the joint states are published.", this, SLOT(updateJointStateTopic()),
      this);
  roboreg_namespace_property_ = new rviz_common::properties::StringProperty(
      "Roboreg Namespace", roboreg_namespace_.c_str(),
      "The namespace under which the roboreg server lives.", this, SLOT(updateRoboregNode()), this);
}

void Display::onInitialize() {
  rviz_common::Display::onInitialize();
  // get node
  auto node_abstraction = this->context_->getRosNodeAbstraction().lock();
  if (!node_abstraction) {
    throw std::runtime_error("Failed to lock node abstraction.");
  }
  node_ptr_ = node_abstraction->get_raw_node();

  parameters_client_ =
      std::make_unique<rclcpp::AsyncParametersClient>(node_ptr_, roboreg_namespace_);

  // initialize properties
  robot_description_topic_property_->initialize(node_abstraction);
  joint_state_topic_property_->initialize(node_abstraction);

  // add a collect data and save synced data widgets
  auto widget = new QWidget();
  collect_data_widget_ = new CollectDataWidget(node_ptr_, roboreg_namespace_, widget);
  register_widget_ = new RegisterWidget(node_ptr_, roboreg_namespace_, widget);
  io_widget_ = new IOWidget(node_ptr_, roboreg_namespace_, widget);
  setAssociatedWidget(widget);

  // set layout
  auto layout = new QVBoxLayout(widget);
  layout->addWidget(collect_data_widget_);
  layout->addWidget(register_widget_);
  layout->addWidget(io_widget_);
}

void Display::updateRobotDescriptionTopic() {
  auto topic = robot_description_topic_property_->getStdString();
  RCLCPP_INFO(node_ptr_->get_logger(), "Updating robot description topic to: %s", topic.c_str());
  parameters_client_->set_parameters({rclcpp::Parameter("topics.robot_description.name", topic)});
}

void Display::updateJointStateTopic() {
  auto topic = joint_state_topic_property_->getStdString();
  RCLCPP_INFO(node_ptr_->get_logger(), "Updating joint state topic to: %s", topic.c_str());
  parameters_client_->set_parameters({rclcpp::Parameter("topics.joint_state.name", topic)});
}

void Display::updateRoboregNode() {
  roboreg_namespace_ = roboreg_namespace_property_->getStdString();
  collect_data_widget_->setupClient(roboreg_namespace_);
  register_widget_->setupClient(roboreg_namespace_);
  io_widget_->setupClient(roboreg_namespace_);
  parameters_client_.reset();
  parameters_client_ =
      std::make_unique<rclcpp::AsyncParametersClient>(node_ptr_, roboreg_namespace_);
}
} // end of namespace ros2_roboreg_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_roboreg_rviz::Display, rviz_common::Display)
