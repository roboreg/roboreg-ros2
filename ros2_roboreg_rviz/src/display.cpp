#include "ros2_roboreg_rviz/display.hpp"

namespace ros2_roboreg_rviz {

void Display::onInitialize() {
  node_ptr_ = this->context_->getRosNodeAbstraction().lock()->get_raw_node();
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
} // end of namespace ros2_roboreg_rviz

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ros2_roboreg_rviz::Display, rviz_common::Display)
