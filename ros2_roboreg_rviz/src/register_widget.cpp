#include "ros2_roboreg_rviz/register_widget.hpp"

namespace ros2_roboreg_rviz {
RegisterWidget::RegisterWidget(rclcpp::Node::SharedPtr node_ptr, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient("roboreg");

  // button
  register_button_ = new QPushButton("Register", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(register_button_);
  this->setLayout(layout);

  // button callback
  connect(register_button_, &QPushButton::clicked, this, &RegisterWidget::onRegister_);
}

void RegisterWidget::setupClient(const std::string &roboreg_nodename) {
  if (register_client_ptr_) {
    register_client_ptr_.reset();
  }
  register_client_ptr_ =
      node_ptr_->create_client<std_srvs::srv::Trigger>("/" + roboreg_nodename + "/register");
}

void RegisterWidget::onRegister_() {
  if (!register_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Register client not initialized.");
    return;
  }
  if (!register_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 register_client_ptr_->get_service_name());
    return;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Registering...");
}
} // end of namespace ros2_roboreg_rviz