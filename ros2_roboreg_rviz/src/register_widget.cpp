#include "ros2_roboreg_rviz/register_widget.hpp"

namespace ros2_roboreg_rviz {
RegisterWidget::RegisterWidget(rclcpp::Node::SharedPtr node_ptr,
                               const std::string &roboreg_namespace, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient(roboreg_namespace);

  // button
  register_button_ = new QPushButton("Register", this);
  broadcast_tf_button_ = new QPushButton("Broadcast Transform", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(register_button_);
  layout->addWidget(broadcast_tf_button_);
  this->setLayout(layout);

  // button callback
  connect(register_button_, &QPushButton::clicked, this, &RegisterWidget::onRegister_);
  connect(broadcast_tf_button_, &QPushButton::clicked, this, &RegisterWidget::onBroadcastTF_);
}

void RegisterWidget::setupClient(const std::string &roboreg_namespace) {
  if (register_client_ptr_) {
    register_client_ptr_.reset();
  }
  register_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::RegHydraICP>(
      format_topic("register/hydra_icp", roboreg_namespace));
  if (broadcast_tf_client_ptr_) {
    broadcast_tf_client_ptr_.reset();
  }
  broadcast_tf_client_ptr_ = node_ptr_->create_client<std_srvs::srv::Trigger>(
      format_topic("broadcast_transform", roboreg_namespace));
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
  auto request = std::make_shared<ros2_roboreg_idl::srv::RegHydraICP::Request>();
  auto future = register_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::RegHydraICP>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Registered");
        } else {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to register");
        }
      });
}

void RegisterWidget::onBroadcastTF_() {

  if (!broadcast_tf_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Publish TF client not initialized.");
    return;
  }
  if (!broadcast_tf_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 broadcast_tf_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = broadcast_tf_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Published TF");
        } else {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to publish TF");
        }
      });
}
} // end of namespace ros2_roboreg_rviz
