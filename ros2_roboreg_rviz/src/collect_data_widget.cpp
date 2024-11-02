#include "ros2_roboreg_rviz/collect_data_widget.hpp"

namespace ros2_roboreg_rviz {
CollectDataWidget::CollectDataWidget(rclcpp::Node::SharedPtr node_ptr,
                                     const std::string &roboreg_node_name, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient(roboreg_node_name);

  // button and count label
  collect_data_button_ = new QPushButton("Collect Data", this);
  clear_data_button_ = new QPushButton("Clear Data", this);
  count_display_ = new QLabel("Collected data points: 0", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(collect_data_button_);
  layout->addWidget(clear_data_button_);
  layout->addWidget(count_display_);
  this->setLayout(layout);

  // button callbacks
  connect(collect_data_button_, &QPushButton::clicked, this, &CollectDataWidget::onCollectData_);
  connect(clear_data_button_, &QPushButton::clicked, this, &CollectDataWidget::onClearData_);
}

void CollectDataWidget::setupClient(const std::string &roboreg_node_name) {
  if (collect_data_client_ptr_) {
    collect_data_client_ptr_.reset();
  }
  collect_data_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::CollectData>(
      "/" + roboreg_node_name + "/collect_data");
  if (clear_data_client_ptr_) {
    clear_data_client_ptr_.reset();
  }
  clear_data_client_ptr_ =
      node_ptr_->create_client<std_srvs::srv::Trigger>("/" + roboreg_node_name + "/clear_data");
}

void CollectDataWidget::onCollectData_() {
  if (!collect_data_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Collect data client not initialized.");
    return;
  }
  if (!collect_data_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 collect_data_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<ros2_roboreg_idl::srv::CollectData::Request>();
  auto future = collect_data_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::CollectData>::SharedFuture result) {
        if (result.get()->success) {
          count_display_->setText(
              QString("Collected data points: %1").arg(result.get()->n_collected));
        }
      });
}

void CollectDataWidget::onClearData_() {
  if (!clear_data_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Clear data client not initialized.");
    return;
  }
  if (!clear_data_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 clear_data_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
  auto future = clear_data_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture result) {
        if (result.get()->success) {
          count_display_->setText("Collected data points: 0");
        }
      });
}
} // end of namespace ros2_roboreg_rviz
