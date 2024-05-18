#include "ros2_roboreg_rviz/save_synced_data_widget.hpp"

namespace ros2_roboreg_rviz {

SaveSyncedDataWidget::SaveSyncedDataWidget(const rclcpp::Node::SharedPtr node_ptr, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  save_synced_data_client_ptr_ =
      node_ptr_->create_client<ros2_roboreg_idl::srv::SaveSyncedData>("save_synced_data");

  // button
  save_synced_data_button_ = new QPushButton("Save synchronized data", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(save_synced_data_button_);
  this->setLayout(layout);

  // button callback
  connect(save_synced_data_button_, &QPushButton::clicked, this,
          &SaveSyncedDataWidget::onSaveSyncedData_);
}

void SaveSyncedDataWidget::onSaveSyncedData_() {
  auto path = QFileDialog::getExistingDirectory(nullptr, "Select output path", QDir::homePath());

  if (!save_synced_data_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 save_synced_data_client_ptr_->get_service_name());
    return;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Saving synchronized data...");
}
} // end of namespace ros2_roboreg_rviz
