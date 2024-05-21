#include "ros2_roboreg_rviz/save_data_widget.hpp"

namespace ros2_roboreg_rviz {

SaveDataWidget::SaveDataWidget(const rclcpp::Node::SharedPtr node_ptr, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  save_data_client_ptr_ =
      node_ptr_->create_client<ros2_roboreg_idl::srv::SaveData>("save_synced_data");

  // button
  save_data_button_ = new QPushButton("Save synchronized data", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(save_data_button_);
  this->setLayout(layout);

  // button callback
  connect(save_data_button_, &QPushButton::clicked, this, &SaveDataWidget::onSaveData_);
}

void SaveDataWidget::onSaveData_() {
  auto path = QFileDialog::getExistingDirectory(nullptr, "Select output path", QDir::homePath());

  if (!save_data_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 save_data_client_ptr_->get_service_name());
    return;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Saving synchronized data...");
}
} // end of namespace ros2_roboreg_rviz
