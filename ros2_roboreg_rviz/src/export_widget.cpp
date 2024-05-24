#include "ros2_roboreg_rviz/export_widget.hpp"

namespace ros2_roboreg_rviz {

ExportSamplesWidget::ExportSamplesWidget(const rclcpp::Node::SharedPtr node_ptr, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient("roboreg");

  // button
  export_samples_button_ = new QPushButton("Save synchronized data", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(export_samples_button_);
  this->setLayout(layout);

  // button callback
  connect(export_samples_button_, &QPushButton::clicked, this,
          &ExportSamplesWidget::onExportSamples_);
}

void ExportSamplesWidget::setupClient(const std::string &roboreg_nodename) {
  if (export_samples_client_ptr_) {
    export_samples_client_ptr_.reset();
  }
  export_samples_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::ExportSamples>(
      "/" + roboreg_nodename + "/export/samples");
}

void ExportSamplesWidget::onExportSamples_() {
  if (!export_samples_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Save data client not initialized.");
    return;
  }
  auto path = QFileDialog::getExistingDirectory(nullptr, "Select output path", QDir::homePath());

  if (!export_samples_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 export_samples_client_ptr_->get_service_name());
    return;
  }
  RCLCPP_INFO(node_ptr_->get_logger(), "Saving synchronized data...");
}
} // end of namespace ros2_roboreg_rviz
