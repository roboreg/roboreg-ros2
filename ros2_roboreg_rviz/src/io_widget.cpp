#include "ros2_roboreg_rviz/io_widget.hpp"

namespace ros2_roboreg_rviz {

IOWidget::IOWidget(const rclcpp::Node::SharedPtr node_ptr, const std::string &roboreg_node_name,
                   QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient(roboreg_node_name);

  // button
  export_samples_button_ = new QPushButton("Export Samples", this);
  export_tf_button_ = new QPushButton("Export Transform", this);
  import_tf_button_ = new QPushButton("Import Transform", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(export_samples_button_);
  layout->addWidget(export_tf_button_);
  layout->addWidget(import_tf_button_);
  this->setLayout(layout);

  // button callback
  connect(export_samples_button_, &QPushButton::clicked, this, &IOWidget::onExportSamples_);
  connect(export_tf_button_, &QPushButton::clicked, this, &IOWidget::onExportTF_);
  connect(import_tf_button_, &QPushButton::clicked, this, &IOWidget::onImportTF_);
}

void IOWidget::setupClient(const std::string &roboreg_node_name) {
  if (export_samples_client_ptr_) {
    export_samples_client_ptr_.reset();
  }
  export_samples_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::Export>(
      "/" + roboreg_node_name + "/export/samples");

  if (export_tf_client_ptr_) {
    export_tf_client_ptr_.reset();
  }
  export_tf_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::Export>(
      "/" + roboreg_node_name + "/export/transform");

  if (import_tf_client_ptr_) {
    import_tf_client_ptr_.reset();
  }
  import_tf_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::Import>(
      "/" + roboreg_node_name + "/import/transform");
}

void IOWidget::onExportSamples_() {
  if (!export_samples_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Export samples client not initialized");
    return;
  }
  auto path = QFileDialog::getExistingDirectory(nullptr, "Select output path", QDir::homePath());
  if (!export_samples_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 export_samples_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<ros2_roboreg_idl::srv::Export::Request>();
  request->mkdir = false;
  request->path = path.toStdString();
  auto future = export_samples_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::Export>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Exported samples");
        } else {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to export samples");
        }
      });
}

void IOWidget::onExportTF_() {
  if (!export_tf_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Export transform client not initialized");
    return;
  }
  auto path = QFileDialog::getSaveFileName(nullptr, "Select output path", QDir::homePath(),
                                           "Transform files (*.npy)");
  if (!export_tf_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 export_tf_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<ros2_roboreg_idl::srv::Export::Request>();
  request->mkdir = false;
  request->path = path.toStdString() + ".npy";
  auto future = export_tf_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::Export>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Exported transform");
        } else {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to export transform");
        }
      });
}

void IOWidget::onImportTF_() {
  if (!import_tf_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Import transform client not initialized");
    return;
  }
  auto path = QFileDialog::getOpenFileName(nullptr, "Select input path", QDir::homePath(),
                                           "Transform files (*.npy)");
  if (!import_tf_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 import_tf_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<ros2_roboreg_idl::srv::Import::Request>();
  request->path = path.toStdString();
  auto future = import_tf_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::Import>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Imported transform");
        } else {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to import transform");
        }
      });
}
} // end of namespace ros2_roboreg_rviz
