#include "ros2_roboreg_rviz/io_widget.hpp"

namespace ros2_roboreg_rviz {

IOWidget::IOWidget(const rclcpp::Node::SharedPtr node_ptr, const std::string &roboreg_namespace,
                   const std::string &roboreg_node_name, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient(roboreg_namespace, roboreg_node_name);

  // button
  export_data_button_ = new QPushButton("Export Data", this);
  export_tf_button_ = new QPushButton("Export Transform", this);
  import_tf_button_ = new QPushButton("Import Transform", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(export_data_button_);
  layout->addWidget(export_tf_button_);
  layout->addWidget(import_tf_button_);
  this->setLayout(layout);

  // button callback
  connect(export_data_button_, &QPushButton::clicked, this, &IOWidget::onExportData_);
  connect(export_tf_button_, &QPushButton::clicked, this, &IOWidget::onExportTF_);
  connect(import_tf_button_, &QPushButton::clicked, this, &IOWidget::onImportTF_);
}

void IOWidget::setupClient(const std::string &roboreg_namespace,
                           const std::string &roboreg_node_name) {
  if (export_data_client_ptr_) {
    export_data_client_ptr_.reset();
  }
  export_data_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::Export>(
      format_topic(roboreg_node_name + "/export/data", roboreg_namespace));

  if (export_tf_client_ptr_) {
    export_tf_client_ptr_.reset();
  }
  export_tf_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::Export>(
      format_topic(roboreg_node_name + "/export/transform", roboreg_namespace));

  if (import_tf_client_ptr_) {
    import_tf_client_ptr_.reset();
  }
  import_tf_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::Import>(
      format_topic(roboreg_node_name + "/import/transform", roboreg_namespace));
}

void IOWidget::onExportData_() {
  if (!export_data_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Export data client not initialized");
    return;
  }
  auto path = QFileDialog::getExistingDirectory(nullptr, "Select output path", QDir::homePath());
  if (!export_data_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 export_data_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<ros2_roboreg_idl::srv::Export::Request>();
  request->mkdir = false;
  request->path = path.toStdString();
  auto future = export_data_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::Export>::SharedFuture result) {
        if (result.get()->success) {
          RCLCPP_INFO(node_ptr_->get_logger(), "Exported data");
        } else {
          RCLCPP_ERROR(node_ptr_->get_logger(), "Failed to export data");
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
