#include "ros2_roboreg_rviz/collect_sample_widget.hpp"

namespace ros2_roboreg_rviz {
CollectSampleWidget::CollectSampleWidget(rclcpp::Node::SharedPtr node_ptr, QWidget *parent)
    : QWidget(parent), node_ptr_(node_ptr) {
  setupClient("roboreg");

  // button and count label
  collect_sample_button_ = new QPushButton("Collect data", this);
  count_display_ = new QLabel("Collected samples: 0", this);

  // set layout
  auto layout = new QVBoxLayout(this);
  layout->addWidget(collect_sample_button_);
  layout->addWidget(count_display_);
  this->setLayout(layout);

  // button callback
  connect(collect_sample_button_, &QPushButton::clicked, this,
          &CollectSampleWidget::onCollectSample_);
}

void CollectSampleWidget::setupClient(const std::string &roboreg_nodename) {
  if (collect_sample_client_ptr_) {
    collect_sample_client_ptr_.reset();
  }
  collect_sample_client_ptr_ = node_ptr_->create_client<ros2_roboreg_idl::srv::CollectSample>(
      "/" + roboreg_nodename + "/collect_sample");
}

void CollectSampleWidget::onCollectSample_() {
  if (!collect_sample_client_ptr_) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Collect data client not initialized.");
    return;
  }
  if (!collect_sample_client_ptr_->wait_for_service(std::chrono::seconds(1))) {
    RCLCPP_ERROR(node_ptr_->get_logger(), "Service %s not available.",
                 collect_sample_client_ptr_->get_service_name());
    return;
  }
  auto request = std::make_shared<ros2_roboreg_idl::srv::CollectSample::Request>();
  auto future = collect_sample_client_ptr_->async_send_request(
      request, [this](rclcpp::Client<ros2_roboreg_idl::srv::CollectSample>::SharedFuture result) {
        if (result.get()->success) {
          count_display_->setText(QString("Collected samples: %1").arg(result.get()->n_collected));
        }
      });
}
} // end of namespace ros2_roboreg_rviz
