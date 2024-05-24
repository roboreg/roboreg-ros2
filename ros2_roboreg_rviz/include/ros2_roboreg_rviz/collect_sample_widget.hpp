#ifndef ROS2_ROBOREG_RVIZ__COLLECT_SAMPLE_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__COLLECT_SAMPLE_WIDGET_HPP_

#include <QBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QWidget>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "ros2_roboreg_idl/srv/collect_sample.hpp"

namespace ros2_roboreg_rviz {
class CollectSampleWidget : public QWidget {

public:
  CollectSampleWidget(rclcpp::Node::SharedPtr node_ptr,
                      const std::string &roboreg_node_name = "roboreg", QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_node_name);

protected:
  void onCollectSample_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::CollectSample>::SharedPtr collect_sample_client_ptr_;

  QPushButton *collect_sample_button_;
  QLabel *count_display_;
};
} // end of namespace ros2_roboreg_rviz
#endif // ROS2_ROBOREG_RVIZ__COLLECT_SAMPLE_WIDGET_HPP_