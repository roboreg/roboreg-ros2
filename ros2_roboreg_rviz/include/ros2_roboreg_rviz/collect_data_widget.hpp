#ifndef ROS2_ROBOREG_RVIZ__COLLECT_DATA_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__COLLECT_DATA_WIDGET_HPP_

#include <QBoxLayout>
#include <QLabel>
#include <QPushButton>
#include <QWidget>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros2_roboreg_idl/srv/collect_data.hpp"
#include "ros2_roboreg_rviz/formatting.hpp"

namespace ros2_roboreg_rviz {
class CollectDataWidget : public QWidget {

public:
  CollectDataWidget(rclcpp::Node::SharedPtr node_ptr, const std::string &roboreg_namespace = "",
                    const std::string &roboreg_node_name = "roboreg", QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_namespace = "",
                   const std::string &roboreg_node_name = "roboreg");

protected:
  void onCollectData_();
  void onClearData_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::CollectData>::SharedPtr collect_data_client_ptr_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr clear_data_client_ptr_;

  QPushButton *collect_data_button_;
  QPushButton *clear_data_button_;
  QLabel *count_display_;
};
} // end of namespace ros2_roboreg_rviz
#endif // ROS2_ROBOREG_RVIZ__COLLECT_DATA_WIDGET_HPP_