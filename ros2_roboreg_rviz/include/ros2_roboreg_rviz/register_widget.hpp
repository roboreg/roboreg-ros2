#ifndef ROS2_ROBOREG_RVIZ__REGISTER_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__REGISTER_WIDGET_HPP_

#include <QBoxLayout>
#include <QPushButton>
#include <QWidget>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace ros2_roboreg_rviz {
class RegisterWidget : public QWidget {
public:
  RegisterWidget(rclcpp::Node::SharedPtr node_ptr, QWidget *parent = nullptr);

protected:
  void onRegister_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr register_client_ptr_;

  QPushButton *register_button_;
};
} // end of namespace ros2_roboreg_rviz

#endif // ROS2_ROBOREG_RVIZ__REGISTER_WIDGET_HPP_