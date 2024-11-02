#ifndef ROS2_ROBOREG_RVIZ__REGISTER_WIDGET_HPP_
#define ROS2_ROBOREG_RVIZ__REGISTER_WIDGET_HPP_

#include <QBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "ros2_roboreg_idl/srv/reg_hydra_icp.hpp"

namespace ros2_roboreg_rviz {
class RegisterWidget : public QWidget {
public:
  RegisterWidget(rclcpp::Node::SharedPtr node_ptr, const std::string &roboreg_node_name = "roboreg",
                 QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_node_name);

protected:
  void onRegister_();
  void onBroadcastTF_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<ros2_roboreg_idl::srv::RegHydraICP>::SharedPtr register_client_ptr_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr broadcast_tf_client_ptr_;

  QPushButton *register_button_;
  QPushButton *broadcast_tf_button_;
};
} // end of namespace ros2_roboreg_rviz

#endif // ROS2_ROBOREG_RVIZ__REGISTER_WIDGET_HPP_
