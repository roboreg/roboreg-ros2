#ifndef ROBOREG_RVIZ__REGISTER_WIDGET_HPP_
#define ROBOREG_RVIZ__REGISTER_WIDGET_HPP_

#include <QBoxLayout>
#include <QPushButton>
#include <QWidget>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "roboreg_idl/srv/reg_hydra_icp.hpp"
#include "roboreg_rviz/formatting.hpp"

namespace roboreg_rviz {
class RegisterWidget : public QWidget {
public:
  RegisterWidget(rclcpp::Node::SharedPtr node_ptr, const std::string &roboreg_namespace = "",
                 QWidget *parent = nullptr);

  void setupClient(const std::string &roboreg_namespace = "");

protected:
  void onRegister_();
  void onBroadcastTF_();

protected:
  rclcpp::Node::SharedPtr node_ptr_;
  rclcpp::Client<roboreg_idl::srv::RegHydraICP>::SharedPtr register_client_ptr_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr broadcast_tf_client_ptr_;

  QPushButton *register_button_;
  QPushButton *broadcast_tf_button_;
};
} // end of namespace roboreg_rviz

#endif // ROBOREG_RVIZ__REGISTER_WIDGET_HPP_
