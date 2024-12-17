#ifndef ROS2_ROBOREG_RVIZ__FORMATTING_HPP_
#define ROS2_ROBOREG_RVIZ__FORMATTING_HPP_

#include <stdexcept>
#include <string>

namespace ros2_roboreg_rviz {
std::string format_topic(const std::string &topic, const std::string &ns = "");
} // end of namespace ros2_roboreg_rviz
#endif // ROS2_ROBOREG_RVIZ__FORMATTING_HPP_
