#ifndef ROBOREG_RVIZ__FORMATTING_HPP_
#define ROBOREG_RVIZ__FORMATTING_HPP_

#include <stdexcept>
#include <string>

namespace roboreg_rviz {
std::string format_topic(const std::string &topic, const std::string &ns = "");
} // end of namespace roboreg_rviz
#endif // ROBOREG_RVIZ__FORMATTING_HPP_
