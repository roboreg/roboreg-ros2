#include "roboreg_rviz/formatting.hpp"

namespace roboreg_rviz {
std::string format_topic(const std::string &topic, const std::string &ns) {
  if (topic.empty()) {
    throw std::invalid_argument("Topic cannot be empty");
  }
  return "/" + (ns.empty() ? "" : ns + "/") + topic;
}
} // end of namespace roboreg_rviz
