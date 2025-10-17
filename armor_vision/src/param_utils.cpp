#include <rclcpp/rclcpp.hpp>

namespace armor_vision {

template<typename T>
static T get_param(rclcpp::Node* node, const std::string& name, const T& def) {
  node->declare_parameter<T>(name, def);
  T v; node->get_parameter(name, v); return v;
}

} // namespace armor_vision

