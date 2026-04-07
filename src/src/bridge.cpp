#include "ros/bridge_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<BridgeNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
