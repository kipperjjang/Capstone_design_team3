#include "ros/test_node.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<TestNode>();
  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
