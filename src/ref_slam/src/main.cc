#include <rclcpp/rclcpp.hpp>

#include "../include/map_optimization.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ref_slam_node");
  auto map = std::make_shared<reflector_slam::MapOptimization>(node);
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}