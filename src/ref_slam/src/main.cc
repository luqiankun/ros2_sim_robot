#include <rclcpp/rclcpp.hpp>

#include "../include/map_optimization.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("slam");
  auto map = std::make_shared<reflector_slam::MapOptimization>(node);
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  std::cout << node->get_parameter("use_sim_time").as_bool() << "\n";
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}