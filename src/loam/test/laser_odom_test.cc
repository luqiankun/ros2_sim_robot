#include "../include/laser_mapping.hpp"
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("laser_odom_test");
  auto mappping = std::make_shared<loam::LaserMapping>(node);
  mappping->Init();
  mappping->Run();
  rclcpp::spin(node);
  mappping->Save();
  mappping->Stop();
  rclcpp::shutdown();

  return 0;
}
