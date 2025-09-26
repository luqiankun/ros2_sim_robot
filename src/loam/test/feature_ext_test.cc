#include "../include/feature_ext.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("feature_ext_test");
  auto feature_ext = std::make_shared<loam::FeatureExtractor>(node);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}