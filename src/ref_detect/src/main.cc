#include "../include/detecter.hpp"
#include "../include/mapping.hpp"
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("ref_detect");
  auto detecter = std::make_shared<RefDetecter>(node);
  auto mapper = std::make_shared<Mapping>(node);
  mapper->read_map(
      "/home/luqk/ros2/sim_car/src/ref_detect/config/map_post.txt");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}