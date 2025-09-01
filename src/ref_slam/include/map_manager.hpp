#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP
#include <yaml-cpp/yaml.h>

#include <opencv2/opencv.hpp>

#include "./common.hpp"
namespace reflector_slam {
class MapManager {
 public:
  MapManager(double map_resolution, double map_origin_x, double map_origin_y,
             double map_height, double map_width, const std::string& path);
  MapManager();
  bool load_from_file(const std::string& filename);
  std::unordered_map<int, Reflector> get_map() {
    if (!valid) return {};
    return map;
  }
  bool generate_from_keyframe(
      std::unordered_map<int, Reflector>& reflectors,
      std::unordered_map<int, Keyframe>& keyframes,
      std::unordered_map<int, Eigen::Matrix4d>& pose,
      std::unordered_map<int, Eigen::Vector3d>& ref_pose);
  bool generate_occupancy_grid(
      std::unordered_map<int, Keyframe>& keyframes,
      std::unordered_map<int, Eigen::Matrix4d>& pose,
      std::unordered_map<int, Eigen::Vector3d>& ref_pose);
  void save_map();

 private:
  bool valid{false};
  std::unordered_map<int, Reflector> map;
  cv::Mat map_image;       // 纯雷达
  cv::Mat comp_map_image;  // 复合信息，反光板，雷达，路径
  cv::Mat occupancy_grid;
  std::string ref_map;
  // params
  double map_resolution{0.05};
  double map_width{1000};
  double map_height{1000};
  double map_origin_x{0};
  double map_origin_y{0};
  std::string save_path{"."};
};
}  // namespace reflector_slam

#endif