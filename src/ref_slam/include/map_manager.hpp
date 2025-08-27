#ifndef MAP_MANAGER_HPP
#define MAP_MANAGER_HPP
#include <opencv2/opencv.hpp>

#include "./common.hpp"
namespace reflector_slam {
class MapManager {
 public:
  MapManager();
  bool load_from_file(const std::string& filename);
  std::unordered_map<int, Reflector> get_map() { return map; }
  bool generate_from_keyframe(
      std::unordered_map<int, Reflector>& reflectors,
      std::unordered_map<int, Keyframe>& keyframes,
      std::unordered_map<int, Eigen::Matrix4d>& pose,
      std::unordered_map<int, Eigen::Vector3d>& ref_pose);
  void save_map(const std::string& filename);

 private:
  bool valid{false};
  std::unordered_map<int, Reflector> map;
  cv::Mat map_image;
  cv::Mat map_image_with_ref;
};
}  // namespace reflector_slam

#endif