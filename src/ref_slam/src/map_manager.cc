#include "../include/map_manager.hpp"
namespace reflector_slam {
MapManager::MapManager() {
  map_image = cv::Mat(4000, 4000, CV_8UC3, cv::Scalar(255, 255, 255));
  map_image_with_ref = cv::Mat(4000, 4000, CV_8UC3, cv::Scalar(255, 255, 255));
}
bool MapManager::generate_from_keyframe(
    std::unordered_map<int, Reflector>& reflectors,
    std::unordered_map<int, Keyframe>& keyframes,
    std::unordered_map<int, Eigen::Matrix4d>& pose,
    std::unordered_map<int, Eigen::Vector3d>& ref_pose)  // 每米对应像素
{
  double map_origin_x = -50;
  double map_origin_y = -50;
  double map_scale = 40.0;
  if (keyframes.empty() || reflectors.empty()) {
    return false;
  }

  map.clear();
  // 1. 更新 map 内部反光板（可选）
  for (auto& kv : reflectors) {
    int id = kv.first;
    if (ref_pose.find(id) != ref_pose.end()) {
      kv.second.position = ref_pose[id];  // 优化后的反光板位置
      map[id] = kv.second;
    }
  }

  // 2. 绘制关键帧、反光板和雷达点
  for (auto& kv : keyframes) {
    int kfid = kv.first;
    Keyframe& kf = kv.second;

    if (pose.find(kfid) == pose.end()) continue;  // 防止缺失
    Eigen::Matrix4d T = pose[kfid];               // 优化后的关键帧位姿

    // 绘制雷达点（绿色）
    if (kf.scan) {
      int num_points = kf.scan->ranges.size();
      for (int i = 0; i < num_points; ++i) {
        float r = kf.scan->ranges[i];
        if (r < kf.scan->range_min || r > kf.scan->range_max) continue;

        float angle = kf.scan->angle_min + i * kf.scan->angle_increment;
        Eigen::Vector4d p_sensor(r * cos(angle), r * sin(angle), 0.0, 1.0);
        Eigen::Vector4d p_world = T * p_sensor;  // 优化后的关键帧位姿

        int px = static_cast<int>((p_world(0) - map_origin_x) * map_scale);
        int py = map_image_with_ref.rows -
                 static_cast<int>((p_world(1) - map_origin_y) * map_scale);
        cv::circle(map_image_with_ref, cv::Point(px, py), 1,
                   cv::Scalar(0, 255, 0), -1);
        cv::circle(map_image, cv::Point(px, py), 1, cv::Scalar(0, 255, 0), -1);
      }
    }
    // 绘制关键帧位置（蓝色）
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    cv::circle(
        map_image_with_ref,
        cv::Point(static_cast<int>((t(0) - map_origin_x) * map_scale),
                  map_image_with_ref.rows -
                      static_cast<int>((t(1) - map_origin_y) * map_scale)),
        1, cv::Scalar(255, 0, 0), -1);

    // 绘制关键帧观测的反光板（红色）
    for (auto& obs : kf.observations) {
      int rid = obs.id;
      if (rid >= 0 && ref_pose.find(rid) != ref_pose.end()) {
        Eigen::Vector3d p = ref_pose[rid];  // 优化后的反光板
        cv::circle(
            map_image_with_ref,
            cv::Point(static_cast<int>((p(0) - map_origin_x) * map_scale),
                      map_image_with_ref.rows -
                          static_cast<int>((p(1) - map_origin_y) * map_scale)),
            std::ceil((0.035) * map_scale), cv::Scalar(0, 0, 255), -1);
      }
    }
  }

  valid = true;
  return true;
}

void MapManager::save_map(const std::string& path) {
  cv::imwrite(path + "/map.png", map_image);
  cv::imwrite(path + "/map_ref.png", map_image_with_ref);
}

}  // namespace reflector_slam
