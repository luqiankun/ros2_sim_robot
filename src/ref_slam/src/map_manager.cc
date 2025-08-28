#include "../include/map_manager.hpp"

#include <fstream>
namespace reflector_slam {
MapManager::MapManager() {
  map_image = cv::Mat(4000, 4000, CV_8UC3, cv::Scalar(255, 255, 255));
  comp_map_image = cv::Mat(4000, 4000, CV_8UC3, cv::Scalar(255, 255, 255));
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
  ref_map.clear();
  valid = false;
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "map_info" << YAML::Value << YAML::BeginMap << YAML::Key
      << "source" << YAML::Value << "robot_A" << YAML::Key << "resolution"
      << YAML::Value << 1 << YAML::Key << "origin" << YAML::Value << YAML::Flow
      << std::vector<double>{0, 0, 0} << YAML::EndMap;
  out << YAML::Key << "landmarks" << YAML::Value << YAML::BeginSeq;

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

    // 绘制雷达点（黑色）
    if (kf.scan) {
      int num_points = kf.scan->ranges.size();
      for (int i = 0; i < num_points; ++i) {
        float r = kf.scan->ranges[i];
        if (r < kf.scan->range_min || r > kf.scan->range_max) continue;

        float angle = kf.scan->angle_min + i * kf.scan->angle_increment;
        Eigen::Vector4d p_sensor(r * cos(angle), r * sin(angle), 0.0, 1.0);
        Eigen::Vector4d p_world = T * p_sensor;  // 优化后的关键帧位姿

        int px = static_cast<int>((p_world(0) - map_origin_x) * map_scale);
        int py = map_image.rows -
                 static_cast<int>((p_world(1) - map_origin_y) * map_scale);
        cv::circle(map_image, cv::Point(px, py), 1, cv::Scalar(0, 0, 0), -1);
        cv::circle(comp_map_image, cv::Point(px, py), 1, cv::Scalar(0, 0, 0),
                   -1);
      }
    }
    // 绘制关键帧位置（蓝色）
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    cv::circle(
        comp_map_image,
        cv::Point(static_cast<int>((t(0) - map_origin_x) * map_scale),
                  comp_map_image.rows -
                      static_cast<int>((t(1) - map_origin_y) * map_scale)),
        3, cv::Scalar(255, 0, 0), -1);

    // 绘制关键帧观测的反光板（红色）

    for (auto& obs : kf.observations) {
      int rid = obs.id;
      if (rid >= 0 && ref_pose.find(rid) != ref_pose.end()) {
        Eigen::Vector3d p = ref_pose[rid];  // 优化后的反光板
        out << YAML::BeginMap;
        out << YAML::Key << "id" << YAML::Value << rid;
        out << YAML::Key << "x" << YAML::Value << p.x();
        out << YAML::Key << "y" << YAML::Value << p.y();
        out << YAML::EndMap;
        cv::circle(
            comp_map_image,
            cv::Point(static_cast<int>((p(0) - map_origin_x) * map_scale),
                      comp_map_image.rows -
                          static_cast<int>((p(1) - map_origin_y) * map_scale)),
            5, cv::Scalar(0, 0, 255), -1);
      }
    }
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  ref_map.clear();
  ref_map = out.c_str();
  valid = true;
  return true;
}

void MapManager::save_map(const std::string& path) {
  cv::imwrite(path + "/map.png", map_image);
  cv::imwrite(path + "/map_comp.png", comp_map_image);
  std::ofstream ofs((path + "/map.yaml").c_str(),
                    std::ios::out | std::ios::trunc);
  ofs << ref_map;
  ofs.close();
}

}  // namespace reflector_slam
