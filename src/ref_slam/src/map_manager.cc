#include "../include/map_manager.hpp"

#include <fstream>
#include <opencv2/core/parallel/backend/parallel_for.openmp.hpp>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
namespace reflector_slam {
MapManager::MapManager() {
  map_image =
      cv::Mat(map_height, map_width, CV_8UC3, cv::Scalar(255, 255, 255));
}

MapManager::MapManager(double map_resolution, double map_origin_x,
                       double map_origin_y, double map_height, double map_width,
                       const std::string& path)
    : map_resolution(map_resolution),
      map_width(map_width),
      map_height(map_height),
      map_origin_x(map_origin_x),
      map_origin_y(map_origin_y),
      save_path(path) {
  map_image =
      cv::Mat(map_height, map_width, CV_8UC3, cv::Scalar(255, 255, 255));
}

bool MapManager::generate_from_keyframe(
    std::unordered_map<int, Reflector>& reflectors,
    std::unordered_map<int, Keyframe>& keyframes,
    std::unordered_map<int, Eigen::Matrix4d>&,
    std::unordered_map<int, Eigen::Vector3d>& ref_pose)  // 每米对应像素
{
  if (keyframes.empty() || reflectors.empty()) {
    return false;
  }
  map.clear();
  ref_map.clear();
  valid = false;

  // 1. 更新 map 内部反光板（可选）
  for (auto& kv : reflectors) {
    int id = kv.first;
    if (ref_pose.find(id) != ref_pose.end()) {
      kv.second.position = ref_pose[id];  // 优化后的反光板位置
      map[id] = kv.second;
    }
  }
  YAML::Emitter out;
  out << YAML::BeginMap;
  out << YAML::Key << "map_info" << YAML::Value << YAML::BeginMap << YAML::Key
      << "source" << YAML::Value << "robot_A" << YAML::Key << "resolution"
      << YAML::Value << 1.0 << YAML::Key << "origin" << YAML::Value
      << YAML::Flow << std::vector<double>{0, 0, 0} << YAML::EndMap;
  out << YAML::Key << "landmarks" << YAML::Value << YAML::BeginSeq;
  for (auto& [id, ref] : reflectors) {
    Eigen::Vector3d p = ref_pose[id];  // 优化后的反光板
    out << YAML::BeginMap;
    out << YAML::Key << "id" << YAML::Value << id;
    out << YAML::Key << "x" << YAML::Value << p.x();
    out << YAML::Key << "y" << YAML::Value << p.y();
    out << YAML::EndMap;
  }
  out << YAML::EndSeq;
  out << YAML::EndMap;
  ref_map.clear();
  ref_map = out.c_str();
  valid = true;
  return true;
}
bool MapManager::load_from_file(const std::string& path) {
  map.clear();
  ref_map.clear();
  std::ifstream ifs(path);
  if (!ifs.is_open()) {
    return false;
  }
  YAML::Node node = YAML::Load(ifs);
  if (!node["map_info"]) {
    return false;
  }
  if (!node["landmarks"]) {
    return false;
  }
  double resolution = node["map_info"]["resolution"].as<double>();
  double ref_x = node["map_info"]["origin"][0].as<double>();
  double ref_y = node["map_info"]["origin"][1].as<double>();
  double ref_yaw = node["map_info"]["origin"][2].as<double>();
  Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
  T(0, 3) = ref_x;
  T(1, 3) = ref_y;
  Eigen::AngleAxis<double> rotate(ref_yaw, Eigen::Vector3d::UnitZ());
  T.block<3, 3>(0, 0) = rotate.toRotationMatrix();
  for (auto x : node["landmarks"]) {
    int id = x["id"].as<int>();
    double x_ = x["x"].as<double>();
    double y_ = x["y"].as<double>();
    Eigen::Vector4d p(x_, y_, 0, 1);
    p = T * p * resolution;
    map[id] = Reflector{p.head<3>(), id};
  }
  valid = true;
  return true;
}
void MapManager::save_map() {
  cv::imwrite(save_path + "/map.png", map_image);
  cv::imwrite(save_path + "/map_occupancy.png", occupancy_grid);
  std::ofstream ofs((save_path + "/map.yaml").c_str(),
                    std::ios::out | std::ios::trunc);
  ofs << ref_map;
  ofs.close();
}

bool MapManager::generate_occupancy_grid(
    std::unordered_map<int, Keyframe>& keyframes,
    std::unordered_map<int, Eigen::Matrix4d>& pose,
    std::unordered_map<int, Eigen::Vector3d>& ref_pose) {
  int grid_width = static_cast<int>(map_width);
  int grid_height = static_cast<int>(map_height);

  cv::Mat log_odds(grid_height, grid_width, CV_32FC1, cv::Scalar(0.0f));
  float l_occ = 0.85f, l_free = -0.4f, l_min = -2.0f, l_max = 3.5f;

  // 收集有效关键帧指针
  std::vector<Keyframe*> frame_ptrs;
  for (auto& kv : keyframes)
    if (pose.find(kv.first) != pose.end()) frame_ptrs.push_back(&kv.second);

// 并行处理每个关键帧
#pragma omp parallel for schedule(dynamic)
  for (int idx = 0; idx < static_cast<int>(frame_ptrs.size()); ++idx) {
    Keyframe* kf = frame_ptrs[idx];
    Eigen::Matrix4d T = pose.at(kf->id);
    Eigen::Vector3d sensor_origin = T.block<3, 1>(0, 3);

    cv::Mat local_log(grid_height, grid_width, CV_32FC1, cv::Scalar(0.0f));

    if (kf->scan) {
      size_t N = kf->scan->ranges.size();
      for (size_t i = 0; i < N; ++i) {
        float r = kf->scan->ranges[i];
        float angle = kf->scan->angle_min + i * kf->scan->angle_increment;

        bool hit_object =
            (r >= kf->scan->range_min && r <= kf->scan->range_max);
        float r_effective = hit_object ? r : kf->scan->range_max;

        Eigen::Vector4d p_sensor(r_effective * cos(angle),
                                 r_effective * sin(angle), 0.0, 1.0);
        Eigen::Vector4d p_world = T * p_sensor;

        int gx = static_cast<int>((p_world(0) - map_origin_x) / map_resolution);
        int gy = grid_height - 1 -
                 static_cast<int>((p_world(1) - map_origin_y) / map_resolution);
        int x0 = static_cast<int>((sensor_origin(0) - map_origin_x) /
                                  map_resolution);
        int y0 = grid_height - 1 -
                 static_cast<int>((sensor_origin(1) - map_origin_y) /
                                  map_resolution);

        // Bresenham 空闲更新
        int dx = abs(gx - x0), sx = x0 < gx ? 1 : -1;
        int dy = abs(gy - y0), sy = y0 < gy ? 1 : -1;
        int err = dx - dy, cx = x0, cy = y0;

        while (cx != gx || cy != gy) {
          if (cx >= 0 && cx < grid_width && cy >= 0 && cy < grid_height)
            local_log.at<float>(cy, cx) = std::min(
                std::max(local_log.at<float>(cy, cx) + l_free, l_min), l_max);
          int e2 = 2 * err;
          if (e2 > -dy) {
            err -= dy;
            cx += sx;
          }
          if (e2 < dx) {
            err += dx;
            cy += sy;
          }
        }

        // 终点
        if (gx >= 0 && gx < grid_width && gy >= 0 && gy < grid_height) {
          if (hit_object)
            local_log.at<float>(gy, gx) = std::min(
                std::max(local_log.at<float>(gy, gx) + l_occ, l_min), l_max);
          else
            local_log.at<float>(gy, gx) = std::min(
                std::max(local_log.at<float>(gy, gx) + l_free, l_min), l_max);
        }
      }
    }

    // 反光板占据格
    for (auto& obs : kf->observations) {
      if (ref_pose.find(obs.id) == ref_pose.end()) continue;
      Eigen::Vector3d p = ref_pose.at(obs.id);
      int gx = static_cast<int>((p(0) - map_origin_x) / map_resolution);
      int gy = grid_height - 1 -
               static_cast<int>((p(1) - map_origin_y) / map_resolution);
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) {
          int nx = gx + dx, ny = gy + dy;
          if (nx >= 0 && nx < grid_width && ny >= 0 && ny < grid_height)
            local_log.at<float>(ny, nx) = std::min(
                std::max(local_log.at<float>(ny, nx) + l_occ, l_min), l_max);
        }
    }

// 线程安全累加到全局 log_odds
#pragma omp critical
    log_odds += local_log;
  }

  // 转概率图 & 可视化
  cv::Mat display(grid_height, grid_width, CV_8UC1, cv::Scalar(127));
  for (int y = 0; y < grid_height; ++y)
    for (int x = 0; x < grid_width; ++x)
      display.at<uint8_t>(y, x) = static_cast<uint8_t>(
          (1.0f - (1.0f - 1.0f / (1.0f + exp(log_odds.at<float>(y, x))))) *
          255);

  cv::cvtColor(display, map_image, cv::COLOR_GRAY2BGR);

  for (auto& kv : keyframes) {
    if (pose.find(kv.first) == pose.end()) continue;
    Eigen::Matrix4d T = pose.at(kv.first);
    Eigen::Vector3d t = T.block<3, 1>(0, 3);
    int px = static_cast<int>((t(0) - map_origin_x) / map_resolution);
    int py = grid_height - 1 -
             static_cast<int>((t(1) - map_origin_y) / map_resolution);
    cv::circle(map_image, cv::Point(px, py), 2, cv::Scalar(255, 0, 0), -1);
  }

  for (auto& kv : ref_pose) {
    Eigen::Vector3d p = kv.second;
    int px = static_cast<int>((p(0) - map_origin_x) / map_resolution);
    int py = grid_height - 1 -
             static_cast<int>((p(1) - map_origin_y) / map_resolution);
    cv::circle(map_image, cv::Point(px, py), 3, cv::Scalar(0, 0, 255), -1);
  }

  occupancy_grid = display;
  return true;
}

}  // namespace reflector_slam
