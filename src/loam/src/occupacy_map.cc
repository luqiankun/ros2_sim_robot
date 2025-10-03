#include "../include/occupacy_gridmap.hpp"
namespace loam {
void MapPublisher::generateMapImage(cv::Mat& out_img) {
  {
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    key_frames_front_ = (key_frames_back_);
  }
  int width = map_width_;
  int height = map_height_;
  occupancy_grid_ = cv::Mat(height, width, CV_32FC1, 0.0f);

  float l_occ = 0.85f, l_free = -0.4f, l_min = -2.0f, l_max = 3.5f;

  // 收集需要绘制的关键帧
  std::vector<KeyFrame*> frames_to_draw;
  if (local_mode_) {
    for (int id : loc_window_)
      if (key_frames_front_.find(id) != key_frames_front_.end())
        frames_to_draw.push_back(&key_frames_front_[id]);
  } else {
    for (auto& kv : key_frames_front_) frames_to_draw.push_back(&kv.second);
  }

  // 遍历关键帧
#pragma omp parallel for schedule(dynamic)
  for (const auto& kf : frames_to_draw) {
    // === 激光雷达扫描 ===
    auto kf_sensor_pose = kf->pose;
    cv::Mat local_log(height, width, CV_32FC1, cv::Scalar(0.0f));
    if (kf->scan) {
      size_t N = kf->scan->ranges.size();
      for (size_t i = 0; i < N; ++i) {
        float r = kf->scan->ranges[i];
        float angle = kf->scan->angle_min + i * kf->scan->angle_increment;
        bool hit_object =
            (r >= kf->scan->range_min && r <= kf->scan->range_max);
        float r_effective = hit_object ? r : kf->scan->range_max;
        Eigen::Vector2d p_sensor(r_effective * cos(angle),
                                 r_effective * sin(angle));
        Eigen::Vector2d p_world = kf_sensor_pose * p_sensor;
        int gx = static_cast<int>((p_world(0) - origin_x_) / map_resolution_);
        int gy = height - 1 -
                 static_cast<int>((p_world(1) - origin_y_) / map_resolution_);
        int x0 = static_cast<int>(
            (kf_sensor_pose.translation().x() - origin_x_) / map_resolution_);
        int y0 =
            height - 1 -
            static_cast<int>((kf_sensor_pose.translation().y() - origin_y_) /
                             map_resolution_);
        // Bresenham 空闲更新
        int dx = abs(gx - x0), sx = x0 < gx ? 1 : -1;
        int dy = abs(gy - y0), sy = y0 < gy ? 1 : -1;
        int err = dx - dy, cx = x0, cy = y0;

        while (cx != gx || cy != gy) {
          if (cx >= 0 && cx < width && cy >= 0 && cy < height)
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
        if (gx >= 0 && gx < width && gy >= 0 && gy < height) {
          if (hit_object)
            local_log.at<float>(gy, gx) = std::min(
                std::max(local_log.at<float>(gy, gx) + l_occ, l_min), l_max);
          else
            local_log.at<float>(gy, gx) = std::min(
                std::max(local_log.at<float>(gy, gx) + l_free, l_min), l_max);
        }
      }
    }
    // === 反光柱 (世界坐标) ===
    for (const auto& ref : kf->reflectors) {
      int gx = static_cast<int>((ref.pos.x() - origin_x_) / map_resolution_);
      int gy = height - 1 -
               static_cast<int>((ref.pos.y() - origin_y_) / map_resolution_);
      for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++) {
          int nx = gx + dx, ny = gy + dy;
          if (nx >= 0 && nx < width && ny >= 0 && ny < height)
            local_log.at<float>(ny, nx) = std::min(
                std::max(local_log.at<float>(ny, nx) + l_occ, l_min), l_max);
        }
    }
#pragma omp critical
    occupancy_grid_ += local_log;
  }
  // 转概率图 & 可视化
  cv::Mat display(height, width, CV_8UC1, cv::Scalar(127));
  for (int y = 0; y < height; ++y)
    for (int x = 0; x < width; ++x)
      display.at<uint8_t>(y, x) = static_cast<uint8_t>(
          (1.0f -
           (1.0f - 1.0f / (1.0f + exp(occupancy_grid_.at<float>(y, x))))) *
          255);

  cv::cvtColor(display, occupancy_grid_, cv::COLOR_GRAY2BGR);
  for (auto& kv : frames_to_draw) {
    int px = static_cast<int>((kv->pose.translation().x() - origin_x_) /
                              map_resolution_);
    int py = height - 1 -
             static_cast<int>((kv->pose.translation().y() - origin_y_) /
                              map_resolution_);
    cv::circle(occupancy_grid_, cv::Point(px, py), 2, cv::Scalar(0, 255, 0),
               -1);
  }
  std::vector<Eigen::Vector2d> ref;
  {
    std::lock_guard<std::mutex> lock(ref_mtx_);
    ref = ref_map;
  }
  for (auto& kv2 : ref) {
    Eigen::Vector2d p = kv2;
    int px = static_cast<int>((p(0) - origin_x_) / map_resolution_);
    int py =
        height - 1 - static_cast<int>((p(1) - origin_y_) / map_resolution_);
    cv::circle(occupancy_grid_, cv::Point(px, py), 3, cv::Scalar(0, 0, 255),
               -1);
  }
  out_img = occupancy_grid_;
}
}  // namespace loam