#include "../include/detecter.hpp"
bool extrac(sensor_msgs::msg::LaserScan::ConstSharedPtr msg,
            std::vector<PostPoint> &ret_scan_ps) {
  int max = msg->ranges.size();
  double step = msg->angle_increment;
  int half_window{2};
  std::vector<double> filter_body;
  std::vector<PostPoint> scan_points;
  // 滑动窗口，提取高反光点
  for (int id = 0; id < max; id++) {
    filter_body.clear();
    for (int bias = id - half_window; bias <= id + half_window; bias++) {
      if (bias < 0) {
        filter_body.push_back(*(msg->intensities.begin() + bias + max));
      } else if (bias >= max) {
        filter_body.push_back(*(msg->intensities.begin() + bias - max));
      } else {
        filter_body.push_back(*(msg->intensities.begin() + bias));
      }
    }
    std::sort(filter_body.begin(), filter_body.end());
    if (filter_body[half_window] > 500 && !std::isinf(msg->ranges.at(id))) {
      PostPoint point;
      double theta = msg->angle_min + id * step;
      point.x = cos(theta) * msg->ranges.at(id);
      point.y = sin(theta) * msg->ranges.at(id);
      point.index = id;
      point.f = msg->intensities.at(id);
      point.theta = theta;
      scan_points.push_back(point);
    }
  }
  ret_scan_ps = scan_points;
  return true;
}
double distance_square(PostPoint &a, PostPoint &b) {
  return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}
bool association(std::vector<PostPoint> &input, std::vector<Post> &out) {
  out.clear();
  int sz = input.size();
  DSU dsu(sz);
  for (int i = 0; i < sz - 1; i++) {
    for (int j = i + 1; j < sz; j++) {
      if (distance_square(*(input.begin() + i), *(input.begin() + j)) <
          0.05 * 0.05) {
        dsu.merge(i, j);
      }
    }
  }
  std::unordered_map<int, std::unordered_set<int>> res;  // dsu root -> points
  for (int i = 0; i < sz; i++) {
    res[dsu.find(i)].insert(i);  // root 对应的index 集合
  }
  for (auto it = res.begin(); it != res.end(); it++) {
    Post p;
    p.index = it->first;
    for (auto &x : it->second) {
      p.points.push_back(input[x]);
    }
    if (p.points.size() >= 3 && p.points.size() <= 30) out.push_back(p);
  }
  return true;
}
