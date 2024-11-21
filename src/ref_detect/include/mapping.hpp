#ifndef MAPPING_HPP
#define MAPPING_HPP
#include <cstdint>
#include <cstdlib>
#include <vector>

#include "detecter.hpp"
struct Feature2DAdjacency {
  int32_t ID1;
  int32_t ID2;
};
struct AdjacencyList {
  Feature2DList feature_list;
  std::vector<Feature2DAdjacency> adjacency_list;
};
class Mapping {
 public:
  Mapping(rclcpp::Node::SharedPtr n) : node(n) {
    publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/feature_graph", 1);
    timer = node->create_wall_timer(std::chrono::milliseconds(10000), [&] {
      if (feature_graph.size() > 0) {
        int num = 0;
        visualization_msgs::msg::MarkerArray markers;
        for (auto &graph : feature_graph) {
          float r, g, b;
          r = static_cast<float>((rand() % (255))) / 255;
          g = static_cast<float>((rand() % (255))) / 255;
          b = static_cast<float>((rand() % (255))) / 255;
          for (auto &x : graph.second) {
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            // marker.header.stamp = time_now->now();
            marker.lifetime.sec = 10;
            marker.lifetime.nanosec = 110000000;
            marker.color.a = 0.7;
            marker.color.r = r;
            marker.color.g = g;
            marker.color.b = b;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.id = num++;
            geometry_msgs::msg::Point st;
            geometry_msgs::msg::Point ed;
            st.x = feature_points[graph.first].first;
            st.y = feature_points[graph.first].second;
            ed.x = feature_points[x.first].first;
            ed.y = feature_points[x.first].second;
            marker.points.push_back(st);
            marker.points.push_back(ed);
            marker.scale.x = 0.01;
            marker.scale.y = 0.01;
            markers.markers.push_back(marker);
          }
        }
        publisher->publish(markers);
      }
    });
  }
  bool read_map(const std::string &path);

 public:
  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
  rclcpp::Clock::SharedPtr time_now;  // NOLINTL
  //   AdjacencyList current_graph;
  int feature_point_cnt;
  std::unordered_map<int, std::pair<double, double> >
      feature_points;  // 索引 坐标
  std::unordered_map<int, std::unordered_map<int, double> >
      feature_graph;  // 索引 索引 距离
};

inline bool Mapping::read_map(const std::string &path) {
  std::fstream file;
  file.open(path, std::iostream::in);
  if (!file.is_open()) {
    return false;
  }
  std::string line;
  while (std::getline(file, line)) {
    if (!line.empty()) {
      std::vector<std::string> res;
      boost::split(res, line, boost::is_any_of("|"));
      assert(res.size() == 3);
      feature_points[std::stoi(res.at(0))] =
          std::make_pair(std::stof(res.at(1)), std::stof(res.at(2)));
    }
    line.clear();
  }
  file.close();
  assert(feature_points.size() > 0);
  feature_graph.clear();
  std::vector<std::pair<double, double> > temp_input;
  std::vector<int> ids;
  for (auto fpt : feature_points) {
    temp_input.push_back(fpt.second);
    ids.push_back(fpt.first);
  }
  InsertToFeatureGraph(temp_input, ids, feature_points, feature_graph);
  return true;
}
#endif