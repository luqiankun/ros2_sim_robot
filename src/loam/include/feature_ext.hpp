#ifndef FEATURE_EXT_HPP
#define FEATURE_EXT_HPP
#include <g2o/types/slam2d/types_slam2d.h>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "./common.hpp"
namespace loam {
class FeatureExtractor {
 public:
  FeatureExtractor(rclcpp::Node::SharedPtr n) : n_(n) {}
  ~FeatureExtractor() = default;
  void extract_high_intensity_points(
      const sensor_msgs::msg::LaserScan::SharedPtr& scan);
  bool extract_reflector_obs();
  void fit_circle_g2o(std::vector<Eigen::Vector2d>&, Eigen::Vector3d&);
  bool extract_line(const sensor_msgs::msg::LaserScan::SharedPtr& scan);
  bool extract_corner();

 public:
  rclcpp::Node::SharedPtr n_;
  double intensity_threshold_{0.8};
  double cluster_radius_{0.02};
  double max_radius_{0.07};
  double min_radius_{0.01};
  int min_num_points_{3};
  double last_scan_time_ = 0.0;
  double curv_threshold_{0.05};
  double line_length_threshold_;
  RefObsArray reflector_obs_;
  std::vector<CornerObs> corner_obs_;
  std::vector<LineObs> line_obs_;
  std::vector<Eigen::Vector2d> high_intensity_points_;
  std::vector<Eigen::Vector2d> pts;
};

}  // namespace loam

#endif