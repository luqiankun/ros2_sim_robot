#ifndef MAP_OPTIMIZATION_HPP
#define MAP_OPTIMIZATION_HPP
#include <tf2_ros/transform_broadcaster.h>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "./fetaure_extractor.hpp"
namespace reflector_slam {

class MapOptimization {
 public:
  MapOptimization(rclcpp::Node::SharedPtr node);

  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void optimize();
  visualization_msgs::msg::MarkerArray getMarkers(
      const std::vector<Observation>& refs);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  std::vector<Reflector> map;
  std::deque<Keyframe> keyframes;
  Eigen::Matrix4d odom_pose{Eigen::Matrix4d::Identity()};
  std::shared_ptr<FeatureExtractor> reflector_extractor_;
  visualization_msgs::msg::MarkerArray cur_markers;
  //
  Eigen::Matrix4d cur_pose_{Eigen::Matrix4d::Identity()};
};
}  // namespace reflector_slam

#endif