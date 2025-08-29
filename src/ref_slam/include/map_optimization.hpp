#ifndef MAP_OPTIMIZATION_HPP
#define MAP_OPTIMIZATION_HPP
#include <tf2_ros/transform_broadcaster.h>

#include <Eigen/Eigen>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "./fetaure_extractor.hpp"
#include "./map_manager.hpp"
namespace reflector_slam {

struct OptimizateStatus {
  std::chrono::steady_clock::time_point last_optimize_time;
  int last_keyframe_num;
  bool first_optimize{true};
};

class MapOptimization {
 public:
  enum Mode { Loc, Slam };
  MapOptimization(rclcpp::Node::SharedPtr node);
  ~MapOptimization() {
    map_manager_->generate_from_keyframe(map, keyframes, optimized_map_,
                                         optimized_reflectors_);
    map_manager_->save_map();
    RCLCPP_INFO(node_->get_logger(), "Map saved.");
  }
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void optimize();
  void reset_optimized_map();
  void reset_optimized_reflectors();
  void optimize_thread();
  std::vector<int> loop_close(const Keyframe& key);
  Eigen::Matrix4d math_keyframe(const Keyframe& key_last,
                                const Keyframe& key_cur, int& num);
  bool save_map(const std_srvs::srv::Empty::Request::SharedPtr& request,
                const std_srvs::srv::Empty::Response::SharedPtr response);
  visualization_msgs::msg::MarkerArray getMarkers(
      const std::vector<Observation>& refs);

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      marker_pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr optimize_and_save_service_;
  std::shared_ptr<MapManager> map_manager_;
  std::unordered_map<int, Reflector> map;
  std::unordered_map<int, Keyframe> keyframes;
  // 优化结果
  std::unordered_map<int, Eigen::Matrix4d>
      optimized_map_;  // keyframe id ->优化后的位姿
  std::unordered_map<int, Eigen::Vector3d>
      optimized_reflectors_;  // 反光板id -> 优化后的位姿
  std::deque<Odometry> odoms;
  std::mutex key_mutex;
  std::mutex ref_mutex;
  Eigen::Matrix4d odom_pose{Eigen::Matrix4d::Identity()};
  std::shared_ptr<FeatureExtractor> reflector_extractor_;
  visualization_msgs::msg::MarkerArray cur_markers;
  //
  Eigen::Matrix4d cur_pose_{Eigen::Matrix4d::Identity()};
  double duplicates_threshold_{0.3};
  double keyframe_distance_{0.5};
  double keyframe_angle_{10};
  int ref_share_count_{3};
  // map sava
  std::thread map_thread_;
  std::mutex map_save_mutex;
  std::condition_variable map_cv;
  // ceres
  int window_size{10};
  std::condition_variable cv;
  std::thread optimize_thread_;
  std::mutex optimize_mutex;
  OptimizateStatus status;
  Mode mode{Slam};
  std::atomic<int> last_opt_size_{0};
};
}  // namespace reflector_slam

#endif