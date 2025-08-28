#ifndef MAP_OPTIMIZATION_HPP
#define MAP_OPTIMIZATION_HPP
#include <ceres/ceres.h>
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

struct ObservationResidual {
  ObservationResidual(const Eigen::Vector3d& obs_pose) : obs_pose_(obs_pose) {}
  template <typename T>
  bool operator()(const T* const pose, const T* const ref_pose,
                  T* residual) const {
    Eigen::Matrix<T, 3, 1> p(pose[0], pose[1], pose[2]);
    Eigen::Quaternion<T> q(pose[6], pose[3], pose[4], pose[5]);
    q.normalize();
    Eigen::Matrix<T, 4, 1> p_ref(ref_pose[0], ref_pose[1], ref_pose[2], T(1));
    Eigen::Matrix<T, 4, 4> T_ref_obs = Eigen::Matrix<T, 4, 4>::Identity();
    T_ref_obs.block(0, 0, 3, 3) = q.toRotationMatrix();
    T_ref_obs.block(0, 3, 3, 1) = p;
    Eigen::Matrix<T, 4, 1> p_in_obs = T_ref_obs.inverse() * p_ref;
    residual[0] = T(p_in_obs.x()) - T(obs_pose_.x());
    residual[1] = T(p_in_obs.y()) - T(obs_pose_.y());
    residual[2] = T(p_in_obs.z()) - T(obs_pose_.z());
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& obs_pose) {
    return new ceres::AutoDiffCostFunction<ObservationResidual, 3, 7, 3>(
        new ObservationResidual(obs_pose));
  }
  //
 private:
  const Eigen::Vector3d obs_pose_;
};

struct OdometryResidual {
  OdometryResidual(double dx, double dy, double dz, double qx, double qy,
                   double qz, double qw,
                   const Eigen::Matrix<double, 6, 6>& info)
      : delta_translation_(dx, dy, dz),
        delta_rotation_(qw, qx, qy, qz),  // Eigen四元数是(w, x, y, z)
        information_(info) {}

  // 模板函数：自动求导
  template <typename T>
  bool operator()(const T* const pose1,  // 前一个位姿 [x, y, z, qx, qy, qz, qw]
                  const T* const pose2,  // 后一个位姿
                  T* residual) const {
    // 1. 提取位姿1的参数
    Eigen::Matrix<T, 3, 1> p1;
    p1 << pose1[0], pose1[1], pose1[2];  // 位置

    Eigen::Quaternion<T> q1(pose1[6], pose1[3], pose1[4],
                            pose1[5]);  // 四元数(w, x, y, z)
    q1.normalize();

    // 2. 提取位姿2的参数
    Eigen::Matrix<T, 3, 1> p2;
    p2 << pose2[0], pose2[1], pose2[2];  // 位置

    Eigen::Quaternion<T> q2(pose2[6], pose2[3], pose2[4],
                            pose2[5]);  // 四元数(w, x, y, z)
    q2.normalize();

    // 3. 计算位置残差
    // 位姿2相对于位姿1的平移（在世界坐标系下）
    Eigen::Matrix<T, 3, 1> pos_diff = p2 - p1;

    // 转换到位姿1的坐标系（应用位姿1的旋转）
    Eigen::Matrix<T, 3, 1> pos_in_p1 = q1 * pos_diff;

    // 观测到的相对平移（转换为模板类型）
    Eigen::Matrix<T, 3, 1> measured_translation;
    measured_translation << T(delta_translation_.x()),
        T(delta_translation_.y()), T(delta_translation_.z());

    // 位置残差
    Eigen::Matrix<T, 3, 1> pos_residual = pos_in_p1 - measured_translation;

    // 4. 计算旋转残差
    // 计算相对旋转: q2 = q_rel * q1 => q_rel = q2 * q1^{-1}
    Eigen::Quaternion<T> q_rel = q2 * q1.inverse();

    // 观测到的相对旋转（转换为模板类型）
    Eigen::Quaternion<T> measured_rotation(
        T(delta_rotation_.w()), T(delta_rotation_.x()), T(delta_rotation_.y()),
        T(delta_rotation_.z()));
    measured_rotation.normalize();

    // 旋转误差: 实际相对旋转 * 观测相对旋转的逆
    Eigen::Quaternion<T> q_error = q_rel * measured_rotation.inverse();
    q_error.normalize();

    // 转换为旋转向量残差（使用虚部，已考虑了2倍旋转关系）
    Eigen::Matrix<T, 3, 1> rot_residual = 2.0 * q_error.vec();

    // 5. 组合残差并应用信息矩阵加权
    Eigen::Matrix<T, 6, 1> combined_residual;
    combined_residual << pos_residual, rot_residual;
    // 应用信息矩阵（转换为模板类型的信息矩阵）
    Eigen::Matrix<T, 6, 6> info;
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 6; ++j) info(i, j) = T(information_(i, j));

    // 计算加权残差
    Eigen::Matrix<T, 6, 1> weighted_residual = info * combined_residual;

    // 复制到输出
    for (int i = 0; i < 6; ++i) {
      residual[i] = weighted_residual[i];
    };
    return true;
  }

  static ceres::CostFunction* Create(const double dx, const double dy,
                                     const double dz, const double qx,
                                     const double qy, const double qz,
                                     const double qw,
                                     const Eigen::Matrix<double, 6, 6>& info) {
    return new ceres::AutoDiffCostFunction<OdometryResidual, 6, 7, 7>(
        new OdometryResidual(dx, dy, dz, qx, qy, qz, qw, info));
  }

 private:
  const Eigen::Vector3d delta_translation_;        // 相对位置
  const Eigen::Quaterniond delta_rotation_;        // 相对旋转
  const Eigen::Matrix<double, 6, 6> information_;  // 信息矩阵
};

struct OptimizateStatus {
  std::chrono::steady_clock::time_point last_optimize_time;
  int last_keyframe_num;
  bool first_optimize{true};
};

class MapOptimization {
 public:
  enum Mode { Loc, Slam };
  MapOptimization(rclcpp::Node::SharedPtr node);
  ~MapOptimization() {}
  void laserCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
  void optimize();
  void reset_optimized_map();
  void reset_optimized_reflectors();
  void optimize_thread();
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

  // ceres
  int window_size{10};
  std::condition_variable cv;
  std::thread optimize_thread_;
  std::mutex optimize_mutex;
  OptimizateStatus status;
  Mode mode{Slam};
};
}  // namespace reflector_slam

#endif