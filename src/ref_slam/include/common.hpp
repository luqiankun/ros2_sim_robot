#ifndef COMMON_H
#define COMMON_H
#include <ceres/ceres.h>

#include <Eigen/Dense>
#include <chrono>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

namespace reflector_slam {

// 反光板观测（传感器坐标系）
struct ObservationReflector {
  Eigen::Vector3d point;  // 3D坐标 雷达坐标系下的
  int id;                 // 匹配到的地图ID（-1表示未匹配）
  double confidence;      // 观测置信度（0~1）
  std::chrono::steady_clock::time_point timestamp;
};

struct ObservationCorner : public ObservationReflector {};

// 地图中的反光板（世界坐标系）
struct Reflector {
  Eigen::Vector3d position;  // 位置
  int id;                    // 唯一ID
};
struct Corner : public Reflector {};
// 关键帧
struct Keyframe {
  int id;                // 关键帧ID
  Eigen::Matrix4d pose;  // 传感器位姿（T_world_sensor）
  std::chrono::steady_clock::time_point timestamp;     // 时间戳
  std::vector<ObservationReflector> observation_refs;  // 观测数据
  std::vector<ObservationCorner> observation_corners;  // 观测数据
  sensor_msgs::msg::LaserScan::SharedPtr scan;         // 激光数据
};

struct Odometry {
  int last_id{-1};
  int cur_id{-1};
  Eigen::Matrix4d last_pose;
  Eigen::Matrix4d current_pose;
  double info[6][6];

};  // 里程计数据两个帧之间的位姿变换
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
}  // namespace reflector_slam

#endif  // COMMON_H