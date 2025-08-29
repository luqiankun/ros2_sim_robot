#ifndef REFLECTOR_EXTRACTOR_HPP
#define REFLECTOR_EXTRACTOR_HPP
#include <ceres/ceres.h>

#include <vector>

#include "common.hpp"
namespace reflector_slam {
struct MatchResidual {
  MatchResidual(const Eigen::Vector3d& ref_pose,
                const Eigen::Vector3d& map_pose)
      : ref_pose(ref_pose), map_pose(map_pose) {}
  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    // x y z qx qy qz qw
    Eigen::Quaternion<T> quat(pose[6], pose[3], pose[4], pose[5]);
    Eigen::Matrix<T, 3, 1> t(pose[0], pose[1], pose[2]);
    Eigen::Matrix<T, 3, 1> ref_pose_t(T(ref_pose.x()), T(ref_pose.y()),
                                      T(ref_pose.z()));
    Eigen::Matrix<T, 3, 1> map_pose_t(T(map_pose.x()), T(map_pose.y()),
                                      T(map_pose.z()));
    Eigen::Matrix<T, 3, 1> ref_pose_t_transformed = quat * ref_pose_t + t;
    residual[0] = ref_pose_t_transformed.x() - T(map_pose_t.x());
    residual[1] = ref_pose_t_transformed.y() - T(map_pose_t.y());
    residual[2] = ref_pose_t_transformed.z() - T(map_pose_t.z());
    return true;
  }
  static ceres::CostFunction* Create(const Eigen::Vector3d& ref_pose,
                                     const Eigen::Vector3d& map_pose) {
    return new ceres::AutoDiffCostFunction<MatchResidual, 3, 7>(
        new MatchResidual(ref_pose, map_pose));
  }

 private:
  Eigen::Vector3d ref_pose;
  Eigen::Vector3d map_pose;
};
struct UnmatchedResidual {
  UnmatchedResidual(double weight) : weight_(weight) {}

  template <typename T>
  bool operator()(const T* const pose, T* residual) const {
    (void)pose;
    // 惩罚项：固定权重（不依赖位姿，仅用于约束未匹配点数量）
    residual[0] = T(weight_);
    return true;
  }

  static ceres::CostFunction* Create(double weight) {
    return new ceres::AutoDiffCostFunction<UnmatchedResidual, 1, 7>(
        new UnmatchedResidual(weight));
  }

 private:
  double weight_;  // 惩罚权重（根据传感器特性设置）
};

class Hungarian {
 public:
  Hungarian(const std::vector<std::vector<double>>& cost) : cost_(cost) {
    n_ = cost.size();
    m_ = cost.empty() ? 0 : cost[0].size();
    u.assign(n_ + 1, 0);
    v.assign(m_ + 1, 0);
    p.assign(m_ + 1, 0);
    way.assign(m_ + 1, 0);
  }

  std::vector<int> Solve() {
    if (n_ == 0 || m_ == 0) return {};
    for (int i = 1; i <= n_; ++i) {
      p[0] = i;
      int j0 = 0;
      std::vector<double> minv(m_ + 1, std::numeric_limits<double>::infinity());
      std::vector<char> used(m_ + 1, false);
      do {
        used[j0] = true;
        int i0 = p[j0], j1 = 0;
        double delta = std::numeric_limits<double>::infinity();
        for (int j = 1; j <= m_; ++j) {
          if (used[j]) continue;
          double cur = cost_[i0 - 1][j - 1] - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
        for (int j = 0; j <= m_; ++j) {
          if (used[j]) {
            u[p[j]] += delta;
            v[j] -= delta;
          } else {
            minv[j] -= delta;
          }
        }
        j0 = j1;
      } while (p[j0] != 0);
      do {
        int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0);
    }
    std::vector<int> ans(n_, -1);
    for (int j = 1; j <= m_; ++j) {
      if (p[j] > 0) ans[p[j] - 1] = j - 1;
    }
    return ans;
  }

 private:
  int n_, m_;
  std::vector<std::vector<double>> cost_;
  std::vector<double> u, v;
  std::vector<int> p, way;
};

class FeatureExtractor {
 public:
  FeatureExtractor() = default;
  FeatureExtractor(double radius, double cluster_threshold, double max_distance,
                   double min_radius, double max_radius,
                   double identify_threshold, int max_iteration);
  ~FeatureExtractor() = default;
  class CircleCostFunction : public ceres::SizedCostFunction<1, 3> {
   public:
    CircleCostFunction(double x, double y) : x_(x), y_(y) {}

    virtual bool Evaluate(double const* const* parameters, double* residuals,
                          double** jacobians) const {
      const double D = parameters[0][0];
      const double E = parameters[0][1];
      const double F = parameters[0][2];

      // Residual: x² + y² + Dx + Ey + F
      residuals[0] = x_ * x_ + y_ * y_ + D * x_ + E * y_ + F;

      if (jacobians != nullptr && jacobians[0] != nullptr) {
        // Jacobian: [d(residual)/dD, d(residual)/dE, d(residual)/dF]
        jacobians[0][0] = x_;   // d(residual)/dD
        jacobians[0][1] = y_;   // d(residual)/dE
        jacobians[0][2] = 1.0;  // d(residual)/dF
      }

      return true;
    }

   private:
    double x_, y_;
  };

  std::vector<Observation> extract(
      const sensor_msgs::msg::LaserScan::SharedPtr& scan);
  Eigen::Matrix<double, 4, 4> match(std::vector<Observation>& reflectors,
                                    std::unordered_map<int, Reflector>& map,
                                    const Eigen::Matrix4d& odom_pose);
  Eigen::Matrix<double, 4, 4> pre_pose_estimation(
      const std::vector<Observation>& reflectors,
      std::unordered_map<int, Reflector>& map);

 private:
  // 参数
  double radius = 0.035;
  double min_radius_ = 0.03;        // 反光板最小半径
  double max_radius_ = 0.04;        // 反光板最大半径
  double cluster_threshold_ = 0.1;  // 聚类阈值
  double max_distance_ = 0.3;
  int max_iterations_ = 10;
  double identify_threshold_ = 0.8;
};
}  // namespace reflector_slam
#endif