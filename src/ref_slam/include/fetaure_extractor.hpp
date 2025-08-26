#ifndef REFLECTOR_EXTRACTOR_HPP
#define REFLECTOR_EXTRACTOR_HPP
#include <ceres/ceres.h>

#include <sensor_msgs/msg/laser_scan.hpp>
#include <vector>

#include "common.hpp"
namespace reflector_slam {

class FeatureExtractor {
 public:
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
  float match(std::vector<Observation>& reflectors,
              const std::unordered_map<int, Reflector>& map,
              const Eigen::Matrix4d& odom_pose);
  Eigen::Matrix<double, 4, 4> pre_pose_estimation(
      const std::vector<Observation>& reflectors,
      std::unordered_map<int, Reflector>& map);

 private:
  // 参数
  double min_radius_ = 0.03;        // 反光板最小半径
  double max_radius_ = 0.04;        // 反光板最大半径
  double cluster_threshold_ = 0.1;  // 聚类阈值
  double max_distance_ = 0.3;
};
}  // namespace reflector_slam
#endif