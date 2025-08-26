#include <cmath>
#include <unordered_map>
#include <vector>

#include "../include/fetaure_extractor.hpp"

class DSU {
 private:
  std::vector<size_t> parent;
  std::vector<size_t> rank;

 public:
  DSU(size_t n) : parent(n), rank(n, 0) {
    for (size_t i = 0; i < n; ++i) {
      parent[i] = i;
    }
  }

  size_t find(size_t x) {
    if (parent[x] != x) {
      parent[x] = find(parent[x]);  // Path compression
    }
    return parent[x];
  }

  void unite(size_t x, size_t y) {
    size_t x_root = find(x);
    size_t y_root = find(y);
    if (x_root == y_root) return;

    // Union by rank
    if (rank[x_root] < rank[y_root]) {
      parent[x_root] = y_root;
    } else if (rank[x_root] > rank[y_root]) {
      parent[y_root] = x_root;
    } else {
      parent[y_root] = x_root;
      rank[x_root]++;
    }
  }
};
namespace reflector_slam {

std::vector<Observation> FeatureExtractor::extract(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
  std::vector<Observation> observations;
  std::vector<Eigen::Vector3d> candidate_points;

  // Step 1: Extract candidate points with high intensity
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    if (scan->intensities[i] > 0.8) {  // Threshold for reflectors
      Eigen::Vector3d point;
      point.x() = scan->ranges[i] *
                  std::cos(scan->angle_min + i * scan->angle_increment);
      point.y() = scan->ranges[i] *
                  std::sin(scan->angle_min + i * scan->angle_increment);
      candidate_points.push_back(point);
    }
  }

  // Step 2: Cluster candidate points using DSU
  if (!candidate_points.empty()) {
    DSU dsu(candidate_points.size());

    // Connect points within threshold distance
    for (size_t i = 0; i < candidate_points.size(); ++i) {
      for (size_t j = i + 1; j < candidate_points.size(); ++j) {
        double dx = candidate_points[i].x() - candidate_points[j].x();
        double dy = candidate_points[i].y() - candidate_points[j].y();
        double distance = std::sqrt(dx * dx + dy * dy);
        if (distance < cluster_threshold_) {
          dsu.unite(i, j);
        }
      }
    }

    // Group points by their root parent
    std::unordered_map<size_t, std::vector<Eigen::Vector3d>> points;

    for (size_t i = 0; i < candidate_points.size(); ++i) {
      size_t root = dsu.find(i);
      points[root].push_back({candidate_points[i].x(), candidate_points[i].y(),
                              candidate_points[i].z()});
    }

    // Fit circle for each cluster
    for (auto& [id, obs] : points) {
      if (points[id].size() < 5) continue;  // Skip small clusters

      ceres::Problem problem;
      // Initial guess: use the first point as center, radius = 0.035
      const auto& first_point = points[id][0];
      double a = first_point.x();
      double b = first_point.y();
      double r = 0.035;
      // Calculate D, E, F from center (a, b) and radius r
      double D = -2 * a;
      double E = -2 * b;
      double F = a * a + b * b - r * r;
      double DEF[3] = {D, E, F};

      // Add residuals for each point
      for (const auto& p : points[id]) {
        ceres::CostFunction* cost_function =
            new CircleCostFunction(p.x(), p.y());
        problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.5), DEF);
      }

      // Solve the problem
      ceres::Solver::Options options;
      options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
      options.minimizer_progress_to_stdout = false;
      ceres::Solver::Summary summary;
      ceres::Solve(options, &problem, &summary);

      // Convert general equation to center-radius form
      D = DEF[0];
      E = DEF[1];
      F = DEF[2];
      auto clusters = Observation();
      clusters.point.x() = -D / 2;
      clusters.point.y() = -E / 2;
      clusters.point.z() = 0;
      clusters.id = -1;
      clusters.confidence = 0;
      auto radius = std::sqrt(D * D / 4 + E * E / 4 - F);
      if (radius > min_radius_ && radius < max_radius_) {
        observations.push_back(clusters);
      }
    }
  }
  return observations;
}

float FeatureExtractor::match(std::vector<Observation>& reflectors,
                              const std::unordered_map<int, Reflector>& map,
                              const Eigen::Matrix4d& odom_pose) {
  if (reflectors.empty() || map.empty()) {
    return 0;
  }
  float sum_confidence = 0;
  int match_count = 0;
  std::unordered_map<int, int> map_matched;
  for (size_t i = 0; i < reflectors.size(); ++i) {
    if (reflectors[i].id != -1) continue;
    Eigen::Vector4d cur_pose = odom_pose * reflectors[i].point.homogeneous();
    double min_distance = max_distance_;
    int best_match = -1;

    for (auto& [id, reflector] : map) {
      if (map_matched.find(id) != map_matched.end()) {
        continue;
      }
      Eigen::Vector4d dist = cur_pose - reflector.position.homogeneous();
      double distance = dist.head<2>().norm();
      if (distance < min_distance) {
        min_distance = distance;
        best_match = id;
      }
    }
    if (best_match != -1) {
      reflectors[i].id = best_match;
      map_matched[best_match] = 1;
      reflectors[i].confidence = 1 - min_distance / max_distance_;
      sum_confidence += reflectors[i].confidence;
      ++match_count;
    }
  }
  if (match_count < 3) {
    return -1;
  }
  return sum_confidence / match_count;
}
Eigen::Matrix<double, 4, 4> FeatureExtractor::pre_pose_estimation(
    const std::vector<Observation>& reflectors,
    std::unordered_map<int, Reflector>& map) {
  std::vector<Eigen::Vector3d> cur_points;
  std::vector<Eigen::Vector3d> map_points;
  for (const auto& reflector : reflectors) {
    if (reflector.id != -1) {
      cur_points.push_back(reflector.point);
      map_points.push_back(map[reflector.id].position);
    }
  }
  if (cur_points.size() < 3) {
    throw std::runtime_error("Not enough points for pose estimation");
  }
  // 1. 计算质心
  Eigen::Vector3d centroid_src(0, 0, 0);
  Eigen::Vector3d centroid_dst(0, 0, 0);
  int n = cur_points.size();

  for (int i = 0; i < n; ++i) {
    centroid_src += cur_points[i];
    centroid_dst += map_points[i];
  }

  centroid_src /= n;
  centroid_dst /= n;

  // 2. 计算去质心点集和协方差矩阵
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();
  for (int i = 0; i < n; ++i) {
    Eigen::Vector3d src_centered = cur_points[i] - centroid_src;
    Eigen::Vector3d dst_centered = map_points[i] - centroid_dst;
    H += src_centered * dst_centered.transpose();
  }

  // 3. SVD分解
  Eigen::JacobiSVD<Eigen::Matrix3d> svd(
      H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix3d U = svd.matrixU();
  Eigen::Matrix3d V = svd.matrixV();

  // 4. 计算旋转矩阵
  Eigen::Matrix3d R = V * U.transpose();

  // 处理可能的反射 (确保旋转矩阵行列式为1)
  if (R.determinant() < 0) {
    Eigen::Matrix3d diag = Eigen::Matrix3d::Identity();
    diag(2, 2) = -1;
    R = V * diag * U.transpose();
  }

  // 5. 计算平移向量
  Eigen::Vector3d t = centroid_dst - R * centroid_src;

  // 6. 构造4x4变换矩阵
  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
  transform.block<3, 3>(0, 0) = R;
  transform.block<3, 1>(0, 3) = t;

  return transform;
}
}  // namespace reflector_slam
