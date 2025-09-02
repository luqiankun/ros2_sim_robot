#include <cmath>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
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
FeatureExtractor::FeatureExtractor(double radius, double cluster_threshold,
                                   double max_distance, double min_radius,
                                   double max_radius, double identify_threshold,
                                   int max_iteration)
    : radius(radius),
      min_radius_(min_radius),
      max_radius_(max_radius),
      cluster_threshold_(cluster_threshold),
      max_distance_(max_distance),
      max_iterations_(max_iteration),
      identify_threshold_(identify_threshold) {}
std::vector<Observation> FeatureExtractor::extract(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
  std::vector<Observation> observations;
  std::vector<Eigen::Vector3d> candidate_points;

  // Step 1: Extract candidate points with high intensity
  for (size_t i = 0; i < scan->ranges.size(); ++i) {
    if (scan->intensities[i] >
        identify_threshold_) {  // Threshold for reflectors
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
      double r = radius;
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
      clusters.timestamp = std::chrono::steady_clock::now();
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

Eigen::Matrix4d FeatureExtractor::match(std::vector<Observation>& reflectors,
                                        std::unordered_map<int, Reflector>& map,
                                        const Eigen::Matrix4d& odom_pose) {
  if (reflectors.empty() || map.empty()) {
    return Eigen::Matrix4d::Zero();
  }

  double pose[7]{odom_pose(0, 3), odom_pose(1, 3), odom_pose(2, 3), 0, 0, 0, 1};
  Eigen::Quaterniond qua(odom_pose.block<3, 3>(0, 0));
  pose[3] = qua.x();
  pose[4] = qua.y();
  pose[5] = qua.z();
  pose[6] = qua.w();
  double err = -1;
  ceres::Problem final_problem;  // 最后一轮 problem
  ceres::Solver::Summary final_summary;
  int matched_count_last = 0;
  double sum_distances_last = 0.0;
  for (int i = 0; i < max_iterations_; ++i) {
    ceres::Problem problem;
    Eigen::Matrix4d opt_pose = Eigen::Matrix4d::Identity();
    opt_pose.block(0, 0, 3, 3) =
        Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5])
            .toRotationMatrix();
    opt_pose.block(0, 3, 3, 1) = Eigen::Vector3d(pose[0], pose[1], pose[2]);

    std::vector<int> map_ids;
    map_ids.reserve(map.size());
    for (auto& [id, ref] : map) map_ids.push_back(id);

    std::vector<std::vector<double>> cost(
        reflectors.size(), std::vector<double>(map_ids.size(), 1e6));

    for (size_t i = 0; i < reflectors.size(); ++i) {
      Eigen::Vector4d cur_pose = opt_pose * reflectors[i].point.homogeneous();
      for (size_t j = 0; j < map_ids.size(); ++j) {
        Eigen::Vector4d diff =
            cur_pose - map[map_ids[j]].position.homogeneous();
        double distance = diff.head<2>().norm();
        cost[i][j] = distance;
      }
    }
    // === Step 2: 匈牙利算法全局匹配 ===
    // auto st = std::chrono::steady_clock::now();
    Hungarian hungarian(cost);
    auto assignment = hungarian.Solve();
    // auto ed = std::chrono::steady_clock::now();
    // auto dur =
    //     std::chrono::duration_cast<std::chrono::milliseconds>(ed -
    //     st).count();
    // std::cerr << dur << " ms\n";
    std::unordered_map<int, int> map_matched;
    int matched_count = 0;
    double sum_distances = 0.0;
    // === Step 3: 根据匹配结果添加残差 ===
    for (size_t i = 0; i < reflectors.size(); ++i) {
      int j = assignment[i];
      if (j == -1) continue;
      double distance = cost[i][j];
      if (distance < max_distance_) {
        int best_match = map_ids[j];
        reflectors[i].id = best_match;
        reflectors[i].confidence = 1 - distance / max_distance_;
        matched_count++;
        sum_distances += distance;
        map_matched[best_match] = 1;
        auto cost_func = MatchResidual::Create(reflectors[i].point,
                                               map[best_match].position);
        problem.AddResidualBlock(cost_func, new ceres::HuberLoss(0.5), pose);
      }
    }

    // 未匹配的点（可选，弱约束）
    // for (auto& [id, ref] : map) {
    //   if (map_matched.find(id) == map_matched.end()) {
    //     auto cost_func = UnmatchedResidual::Create(0.1);
    //     problem.AddResidualBlock(cost_func, nullptr, pose);
    //   }
    // }

    // Solve the problem
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.max_num_iterations = 50;
    options.minimizer_progress_to_stdout = false;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    // std::cout << summary.BriefReport() << std::endl;
    if (fabs(err - summary.final_cost) < 1e-4) {
      err = summary.final_cost;
      final_problem = std::move(problem);
      final_summary = std::move(summary);
      matched_count_last = matched_count;
      sum_distances_last = sum_distances;
      break;
    }
    err = summary.final_cost;
    if (summary.final_cost < 1e-4) {
      final_problem = std::move(problem);
      final_summary = std::move(summary);
      matched_count_last = matched_count;
      sum_distances_last = sum_distances;
      break;
    }
    if (i == max_iterations_ - 1) {
      final_problem = std::move(problem);
      final_summary = summary;
      matched_count_last = matched_count;
      sum_distances_last = sum_distances;
    }
  }
  Eigen::Matrix4d result = Eigen::Matrix4d::Identity();
  result.block(0, 0, 3, 3) =
      Eigen::Quaterniond(pose[6], pose[3], pose[4], pose[5]).toRotationMatrix();
  result.block(0, 3, 3, 1) = Eigen::Vector3d(pose[0], pose[1], pose[2]);
  // std::cout << "final cost: " << err << std::endl;

  if (matched_count_last <= 0) {
    // std::cerr << "No matched reflectors" << std::endl;
    return Eigen::Matrix4d::Zero();
  }
  double mean_residual =
      matched_count_last > 0 ? sum_distances_last / matched_count_last : 1e6;
  double norm_residual = std::exp(-mean_residual / (max_distance_ + 1e-6));
  // std::cout << "pose_confidence = " << pose_confidence << std::endl;
  RCLCPP_INFO(rclcpp::get_logger("ref_slam_node"), "pose_confidence = %f",
              norm_residual);
  return result;
}
}  // namespace reflector_slam
