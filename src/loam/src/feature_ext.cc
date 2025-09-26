#include "../include/feature_ext.hpp"

#include <nanoflann.hpp>
// g2o
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimizable_graph.h>
#include <g2o/core/optimization_algorithm_factory.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/slam2d/types_slam2d.h>
namespace loam {

Eigen::Vector2d transformPoint(const Eigen::Vector2d& p, const g2o::SE2& pose) {
  double c = cos(pose[2]), s = sin(pose[2]);
  return {c * p.x() - s * p.y() + pose[0], s * p.x() + c * p.y() + pose[1]};
}

struct PointCloud2D {
  std::vector<std::array<double, 2>> pts;
  inline size_t kdtree_get_point_count() const { return pts.size(); }
  inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
    return pts[idx][dim];
  }
  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

// ================= Split-and-Merge 核心 =================
void splitAndMerge(const std::vector<Eigen::Vector2d>& pts, int start, int end,
                   double threshold, std::vector<Line>& lines) {
  if (end <= start + 1) return;

  // 拟合直线：取两端点
  Eigen::Vector2d p1 = pts[start];
  Eigen::Vector2d p2 = pts[end];
  Eigen::Vector2d line = p2 - p1;

  // 找到距离直线最远的点
  double max_dist = -1;
  int idx = -1;
  for (int i = start + 1; i < end; i++) {
    double dist = std::abs((line.y()) * (pts[i].x() - p1.x()) -
                           (line.x()) * (pts[i].y() - p1.y())) /
                  line.norm();
    if (dist > max_dist) {
      max_dist = dist;
      idx = i;
    }
  }

  if (max_dist > threshold) {
    // 分裂
    splitAndMerge(pts, start, idx, threshold, lines);
    splitAndMerge(pts, idx, end, threshold, lines);
  } else {
    // 保留直线段
    Line l;
    l.start = p1;
    l.end = p2;
    l.n_vec = (p2 - p1).normalized();
    l.num_points = end - start + 1;
    lines.push_back(l);
  }
}
// ================= 筛选逻辑 =================
void filterLines(const std::vector<Line>& raw, std::vector<Line>& filtered,
                 int min_points, double min_length, double merge_angle,
                 double merge_dist) {
  std::vector<Line> tmp;

  // 1. 点数 & 长度过滤
  for (auto& l : raw) {
    double dx = l.end.x() - l.start.x();
    double dy = l.end.y() - l.start.y();
    double len = std::sqrt(dx * dx + dy * dy);
    if (l.num_points >= min_points && len >= min_length) {
      tmp.push_back(l);
    }
  }

  // 2. 合并相邻直线（简单实现：顺序两两合并）
  for (size_t i = 0; i < tmp.size(); i++) {
    if (filtered.empty()) {
      filtered.push_back(tmp[i]);
      continue;
    }

    Line& last = filtered.back();
    Eigen::Vector2d v1(last.end.x() - last.start.x(),
                       last.end.y() - last.start.y());
    Eigen::Vector2d v2(tmp[i].end.x() - tmp[i].start.x(),
                       tmp[i].end.y() - tmp[i].start.y());

    double angle = std::acos(v1.normalized().dot(v2.normalized()));
    double dist = std::hypot(tmp[i].start.x() - last.end.x(),
                             tmp[i].start.y() - last.end.y());

    if (angle < merge_angle && dist < merge_dist) {
      // 合并为新直线
      last.end = tmp[i].end;
      last.num_points += tmp[i].num_points;
    } else {
      filtered.push_back(tmp[i]);
    }
  }
}

std::vector<int> extractCorners(const std::vector<Eigen::Vector2d>& pts, int k,
                                double thresh, double max_thresh) {
  std::vector<int> corners;
  int N = pts.size();
  std::vector<double> curvs(pts.size(), 0.0);
  for (int i = k; i < N - k; i++) {
    Eigen::Vector2d mean(0, 0);
    for (int j = -k; j <= k; j++) mean += pts[i + j];
    mean /= (2 * k + 1);

    double curv = 0;
    for (int j = -k; j <= k; j++) curv += (pts[i + j] - mean).squaredNorm();
    curv /= (2 * k + 1);
    curvs[i] = curv;
  }
  int nms_k = 3;
  for (size_t i = k; i < pts.size() - k; i++) {
    if (curvs[i] < thresh || curvs[i] > max_thresh) continue;

    bool is_max = true;
    for (int j = -nms_k; j <= nms_k; j++) {
      if (j == 0) continue;
      if (curvs[i] < curvs[i + j]) {
        is_max = false;
        break;
      }
    }
    if (is_max) corners.push_back(i);
  }
  return corners;
}

// ---------------------------
// g2o vertex that holds circle parameters in general form: [D, E, F]
// Note: we do not optimize scale -- x^2+y^2 is fixed. The algebraic form is:
// x^2 + y^2 + D*x + E*y + F = 0
// ---------------------------
class CircleVertex : public g2o::BaseVertex<3, Eigen::Vector3d> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CircleVertex() { _estimate = Eigen::Vector3d::Zero(); }
  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }

  virtual void setToOriginImpl() { _estimate.setZero(); }
  virtual void oplusImpl(const double* update) {
    _estimate += Eigen::Vector3d(update);
  }
};

class CircleEdge : public g2o::BaseUnaryEdge<1, double, CircleVertex> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  CircleEdge(Eigen::Vector2d p) : _p(p) {}
  virtual bool read(std::istream&) { return false; }
  virtual bool write(std::ostream&) const { return false; }
  virtual void computeError() {
    const CircleVertex* v = static_cast<CircleVertex*>(_vertices[0]);
    const Eigen::Vector3d& est = v->estimate();
    _error[0] = _p.x() * _p.x() + _p.y() * _p.y() + est[0] * _p.x() +
                est[1] * _p.y() + est[2];
  }
  virtual void linearizeOplus() {
    _jacobianOplusXi(0, 0) = _p.x();
    _jacobianOplusXi(0, 1) = _p.y();
    _jacobianOplusXi(0, 2) = 1;
  }

 private:
  Eigen::Vector2d _p;
};

void FeatureExtractor::extract_high_intensity_points(
    const sensor_msgs::msg::LaserScan::SharedPtr& scan) {
  high_intensity_points_.clear();
  for (size_t i = 0; i < scan->ranges.size(); i++) {
    if (scan->intensities[i] < intensity_threshold_) continue;
    double x =
        scan->ranges[i] * cos(scan->angle_min + i * scan->angle_increment);
    double y =
        scan->ranges[i] * sin(scan->angle_min + i * scan->angle_increment);
    if (std::isfinite(x) && std::isfinite(y))
      high_intensity_points_.push_back(Eigen::Vector2d(x, y));
  }
}

bool FeatureExtractor::extract_line(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
  // 1. 转换 LaserScan 到 2D 点
  std::vector<Eigen::Vector2d> points;
  points.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    float r = msg->ranges[i];
    if (std::isfinite(r)) {
      float angle = msg->angle_min + i * msg->angle_increment;
      points.emplace_back(r * std::cos(angle), r * std::sin(angle));
    }
  }
  std::vector<Line> lines;
  splitAndMerge(points, 0, points.size() - 1, 0.07 /*阈值*/, lines);
  // 3. 筛选直线
  std::vector<Line> filtered_lines;
  filterLines(lines, filtered_lines, 20, 0.1 /*min_len*/, 5.0 * M_PI / 180.0,
              0.1);
  line_obs_.clear();
  line_obs_ = filtered_lines;
  // 3. 输出直线数量
  RCLCPP_INFO(rclcpp::get_logger(""), "Extracted %zu lines",
              filtered_lines.size());
  for (auto& l : filtered_lines) {
    RCLCPP_INFO(rclcpp::get_logger(""), "Line: (%.2f, %.2f) -> (%.2f, %.2f)",
                l.start.x(), l.start.y(), l.end.x(), l.end.y());
  }
  return line_obs_.size() > 3;
}

bool FeatureExtractor::extract_corner(
    const sensor_msgs::msg::LaserScan::SharedPtr& msg) {
  std::vector<Eigen::Vector2d> pts;
  pts.reserve(msg->ranges.size());
  for (size_t i = 0; i < msg->ranges.size(); i++) {
    if (!std::isfinite(msg->ranges[i])) continue;
    float angle = msg->angle_min + i * msg->angle_increment;
    pts.emplace_back(msg->ranges[i] * cos(angle), msg->ranges[i] * sin(angle));
  }

  std::vector<int> corner_idx = extractCorners(pts, 5, 0.02, 0.15);
  // RCLCPP_INFO(rclcpp::get_logger(""), "Found %zu corners",
  // corner_idx.size());

  corner_obs_.clear();
  for (auto idx : corner_idx) {
    auto& p = pts[idx];
    CornerObs corner;
    corner.pos = p;
    corner_obs_.push_back(corner);
    // RCLCPP_INFO(rclcpp::get_logger(""), "Corner: %.2f, %.2f", p.x(), p.y());
  }
  return corner_obs_.size() > 5;
}

bool FeatureExtractor::extract_reflector_obs() {
  reflector_obs_.clear();
  PointCloud2D pc2d;
  for (auto& x : high_intensity_points_) pc2d.pts.push_back({x.x(), x.y()});
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, PointCloud2D>, PointCloud2D, 2>
      KDTree;
  KDTree tree(2, pc2d, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  tree.buildIndex();

  size_t N = pc2d.pts.size();
  std::vector<int> labels(N, -1);
  int cid = 0;
  std::vector<size_t> ret_indexes;

  for (size_t i = 0; i < N; ++i) {
    if (labels[i] != -1) continue;
    // radius search
    ret_indexes.clear();
    const double search_radius =
        cluster_radius_ *
        cluster_radius_;  // nanoflann uses squared radius for radiusSearch?
    // nanoflann radius search API uses squared radius in search!
    std::vector<nanoflann::ResultItem<uint32_t, double>> out;
    nanoflann::SearchParameters params;
    tree.radiusSearch(&pc2d.pts[i][0], search_radius, out, params);
    if ((int)out.size() < min_num_points_) {
      labels[i] = -2;  // noise
      continue;
    }
    // BFS expand
    std::vector<size_t> seeds;
    for (auto& it : out) seeds.push_back(it.first);
    labels[i] = cid;
    for (size_t sidx = 0; sidx < seeds.size(); ++sidx) {
      size_t idx = seeds[sidx];
      if (labels[idx] == -2) {
        labels[idx] = cid;
      }
      if (labels[idx] != -1) continue;
      labels[idx] = cid;
      // neighbors of idx
      std::vector<nanoflann::ResultItem<uint32_t, double>> out2;
      tree.radiusSearch(&pc2d.pts[idx][0], search_radius, out2, params);
      if (out2.size() >= (size_t)min_num_points_) {
        for (auto& it2 : out2) seeds.push_back(it2.first);
      }
    }
    cid++;
  }
  std::unordered_map<int, std::vector<Eigen::Vector2d>> clusters;
  for (size_t i = 0; i < N; ++i) {
    if (labels[i] >= 0)
      clusters[labels[i]].push_back(
          Eigen::Vector2d(pc2d.pts[i][0], pc2d.pts[i][1]));
  }
  // RCLCPP_INFO(n_->get_logger(), "- - - - - - - - - ");
  for (auto& kv : clusters) {
    auto& pts = kv.second;
    // build a ReflectorObs with initial pos = centroid
    Eigen::Vector3d xy_radius;
    fit_circle_g2o(pts, xy_radius);
    ReflectorObs ro;
    ro.pos.x() = xy_radius[0];
    ro.pos.y() = xy_radius[1];
    ro.id = -1;  // assigned later
    if (xy_radius[2] > max_radius_ || xy_radius[2] < min_radius_) continue;
    reflector_obs_.push_back(ro);
  }
  // RCLCPP_INFO(n_->get_logger(), "Extracted %ld reflectors",
  //             reflector_obs_.size());
  return reflector_obs_.size() > 2;
}

void FeatureExtractor::fit_circle_g2o(std::vector<Eigen::Vector2d>& ps,
                                      Eigen::Vector3d& xy_radius) {
  if (ps.size() < 3) return;
  using BlockSolverType =
      g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;  // D,E,F (3) x residual
                                                       // dim 1
  using LinearSolverType =
      g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

  auto linearSolver = std::make_unique<LinearSolverType>();
  auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
  g2o::OptimizationAlgorithmLevenberg* solver =
      new g2o::OptimizationAlgorithmLevenberg(std::move(blockSolver));
  g2o::SparseOptimizer optimizer;
  optimizer.setAlgorithm(solver);
  // add circle vertex
  CircleVertex* vc = new CircleVertex();
  // initial guess: algebraic from centroid method: D = -2*cx, E = -2*cy, F =
  // cx^2+cy^2 - r0^2 use radius estimate as mean distance to centroid
  Eigen::Vector2d centroid(0, 0);
  for (auto& p : ps) centroid += p;
  centroid /= ps.size();
  double r0 = 0.0;
  for (auto& p : ps) r0 += (p - centroid).norm();
  r0 /= ps.size();
  Eigen::Vector3d init;
  init[0] = -2.0 * centroid.x();
  init[1] = -2.0 * centroid.y();
  init[2] = centroid.squaredNorm() - r0 * r0;
  vc->setEstimate(init);
  vc->setId(0);
  optimizer.addVertex(vc);

  // add edges for each point
  for (size_t pi = 0; pi < ps.size(); ++pi) {
    CircleEdge* e = new CircleEdge(ps[pi]);
    e->setVertex(0, vc);
    // e->setPoint(pts[pi]);
    // measurement is unused (we keep residual as algebraic), but g2o requires a
    // measurement; set to 0
    e->setMeasurement(0.0);
    Eigen::Matrix<double, 1, 1> info;
    info(0, 0) = 1.0;
    e->setInformation(info);
    // robust kernel
    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
    rk->setDelta(1.0);
    e->setRobustKernel(rk);
    optimizer.addEdge(e);
  }

  optimizer.initializeOptimization();
  optimizer.setVerbose(false);
  optimizer.optimize(20);
  Eigen::Vector3d sol = vc->estimate();
  // convert to center/radius
  Eigen::Vector2d center(-sol[0] / 2.0, -sol[1] / 2.0);
  double rad_sq = (sol[0] * sol[0] + sol[1] * sol[1]) / 4.0 - sol[2];
  double radius = (rad_sq > 0) ? std::sqrt(rad_sq) : 0.0;
  xy_radius = Eigen::Vector3d(center.x(), center.y(), radius);
}

}  // namespace loam