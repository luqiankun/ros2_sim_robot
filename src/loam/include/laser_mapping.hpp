#ifndef LASER_MAPPING_HPP
#define LASER_MAPPING_HPP
#include <nanoflann.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "./feature_ext.hpp"
#include "./imu_integrate.hpp"
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

class Solver {
 public:
  RefMatchRes associate_ref(RefObsArray&, const RefMap&,
                            const g2o::SE2& pose = {0, 0, 0});
  CornerMatchRes associate_corner(std::vector<CornerObs>&, const CornerMap&,
                                  const g2o::SE2& pose = {0, 0, 0});
  Eigen::Isometry2d compute_svd_transform(const RefMatchRes&);
  g2o::SE2 front_icp(RefObsArray&, const RefMap&, CornerObsArray&,
                     const CornerMap&, const Eigen::Isometry2d& init,
                     Eigen::Matrix3d& covariance);

 private:
  double flann_threshold{0.3};
};

class GlobalReflectorMap {
 public:
  // 查找已有顶点（通过大致位置 hash），若不存在则在 optimizer 中新增一个
  // VertexPointXY，返回 pointer
  g2o::VertexPointXY* getOrAddVertex(
      std::shared_ptr<g2o::SparseOptimizer> optimizer,
      const Eigen::Vector2d& pos, int& next_vid) {
    // 简单 hash: 0.1m 网格，可按需改
    int hx = int(std::round(pos.x() * 10.0));
    int hy = int(std::round(pos.y() * 10.0));
    int key = (hx << 16) ^ (hy & 0xFFFF);
    auto it = reflector_vertices.find(key);
    if (it != reflector_vertices.end()) {
      return dynamic_cast<g2o::VertexPointXY*>(optimizer->vertex(it->second));
    }
    // 新建顶点
    auto* v = new g2o::VertexPointXY();
    v->setId(next_vid);
    v->setEstimate(pos);
    // 可选择 v->setFixed(true/false) 根据是否优化反光柱
    v->setFixed(false);
    optimizer->addVertex(v);
    reflector_vertices[key] = next_vid;
    next_vid++;
    return v;
  }

  // 根据位置找到顶点 id（用于更新时查询）
  g2o::VertexPointXY* findVertex(
      std::shared_ptr<g2o::SparseOptimizer> optimizer,
      const Eigen::Vector2d& pos) {
    int hx = int(std::round(pos.x() * 10.0));
    int hy = int(std::round(pos.y() * 10.0));
    int key = (hx << 16) ^ (hy & 0xFFFF);
    auto it = reflector_vertices.find(key);
    if (it != reflector_vertices.end())
      return dynamic_cast<g2o::VertexPointXY*>(optimizer->vertex(it->second));
    return nullptr;
  }

 private:
  std::unordered_map<int, int> reflector_vertices;  // hash -> vid
};

class LaserMapping {
 public:
  LaserMapping(rclcpp::Node::SharedPtr node) : node(node) {}
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void front_icp_func();
  void optimize_map_func();
  void optimize_local_map_func();
  std::vector<unsigned long> find_loop_candidates(const Eigen::VectorXd& query,
                                                  int k = 1);
  int check_loop_closure(const KeyFrame& cur_key_frame);
  void add_kf_to_optimizer(const KeyFrame& cur_key_frame);
  // 从 optimizer 中读取所有关键帧估计回填
  void extractKeyFramePoses(std::vector<KeyFrame>& out_kfs);
  // 优化
  void optimizeLocal(int window_size = 20, int iterations = 10);
  void optimizeGlobal(int iterations = 30);
  ~LaserMapping() { Stop(); }
  void Init();
  void Run();
  void Stop() {
    is_running = false;
    scan_queue_cond.notify_all();
  }
  void add_loop_closure_edge(const KeyFrame& key_frame);

 private:
  g2o::SE2 cur_pose{0, 0, 0};
  std::atomic_bool is_loop{false};
  Eigen::Matrix3d covariance;
  std::atomic_bool is_running{true};
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_queue;
  std::unique_ptr<MapManager> map;
  std::unique_ptr<Solver> solver;
  std::unique_ptr<FeatureExtractor> feature_extractor;
  std::unique_ptr<ImuIntegrate> imu_integrate;
  std::shared_ptr<g2o::SparseOptimizer> global_optimizer;
  std::map<int, int> frame_vertices;
  GlobalReflectorMap reflector_map;
  std::map<int, int> corner_vertices;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  std::shared_ptr<std::thread> front_icp_thread;
  std::mutex scan_queue_mutex;
  std::condition_variable scan_queue_cond;
  std::shared_ptr<std::thread> optimize_map_thread;
  std::mutex optimize_pose_mutex;
  std::condition_variable optimize_pose_cond;
  std::chrono::steady_clock::time_point last_global_optimize_time;
};
}  // namespace loam
#endif