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
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/slam2d/types_slam2d.h>

#include "./occupacy_gridmap.hpp"
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
struct MapVertexInfo {
  int vid;
  int ref_count;
};
class GlobalReflectorMap {
 public:
  // 查找已有顶点（通过大致位置 hash），若不存在则在 optimizer 中新增一个
  // VertexPointXY，返回 pointer
  g2o::VertexPointXY* getOrAddVertex(
      std::shared_ptr<g2o::SparseOptimizer> optimizer,
      const Eigen::Vector2d& pos, int& next_vid, bool is_local) {
    // 简单 hash: 0.5m 网格，可按需改
    int hx = int(std::round(pos.x() * 50.0));
    int hy = int(std::round(pos.y() * 50.0));
    int key = (hx << 16) ^ (hy & 0xFFFF);
    // 先查找同 hash 顶点
    auto it = reflector_vertices.find(key);
    if (it != reflector_vertices.end()) {
      auto* v =
          dynamic_cast<g2o::VertexPointXY*>(optimizer->vertex(it->second.vid));
      if ((v->estimate() - pos).norm() < 0.1) {
        it->second.ref_count++;
        return v;
      }
    }
    // 新建顶点
    auto* v = new g2o::VertexPointXY();
    v->setId(next_vid);
    v->setEstimate(pos);
    // 可选择 v->setFixed(true/false) 根据是否优化反光柱
    v->setFixed(is_local);
    optimizer->addVertex(v);
    reflector_vertices[key] = {next_vid, 1};
    next_vid++;
    return v;
  }
  std::vector<Eigen::Vector2d> getAllPoints(
      std::shared_ptr<g2o::SparseOptimizer> optimizer) {
    std::vector<Eigen::Vector2d> points;
    std::vector<bool> used(reflector_vertices.size(), false);
    std::vector<std::pair<int, Eigen::Vector2d>> verts;

    for (auto& kv : reflector_vertices) {
      auto* v =
          dynamic_cast<g2o::VertexPointXY*>(optimizer->vertex(kv.second.vid));
      if (v) verts.push_back({kv.second.vid, v->estimate()});
    }

    for (size_t i = 0; i < verts.size(); ++i) {
      if (used[i]) continue;
      Eigen::Vector2d sum = verts[i].second;
      int count = 1;
      used[i] = true;
      for (size_t j = i + 1; j < verts.size(); ++j) {
        if ((verts[j].second - verts[i].second).norm() < 0.1) {
          sum += verts[j].second;
          used[j] = true;
          count++;
        }
      }
      points.push_back(sum / count);
    }

    return points;
  }
  // 根据位置找到顶点 id（用于更新时查询）
  g2o::VertexPointXY* findVertex(
      std::shared_ptr<g2o::SparseOptimizer> optimizer,
      const Eigen::Vector2d& pos) {
    int hx = int(std::round(pos.x() * 50.0));
    int hy = int(std::round(pos.y() * 50.0));
    int key = (hx << 16) ^ (hy & 0xFFFF);
    auto it = reflector_vertices.find(key);
    if (it != reflector_vertices.end())
      return dynamic_cast<g2o::VertexPointXY*>(
          optimizer->vertex(it->second.vid));
    return nullptr;
  }
  void releaseVertex(std::shared_ptr<g2o::SparseOptimizer> optimizer,
                     const Eigen::Vector2d& pos) {
    int hx = int(std::round(pos.x() * 50.0));
    int hy = int(std::round(pos.y() * 50.0));
    int key = (hx << 16) ^ (hy & 0xFFFF);
    auto it = reflector_vertices.find(key);
    if (it == reflector_vertices.end()) return;
    it->second.ref_count--;
    if (it->second.ref_count > 0) return;

    int vid = it->second.vid;
    reflector_vertices.erase(it);

    auto* v = optimizer->vertex(vid);
    if (!v) return;

    // 先删边
    std::vector<g2o::HyperGraph::Edge*> edges_to_remove(v->edges().begin(),
                                                        v->edges().end());
    for (auto* e : edges_to_remove) {
      optimizer->removeEdge(e);
      // delete e;
    }
    optimizer->removeVertex(v);
    // delete v;
  }

 private:
  std::unordered_map<int, MapVertexInfo> reflector_vertices;  // hash -> vid
};

class LaserMapping {
 public:
  LaserMapping(rclcpp::Node::SharedPtr node) : node(node) {}
  void callback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
  void front_icp_func();
  void optimize_local_map_func();
  void optimize_global_map_func();
  std::vector<unsigned long> find_loop_candidates(const Eigen::VectorXd& query,
                                                  int k = 1);
  int check_loop_closure(const KeyFrame& cur_key_frame);
  void add_kf_to_global_optimizer(const KeyFrame& cur_key_frame);
  void add_kf_to_local_optimizer(const KeyFrame& cur_key_frame);
  // 从 optimizer 中读取所有关键帧估计回填
  void extractKeyFramePoses(std::vector<KeyFrame>& out_kfs);
  ~LaserMapping() {
    Stop();
    if (front_icp_thread->joinable()) front_icp_thread->join();
    if (optimize_map_thread->joinable()) optimize_map_thread->join();
    if (optimize_global_map_thread->joinable())
      optimize_global_map_thread->join();
  }
  void Init();
  void Run();
  void Save() {
    occupancy_grid_map->updateReflectorMap(
        global_reflector_map.getAllPoints(global_optimizer));
    occupancy_grid_map->Stop();
  }
  void Stop() {
    is_running = false;
    scan_queue_cond.notify_all();
    optimize_pose_cond.notify_all();
    optimize_global_pose_cond.notify_all();
  }
  void add_loop_closure_edge(const KeyFrame& key_frame);

 private:
  g2o::SE2 cur_pose{0, 0, 0};
  Eigen::Matrix3d covariance;
  std::atomic_bool is_running{true};
  std::deque<sensor_msgs::msg::LaserScan::SharedPtr> scan_queue;
  std::unique_ptr<MapManager> map;
  std::unique_ptr<Solver> solver;
  std::unique_ptr<FeatureExtractor> feature_extractor;
  std::unique_ptr<ImuIntegrate> imu_integrate;
  std::unique_ptr<MapPublisher> occupancy_grid_map;
  uint32_t icp_fail_count = 0;
  std::shared_ptr<g2o::SparseOptimizer> global_optimizer;
  std::shared_ptr<g2o::SparseOptimizer> local_optimizer;
  GlobalReflectorMap global_reflector_map;
  std::map<int, int> global_frame_vertices;
  std::map<int, int> global_corner_vertices;
  GlobalReflectorMap local_reflector_map;
  std::map<int, int> local_frame_vertices;
  std::map<int, int> local_corner_vertices;

  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;

  std::shared_ptr<std::thread> front_icp_thread;
  std::mutex scan_queue_mutex;
  std::condition_variable scan_queue_cond;
  std::shared_ptr<std::thread> optimize_map_thread;
  std::mutex optimize_pose_mutex;
  std::condition_variable optimize_pose_cond;
  std::shared_ptr<std::thread> optimize_global_map_thread;
  std::mutex optimize_global_pose_mutex;
  std::condition_variable optimize_global_pose_cond;
};
}  // namespace loam
#endif