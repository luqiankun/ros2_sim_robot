#include "../include/laser_mapping.hpp"

namespace loam {
extern Eigen::Vector2d transformPoint(const Eigen::Vector2d& p,
                                      const g2o::SE2& pose);

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
struct FeatureCloud {
  std::vector<Eigen::VectorXd> descriptors;

  // nanoflann 接口
  inline size_t kdtree_get_point_count() const { return descriptors.size(); }

  inline double kdtree_distance(const double* a, const size_t b,
                                size_t /*size*/) const {
    double dist = 0;
    for (int i = 0; i < descriptors[b].size(); ++i) {
      double d = a[i] - descriptors[b](i);
      dist += d * d;
    }
    return dist;
  }

  inline double kdtree_get_pt(const size_t idx, int dim) const {
    return descriptors[idx](dim);
  }

  template <class BBOX>
  bool kdtree_get_bbox(BBOX&) const {
    return false;
  }
};

Eigen::Matrix3d computePoseCovariance(const g2o::SE2& pose,
                                      const RefMatchRes& ref_matches,
                                      const CornerMatchRes& corner_matches) {
  Eigen::Matrix3d H = Eigen::Matrix3d::Zero();

  auto accumulate = [&](const Eigen::Vector2d& obs,
                        const Eigen::Vector2d& map_pt, double weight) {
    // SE2 pose = [x, y, theta]
    double theta = pose.rotation().angle();
    double c = std::cos(theta);
    double s = std::sin(theta);

    // 机器人观测点旋转回世界坐标
    Eigen::Vector2d p_world = pose.rotation() * obs + pose.translation();
    Eigen::Vector2d e = p_world - map_pt;  // 残差
    (void)(e);                             // unused
    // 雅可比 w.r.t pose [dx, dy, dtheta]
    Eigen::Matrix<double, 2, 3> J;
    J << 1, 0, -s * obs.x() - c * obs.y(), 0, 1, c * obs.x() - s * obs.y();

    H += weight * J.transpose() * J;  // 简化信息矩阵为 identity
  };

  // 反光柱（权重大）
  for (const auto& m : ref_matches)
    accumulate(Eigen::Vector2d(m.first.x(), m.first.y()),
               Eigen::Vector2d(m.second.x(), m.second.y()),
               REFLECTOR_INFORMATION_WEIGHT);

  // 角点（权重小）
  for (const auto& m : corner_matches)
    accumulate(Eigen::Vector2d(m.first.x(), m.first.y()),
               Eigen::Vector2d(m.second.x(), m.second.y()),
               CORNER_INFORMATION_WEIGHT);

  // 协方差 = 信息矩阵逆
  Eigen::Matrix3d H_reg = H;
  double lambda = 1e-6;
  H_reg += lambda * Eigen::Matrix3d::Identity();
  // 用 LDLT 求逆（更稳健）
  Eigen::LDLT<Eigen::Matrix3d> ldlt(H_reg);
  Eigen::Matrix3d covariance = ldlt.solve(Eigen::Matrix3d::Identity());
  return covariance;
}

RefMatchRes Solver::associate_ref(RefObsArray& ref, const RefMap& map,
                                  const g2o::SE2& pose) {
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, PointCloud2D>, PointCloud2D, 2>;
  PointCloud2D map_ps;
  for (auto& x : map) map_ps.pts.push_back({x.pos.x(), x.pos.y()});
  KDTree index(2, map_ps, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();
  RefMatchRes result;
  for (auto& q : ref) {
    Eigen::Vector2d q_world = transformPoint(q.pos, pose);
    size_t index_;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultset(1);
    resultset.init(&index_, &out_dist_sqr);
    index.findNeighbors(resultset, q_world.data(),
                        nanoflann::SearchParameters(10));
    if (out_dist_sqr < flann_threshold * flann_threshold) {
      result.push_back({q.pos, map[index_].pos});
      q.id = map[index_].id;
    }
  }
  return result;
}

CornerMatchRes Solver::associate_corner(std::vector<CornerObs>& corners,
                                        const CornerMap& map,
                                        const g2o::SE2& pose) {
  using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, PointCloud2D>, PointCloud2D, 2>;
  PointCloud2D map_ps;
  for (auto& x : map) map_ps.pts.push_back({x.pos.x(), x.pos.y()});
  KDTree index(2, map_ps, nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();
  RefMatchRes result;
  for (auto& q : corners) {
    Eigen::Vector2d q_world = transformPoint(q.pos, pose);
    size_t index_;
    double out_dist_sqr;
    nanoflann::KNNResultSet<double> resultset(1);
    resultset.init(&index_, &out_dist_sqr);
    index.findNeighbors(resultset, q_world.data(),
                        nanoflann::SearchParameters(10));
    if (out_dist_sqr < flann_threshold * flann_threshold) {
      result.push_back({q.pos, map[index_].pos});
    }
  }
  return result;
}

Eigen::Isometry2d Solver::compute_svd_transform(const RefMatchRes& match) {
  assert(match.size() >= 2);
  Eigen::MatrixXd src(2, match.size());
  Eigen::MatrixXd dst(2, match.size());

  for (size_t i = 0; i < match.size(); ++i) {
    src.col(i) = match[i].first;   // z_j (观测)
    dst.col(i) = match[i].second;  // m_i (地图)
  }
  Eigen::Vector2d src_mean = src.rowwise().mean();
  Eigen::Vector2d dst_mean = dst.rowwise().mean();

  src.colwise() -= src_mean;
  dst.colwise() -= dst_mean;

  Eigen::Matrix2d W = src * dst.transpose();
  Eigen::JacobiSVD<Eigen::Matrix2d> svd(
      W, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();
  if (R.determinant() < 0) R.col(1) *= -1;  // 保证旋转矩阵

  Eigen::Vector2d t = dst_mean - R * src_mean;
  Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
  T.linear() = R;
  T.translation() = t;
  return T;
}
g2o::SE2 Solver::front_icp(RefObsArray& obs, const RefMap& map,
                           CornerObsArray& corner_obs,
                           const CornerMap& corner_map,
                           const Eigen::Isometry2d& init_pre,
                           Eigen::Matrix3d& covariance) {
  auto makeOptimizer = []() -> std::unique_ptr<g2o::SparseOptimizer> {
    using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
    using LinearSolverType =
        g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

    auto linearSolver = std::make_unique<LinearSolverType>();
    auto blockSolver =
        std::make_unique<BlockSolverType>(std::move(linearSolver));
    auto solver =
        new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));

    auto opt = std::make_unique<g2o::SparseOptimizer>();
    opt->setAlgorithm(solver);
    return opt;
  };
  // 初始位姿（世界坐标系）
  g2o::SE2 pose{init_pre.translation().x(), init_pre.translation().y(),
                Eigen::Rotation2Dd(init_pre.rotation()).angle()};
  RefMatchRes last_math;
  CornerMatchRes last_corner_match;
  const int max_iters = 20;

  for (int iter = 0; iter < max_iters; ++iter) {
    // 与局部地图做关联
    RefMatchRes match = associate_ref(obs, map, pose);
    CornerMatchRes match_corner =
        associate_corner(corner_obs, corner_map, pose);

    if (match.size() + match_corner.size() < 3) {
      // 匹配太少，不能可靠优化
      break;
    }

    if (iter == 0) {
      // 用反光板做 SVD 初值（若 match 非空）
      if (match.size() >= 2) {
        Eigen::Isometry2d init = compute_svd_transform(match);
        pose = g2o::SE2(init.translation().x(), init.translation().y(),
                        Eigen::Rotation2Dd(init.rotation()).angle());
      }
    }

    // 每次迭代用新的 optimizer
    auto optimizer = makeOptimizer();

    // 添加机器人位姿顶点 id = 0
    g2o::VertexSE2* v = new g2o::VertexSE2();
    v->setId(0);
    v->setEstimate(g2o::SE2(pose.translation()[0], pose.translation()[1],
                            pose.rotation().angle()));
    optimizer->addVertex(v);

    // 从 id = 1
    // 开始逐个添加地图点顶点（本次观测的反光板和角点），并添加点观测边
    int vid = 1;
    // 反光板（权重大）
    for (const auto& m : match) {
      auto* lm = new g2o::VertexPointXY();
      lm->setId(vid);
      lm->setEstimate(
          Eigen::Vector2d(m.second.x(), m.second.y()));  // 地图点 (in world)
      lm->setFixed(true);                                // 地图点固定
      optimizer->addVertex(lm);

      auto* edge = new g2o::EdgeSE2PointXY();
      edge->setVertex(0, v);
      edge->setVertex(1, lm);
      edge->setMeasurement(Eigen::Vector2d(
          m.first.x(), m.first.y()));  // 机器人观测 in robot frame
      edge->setInformation(
          Eigen::Matrix2d::Identity() *
          REFLECTOR_INFORMATION_WEIGHT);  // 反光板权重大，按需调
      edge->setRobustKernel(new g2o::RobustKernelHuber());  // 加鲁棒核
      edge->robustKernel()->setDelta(1 / sqrt(REFLECTOR_INFORMATION_WEIGHT) *
                                     2);
      optimizer->addEdge(edge);

      ++vid;
    }

    // 角点（权重较小）
    for (const auto& m : match_corner) {
      auto* lm = new g2o::VertexPointXY();
      lm->setId(vid);
      lm->setEstimate(Eigen::Vector2d(m.second.x(), m.second.y()));
      lm->setFixed(true);
      optimizer->addVertex(lm);

      auto* edge = new g2o::EdgeSE2PointXY();
      edge->setVertex(0, v);
      edge->setVertex(1, lm);
      edge->setMeasurement(Eigen::Vector2d(m.first.x(), m.first.y()));
      edge->setInformation(Eigen::Matrix2d::Identity() *
                           CORNER_INFORMATION_WEIGHT);  // 角点权重中等/较小
      edge->setRobustKernel(new g2o::RobustKernelHuber());
      edge->robustKernel()->setDelta(1 / sqrt(CORNER_INFORMATION_WEIGHT) * 2);
      optimizer->addEdge(edge);

      ++vid;
    }

    // 执行优化
    optimizer->initializeOptimization();
    optimizer->optimize(10);

    // 读取当前估计作为下一轮的初值
    g2o::SE2 new_pose = v->estimate();
    Eigen::Vector3d delta = new_pose.toVector() - pose.toVector();
    pose = new_pose;
    last_math = match;
    last_corner_match = match_corner;
    // 收敛判断
    if (delta.norm() < 1e-4) break;
  }  // end iter loop

  covariance = computePoseCovariance(pose, last_math, last_corner_match);

  return pose;
}

//////////////////////////////////////////
///

void LaserMapping::Init() {
  using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<-1, -1>>;
  using LinearSolverType =
      g2o::LinearSolverDense<BlockSolverType::PoseMatrixType>;

  auto linearSolver = std::make_unique<LinearSolverType>();
  auto blockSolver = std::make_unique<BlockSolverType>(std::move(linearSolver));
  auto op_solver =
      new g2o::OptimizationAlgorithmGaussNewton(std::move(blockSolver));
  global_optimizer = std::make_shared<g2o::SparseOptimizer>();
  global_optimizer->setAlgorithm(op_solver);
  map = std::make_unique<MapManager>();
  feature_extractor = std::make_unique<FeatureExtractor>(node);
  solver = std::make_unique<Solver>();
  imu_integrate = std::make_unique<ImuIntegrate>(node);
  sub_ = node->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar", 100,
      std::bind(&LaserMapping::callback, this, std::placeholders::_1));
}
void LaserMapping::Run() {
  is_running = true;
  front_icp_thread =
      std::make_shared<std::thread>(&LaserMapping::front_icp_func, this);
  optimize_map_thread =
      std::make_shared<std::thread>(&LaserMapping::optimize_map_func, this);
}
void LaserMapping::callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  std::unique_lock<std::mutex> lock(scan_queue_mutex);
  scan_queue.push_back(msg);
  scan_queue_cond.notify_one();
}

void LaserMapping::front_icp_func() {
  bool first_scan = true;
  while (is_running) {
    std::unique_lock<std::mutex> lock(scan_queue_mutex);
    scan_queue_cond.wait(lock, [&] { return !scan_queue.empty(); });
    auto scan = scan_queue.front();
    scan_queue.pop_front();
    lock.unlock();
    if (first_scan) {
      feature_extractor->extract_high_intensity_points(scan);
      bool success1 = feature_extractor->extract_reflector_obs();
      bool success2 = feature_extractor->extract_corner(scan);
      if (!success1 && !success2) {
        RCLCPP_WARN(node->get_logger(), "feature size too small");
        continue;
      } else {
        // 初始化地图
        KeyFrame frame;
        frame.pose = Eigen::Isometry2d::Identity();
        frame.id = 0;
        for (auto& x : feature_extractor->reflector_obs_) {
          Reflector ref = x;
          ref.id = frame.reflectors.size();
          frame.reflectors.push_back(ref);
        }
        frame.corners = feature_extractor->corner_obs_;
        frame.scan = scan;
        frame.time_stamp =
            scan->header.stamp.sec + scan->header.stamp.nanosec / 1e9;
        frame.computeDescriptor();
        map->addKeyFrameToCache(frame);
        first_scan = false;
      }
    } else {
      feature_extractor->extract_high_intensity_points(scan);
      bool success1 = feature_extractor->extract_reflector_obs();
      bool success2 = feature_extractor->extract_corner(scan);
      if (!success1 && !success2) {
        RCLCPP_WARN(node->get_logger(), "feature size too small");
        continue;
      }
      double time_now =
          scan->header.stamp.sec + scan->header.stamp.nanosec / 1e9;
      Eigen::Isometry2d delta_t = imu_integrate->get_delta_pose(
          feature_extractor->last_scan_time_, time_now);
      feature_extractor->last_scan_time_ = time_now;
      Eigen::Isometry2d pose = cur_pose.toIsometry();
      pose = pose * delta_t;
      g2o::SE2 pre{pose.translation().x(), pose.translation().y(),
                   Eigen::Rotation2Dd(pose.rotation()).angle()};
      // 获取局部地图
      map->integrateCacheFrames(this);
      std::vector<Reflector> ref_map;
      std::vector<Corner> corner_map;
      std::vector<Line> line_map;
      map->getLocalMapFeatures(ref_map, corner_map, line_map);
      Eigen::Matrix3d cov;
      g2o::SE2 est_pose = solver->front_icp(
          feature_extractor->reflector_obs_, ref_map,
          feature_extractor->corner_obs_, corner_map, pose, cov);
      // 判断关键帧
      auto last_kf = map->getLatestKeyFrame();
      auto last_kf_time = last_kf.time_stamp;
      g2o::SE2 last_se2{last_kf.pose.translation().x(),
                        last_kf.pose.translation().y(),
                        Eigen::Rotation2Dd(last_kf.pose.rotation()).angle()};
      g2o::SE2 delta_pose = last_se2.inverse() * est_pose;
      if (delta_pose.translation().norm() > 0.3 ||
          delta_pose.rotation().angle() > 5.0 / 180 * M_PI ||
          (time_now - last_kf_time) > 10) {
        KeyFrame frame;
        frame.pose = est_pose.toIsometry();
        frame.id = last_kf.id + 1;
        frame.scan = scan;
        frame.cov = cov;
        frame.time_stamp =
            scan->header.stamp.sec + scan->header.stamp.nanosec / 1e9;
        for (auto& ref : feature_extractor->reflector_obs_) {
          auto sensor_world = est_pose.toIsometry() * ref.pos;
          Reflector new_ref;
          new_ref.pos = sensor_world;
          frame.reflectors.push_back(new_ref);
        }
        for (auto& corner : feature_extractor->corner_obs_) {
          auto sensor_world = est_pose.toIsometry() * corner.pos;
          Corner new_corner;
          new_corner.pos = sensor_world;
          frame.corners.push_back(new_corner);
        }
        frame.computeDescriptor();
        map->addKeyFrameToCache(frame);

        optimize_pose_cond.notify_one();
        RCLCPP_WARN_STREAM(node->get_logger(),
                           "add new kf" << frame.reflectors.size() << " "
                                        << frame.corners.size());
      }
      RCLCPP_INFO(node->get_logger(), "est_pose %f %f %f--%ld------",
                  est_pose.translation().x(), est_pose.translation().y(),
                  Eigen::Rotation2Dd(est_pose.rotation()).angle(),
                  map->getLocalWindow().size());
      cur_pose = est_pose;
    }
  }
}

void LaserMapping::add_kf_to_optimizer(const KeyFrame& kf) {
  static int id = 0;
  if (frame_vertices.count(kf.id) == 0) {
    auto* v = new g2o::VertexSE2();
    v->setId(id);
    v->setEstimate(g2o::SE2(kf.pose.translation().x(),
                            kf.pose.translation().y(),
                            Eigen::Rotation2Dd(kf.pose.rotation()).angle()));
    if (kf.id == 0) v->setFixed(true);
    global_optimizer->addVertex(v);
    frame_vertices[kf.id] = id;
    id++;
  }
  auto v_pose = dynamic_cast<g2o::VertexSE2*>(
      global_optimizer->vertex(frame_vertices[kf.id]));
  // 反光柱
  for (auto& r : kf.reflectors) {
    auto* v_ref = reflector_map.getOrAddVertex(global_optimizer, r.pos, id);
    auto* edge = new g2o::EdgeSE2PointXY();
    edge->setVertex(0, v_pose);
    edge->setVertex(1, v_ref);
    edge->setMeasurement(v_pose->estimate().inverse() * r.pos);
    edge->setInformation(Eigen::Matrix2d::Identity() *
                         REFLECTOR_INFORMATION_WEIGHT);
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    edge->robustKernel()->setDelta(1 / sqrt(REFLECTOR_INFORMATION_WEIGHT) * 2);
    global_optimizer->addEdge(edge);
  }
  // 角点
  for (auto& c : kf.corners) {
    g2o::VertexPointXY* v_corner = nullptr;
    int hash = int(c.pos.x() * 1e3 + c.pos.y() * 1e3);
    if (corner_vertices.count(hash) == 0) {
      v_corner = new g2o::VertexPointXY();
      v_corner->setId(id);
      v_corner->setEstimate(c.pos);
      v_corner->setFixed(true);  // 可按需优化
      global_optimizer->addVertex(v_corner);
      corner_vertices[hash] = id;
      id++;
    } else {
      v_corner = dynamic_cast<g2o::VertexPointXY*>(
          global_optimizer->vertex(corner_vertices[hash]));
    }
    auto* edge = new g2o::EdgeSE2PointXY();
    edge->setVertex(0, v_pose);
    edge->setVertex(1, v_corner);
    edge->setMeasurement(v_pose->estimate().inverse() * c.pos);
    edge->setInformation(Eigen::Matrix2d::Identity() *
                         CORNER_INFORMATION_WEIGHT);
    edge->setRobustKernel(new g2o::RobustKernelHuber());
    edge->robustKernel()->setDelta(1 / sqrt(CORNER_INFORMATION_WEIGHT) * 2);
    global_optimizer->addEdge(edge);
  }
}

void LaserMapping::optimize_map_func() {
  while (is_running) {
    std::unique_lock<std::mutex> lock(optimize_pose_mutex);
    optimize_pose_cond.wait_for(lock, std::chrono::seconds(1));
    auto start_time = std::chrono::steady_clock::now();
    auto kfs = map->integrateCacheFrames(this);
    if (global_optimizer->vertices().empty()) continue;
    bool do_global = false;
    auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration_cast<std::chrono::seconds>(
            now - last_global_optimize_time)
            .count() > 20) {
      do_global = true;
      last_global_optimize_time = now;
    }
    if (is_loop) {
      do_global = true;
      is_loop = false;
    }
    if (do_global) {
      global_optimizer->initializeOptimization();
      global_optimizer->optimize(20);
    } else {
      const int N = 20;
      std::set<g2o::HyperGraph::Edge*> active_edges;

      for (auto& [kf_id, vid] : frame_vertices) {
        if (kf_id >= (int)kfs.size() - N) {
          auto* v = global_optimizer->vertex(vid);
          if (!v) continue;
          for (auto* e : v->edges()) {
            active_edges.insert(e);  // 收集这些顶点相关的边
          }
        }
      }
      if (!active_edges.empty()) {
        global_optimizer->initializeOptimization(active_edges);
        global_optimizer->optimize(10);
      }
    }

    // 更新
    std::vector<KeyFrame> results;
    for (auto& kf : kfs) {
      auto* v = dynamic_cast<g2o::VertexSE2*>(
          global_optimizer->vertex(frame_vertices[kf.first]));
      if (!v) continue;
      g2o::SE2 est = v->estimate();
      kf.second.pose =
          Eigen::Translation2d(est.translation()[0], est.translation()[1]) *
          Eigen::Rotation2Dd(est.rotation().angle());
      for (auto& r : kf.second.reflectors) {
        auto* v_pt = reflector_map.findVertex(global_optimizer, r.pos);
        if (v_pt) r.pos = v_pt->estimate();
      }
      results.push_back(kf.second);
    }
    map->updateKeyFrames(results);
    auto end_time = std::chrono::steady_clock::now();
    RCLCPP_INFO_STREAM(
        node->get_logger(),
        " optimize time: "
            << std::chrono::duration<double>(end_time - start_time).count()
            << std::endl);
  }
}

std::vector<unsigned long> LaserMapping::find_loop_candidates(
    const Eigen::VectorXd& query, int k) {
  typedef nanoflann::KDTreeSingleIndexAdaptor<
      nanoflann::L2_Simple_Adaptor<double, FeatureCloud>, FeatureCloud, -1>
      KDTree;
  FeatureCloud cloud;
  auto kfs = map->getAllKeyFrames();

  for (auto& kf : kfs) {
    cloud.descriptors.push_back(kf.second.descriptor);
  }

  KDTree index(cloud.descriptors[0].size(), cloud,
               nanoflann::KDTreeSingleIndexAdaptorParams(10));
  index.buildIndex();
  std::vector<unsigned long> result_indices(k);
  std::vector<double> out_dists_sqr(k);
  nanoflann::KNNResultSet<double> resultSet(k);
  resultSet.init(result_indices.data(), out_dists_sqr.data());
  index.findNeighbors(resultSet, query.data());
  return result_indices;
}

int LaserMapping::check_loop_closure(const KeyFrame& cur_key_frame) {
  auto query = cur_key_frame.descriptor;
  auto candidates = find_loop_candidates(query, 3);
  for (auto& x : candidates) {
    if ((int)x == cur_key_frame.id) continue;
    auto kf = map->getKeyFrame(x);
    if ((cur_key_frame.id - kf.id) < 30) {
      continue;
    }
    Eigen::Isometry2d pre = kf.pose.inverse() * cur_key_frame.pose;
    if (pre.translation().norm() > 10) continue;
    Eigen::Matrix3d cov;
    auto T_loop =
        solver->front_icp(kf.reflectors, cur_key_frame.reflectors, kf.corners,
                          cur_key_frame.corners, pre, cov);
    if (T_loop.translation().norm() > 2) continue;
    if (cov(0, 0) > 0.1 || cov(1, 1) > 0.1 || cov(2, 2) > 0.1) continue;
    return x;
  }
  return -1;
}
void LaserMapping::add_loop_closure_edge(const KeyFrame& kf) {
  int loop = check_loop_closure(kf);
  if (loop >= 0) {
    auto old = map->getKeyFrame(loop);
    // icp
    Eigen::Isometry2d pre_t = old.pose.inverse() * kf.pose;
    RefObsArray last_refs;
    for (auto& ref : old.reflectors) {
      Reflector new_ref;
      new_ref.pos = old.pose.inverse() * ref.pos;
      last_refs.push_back(new_ref);
    }
    CornerObsArray last_corners;
    for (auto& corner : old.corners) {
      Corner new_corner;
      new_corner.pos = old.pose.inverse() * corner.pos;
      last_corners.push_back(new_corner);
    }
    RefObsArray cur_refs;
    for (auto& ref : kf.reflectors) {
      Reflector new_ref;
      new_ref.pos = kf.pose.inverse() * ref.pos;
      cur_refs.push_back(new_ref);
    }
    CornerObsArray cur_corners;
    for (auto& corner : kf.corners) {
      Corner new_corner;
      new_corner.pos = kf.pose.inverse() * corner.pos;
      cur_corners.push_back(new_corner);
    }
    Eigen::Matrix3d cov;
    auto T = solver->front_icp(last_refs, cur_refs, last_corners, cur_corners,
                               pre_t, cov);
    auto* v_from = dynamic_cast<g2o::VertexSE2*>(
        global_optimizer->vertex(frame_vertices[loop]));
    auto* v_to = dynamic_cast<g2o::VertexSE2*>(
        global_optimizer->vertex(frame_vertices[kf.id]));
    if (!v_from || !v_to) return;
    auto* edge = new g2o::EdgeSE2();
    edge->setVertex(0, v_from);
    edge->setVertex(1, v_to);
    double dx = T.translation().x();
    double dy = T.translation().y();
    g2o::SE2 meas(dx, dy, Eigen::Rotation2Dd(T.rotation()).angle());
    edge->setMeasurement(meas);
    edge->setInformation(Eigen::Matrix3d::Identity() * 400);
    // Robust kernel to reduce effect of false-positive loops
    auto* rk = new g2o::RobustKernelHuber();
    rk->setDelta(1.0);
    edge->setRobustKernel(rk);
    global_optimizer->addEdge(edge);
    is_loop = true;
    RCLCPP_INFO_STREAM(node->get_logger(),
                       "add loop closure edge-----------------------");
  }
}

}  // namespace loam