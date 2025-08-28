#include "../include/map_optimization.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
namespace reflector_slam {
Eigen::Matrix<double, 6, 6> createInformationMatrix(double trans_std,
                                                    double rot_std) {
  Eigen::Matrix<double, 6, 6> info = Eigen::Matrix<double, 6, 6>::Zero();

  // 位置信息（对角线为方差的倒数）
  double trans_info = 1.0 / (trans_std * trans_std);
  info.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * trans_info;

  // 旋转信息
  double rot_info = 1.0 / (rot_std * rot_std);
  info.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * rot_info;

  return info;
}
MapOptimization::MapOptimization(rclcpp::Node::SharedPtr node) : node_(node) {
  // odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
  //     "odom", 10,
  //     std::bind(&MapOptimization::odomCallback, this,
  //     std::placeholders::_1));
  // 注意ROS2读参数层级关系是".",而不是"/"
  node_->declare_parameter<int>("map_optimizer.window_size", 10);
  node_->declare_parameter<double>("map_optimizer.duplicates_threshold", 0.3);
  node_->declare_parameter<double>("map_optimizer.keyframe_distance", 0.5);
  node_->declare_parameter<double>("map_optimizer.keyframe_angle", 10.0);
  node_->declare_parameter<int>("map_optimizer.ref_share_count", 3);
  node_->declare_parameter<std::string>("map_save.save_path", "./map");
  node_->declare_parameter<int>("map_save.image_width", 1000);
  node_->declare_parameter<int>("map_save.image_height", 1000);
  node_->declare_parameter<double>("map_save.origin_x", -50);
  node_->declare_parameter<double>("map_save.origin_y", -50);
  node_->declare_parameter<double>("map_save.resolution", 0.1);
  node_->declare_parameter<double>("ref_extractor.max_radius", 0.04);
  node_->declare_parameter<double>("ref_extractor.min_radius", 0.03);
  node_->declare_parameter<double>("ref_extractor.cluster_threshold", 0.1);
  node_->declare_parameter<double>("ref_extractor.match_threshold", 0.3);
  node_->declare_parameter<int>("ref_extractor.max_iterations", 10);
  window_size = node->get_parameter_or<int>("map_optimizer.window_size", 10);
  duplicates_threshold_ =
      node->get_parameter_or<double>("map_optimizer.duplicates_threshold", 0.3);
  keyframe_distance_ =
      node->get_parameter_or<double>("map_optimizer.keyframe_distance", 0.5);
  keyframe_angle_ =
      node->get_parameter_or<double>("map_optimizer.keyframe_angle", 10.0);
  ref_share_count_ =
      node->get_parameter_or<int>("map_optimizer.ref_share_count", 3);

  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar", 10,
      std::bind(&MapOptimization::laserCallback, this, std::placeholders::_1));
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "reflector_markers", 10);

  double max_radius =
      node->get_parameter_or<double>("ref_extractor.max_radius", 0.04);
  double min_radius =
      node->get_parameter_or<double>("ref_extractor.min_radius", 0.03);
  double cluster_threshold =
      node->get_parameter_or<double>("ref_extractor.cluster_threshold", 0.1);
  double match_threshold =
      node->get_parameter_or<double>("ref_extractor.match_threshold", 0.3);
  int max_iterations =
      node->get_parameter_or<int>("ref_extractor.max_iterations", 10);
  double identify_threshold =
      node->get_parameter_or<double>("ref_extractor.identify_threshold", 0.8);
  reflector_extractor_ = std::make_shared<FeatureExtractor>(
      cluster_threshold, match_threshold, min_radius, max_radius,
      identify_threshold, max_iterations);

  std::string save_path =
      node->get_parameter_or<std::string>("map_save.save_path", "./map");
  int image_width = node->get_parameter_or<int>("map_save.image_width", 1000);
  int image_height = node->get_parameter_or<int>("map_save.image_height", 1000);
  double origin_x = node->get_parameter_or<double>("map_save.origin_x", -50);
  double origin_y = node->get_parameter_or<double>("map_save.origin_y", -50);
  double resolution =
      node->get_parameter_or<double>("map_save.resolution", 0.1);
  map_manager_ = std::make_shared<MapManager>(
      resolution, origin_x, origin_y, image_width, image_height, save_path);
  optimize_thread_ = std::thread(&MapOptimization::optimize_thread, this);
  optimize_and_save_service_ = node_->create_service<std_srvs::srv::Empty>(
      "optimize_and_save",
      std::bind(&MapOptimization::save_map, this, std::placeholders::_1,
                std::placeholders::_2));
  map_thread_ = std::thread([&] {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lock(map_save_mutex);
      map_cv.wait(lock);
      map_manager_->generate_from_keyframe(map, keyframes, optimized_map_,
                                           optimized_reflectors_);
      map_manager_->save_map();
      RCLCPP_INFO(node_->get_logger(), "Map saved.");
    }
  });
}

void MapOptimization::odomCallback(
    const nav_msgs::msg::Odometry::SharedPtr msg) {
  Eigen::Matrix4d lidar_to_base = Eigen::Matrix4d::Identity();
  lidar_to_base(0, 3) = 1.125;

  Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

  transform(0, 3) = msg->pose.pose.position.x;
  transform(1, 3) = msg->pose.pose.position.y;
  transform(2, 3) = msg->pose.pose.position.z;
  Eigen::Quaterniond q(
      msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
      msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
  q.normalize();
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  odom_pose = lidar_to_base.inverse() * transform *
              lidar_to_base;  // odom是base_link下的变换，要转换到雷达坐标系下
}

bool MapOptimization::save_map(
    const std_srvs::srv::Empty::Request::SharedPtr&,
    const std_srvs::srv::Empty::Response::SharedPtr) {
  map_cv.notify_all();
  return true;
}

void MapOptimization::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  auto cur_frame = reflector_extractor_->extract(msg);
  std::unique_lock<std::mutex> lock(ref_mutex);
  if (map.empty()) {
    for (size_t i = 0; i < cur_frame.size(); ++i) {
      Reflector ref;
      ref.id = map.size();
      ref.position = cur_frame[i].point;
      map[ref.id] = ref;
      optimized_reflectors_[ref.id] = ref.position;
    }
    Keyframe frame;
    frame.id = keyframes.size();
    frame.pose = cur_pose_;
    frame.observations = cur_frame;
    frame.timestamp = std::chrono::steady_clock::now();
    keyframes[frame.id] = frame;
    optimized_map_[frame.id] = frame.pose;
    lock.unlock();
  } else {
    Eigen::Matrix4d cur_pose =
        reflector_extractor_->match(cur_frame, map, cur_pose_);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       cur_pose(0, 3) << " " << cur_pose(1, 3) << std::endl);
    Eigen::Matrix4d odom = keyframes[keyframes.size() - 1].pose.inverse() *
                           cur_pose;  // T_cur_last
    int own = 0;
    for (auto& obs : keyframes[keyframes.size() - 1].observations) {
      auto it =
          std::find_if(cur_frame.begin(), cur_frame.end(),
                       [&obs](const Observation& x) { return x.id == obs.id; });
      if (it != cur_frame.end()) {
        own++;
      }
    }
    Eigen::Vector3d t = odom.block<3, 1>(0, 3);
    Eigen::Quaterniond Qua(odom.block<3, 3>(0, 0));
    Qua.normalize();
    double angle = 2 * acos(Qua.w());  // yaw
    cur_pose_ = cur_pose;
    for (auto x : cur_frame) {
      // 没有匹配的 新加入
      if (x.id == -1) {
        Eigen::Vector3d pose = (cur_pose_ * x.point.homogeneous()).head<3>();
        bool exit = false;
        for (auto& x : map) {
          if ((x.second.position - pose).norm() < duplicates_threshold_) {
            exit = true;
            break;
          }
        }
        if (!exit) {
          Reflector ref;
          ref.id = map.size();
          ref.position = (cur_pose_ * x.point.homogeneous()).head<3>();
          map[ref.id] = ref;
          optimized_reflectors_[ref.id] = ref.position;
          RCLCPP_INFO(node_->get_logger(), "new ref %d", ref.id);
        }
      }
    }
    lock.unlock();
    if (t.norm() > keyframe_distance_ ||
        fabs(angle) > keyframe_angle_ * M_PI / 180 || own < ref_share_count_) {
      Keyframe frame;
      frame.id = keyframes.size();
      frame.scan = msg;
      frame.pose = cur_pose;
      for (auto& obs : cur_frame) {
        if (obs.id != -1) {
          frame.observations.push_back(obs);
        }
      }
      frame.timestamp = std::chrono::steady_clock::now();
      std::unique_lock<std::mutex> lock2(key_mutex);
      if (keyframes.size() > 0) {
        // 添加里程计信息
        auto last = keyframes[keyframes.size() - 1];
        Odometry odom;
        auto info_mat = createInformationMatrix(0.1, 0.1);
        for (int i = 0; i < 6; i++) {
          for (int j = 0; j < 6; j++) {
            odom.info[i][j] = info_mat(i, j);
          }
        }
        odom.last_id = keyframes.size() - 1;
        odom.cur_id = keyframes.size();
        odom.last_pose = last.pose;
        odom.current_pose = cur_pose;
        odoms.push_back(odom);
      }
      keyframes[frame.id] = frame;
      optimized_map_[frame.id] = frame.pose;
      lock2.unlock();
      // if ((keyframes.size() - last_opt_size_) > 1) {
      //   cv.notify_all();
      // }
      cv.notify_one();
      // optimize();
    }
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = node_->now();
      transformStamped.header.frame_id = "map";
      transformStamped.child_frame_id = "main_2d_lidar_link";
      transformStamped.transform.translation.x = cur_pose_(0, 3);
      transformStamped.transform.translation.y = cur_pose_(1, 3);
      transformStamped.transform.translation.z = 1.9;
      Eigen::Matrix3d q = cur_pose_.block<3, 3>(0, 0);
      Eigen::Quaterniond quat(q);
      quat.normalize();
      transformStamped.transform.rotation.x = quat.x();
      transformStamped.transform.rotation.y = quat.y();
      transformStamped.transform.rotation.z = quat.z();
      transformStamped.transform.rotation.w = quat.w();
      tf2_ros::TransformBroadcaster transformBroadcaster(node_);
      transformBroadcaster.sendTransform(transformStamped);
    }

    cur_markers = getMarkers(cur_frame);
    if (cur_markers.markers.size() > 0) {
      marker_pub_->publish(cur_markers);
    }
  }
}

visualization_msgs::msg::MarkerArray MapOptimization::getMarkers(
    const std::vector<Observation>& observations) {
  visualization_msgs::msg::MarkerArray markers;
  size_t id = 0;
  for (auto obs = observations.begin(); obs != observations.end(); ++obs) {
    Eigen::Vector3d cur_pose = (cur_pose_ * obs->point.homogeneous()).head<3>();
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = node_->now();
    marker.lifetime.sec = 0;
    marker.lifetime.nanosec = 100000000;
    marker.id = id++;
    marker.color.a = 0.7;
    marker.color.r = 35.0 / 255;
    marker.color.g = 204.0 / 255;
    marker.color.b = 150.0 / 255;
    marker.scale.x = 0.07;
    marker.scale.y = 0.07;
    marker.scale.z = 1;
    marker.pose.position.x = cur_pose.x();
    marker.pose.position.y = cur_pose.y();
    marker.pose.position.z = 0;
    marker.type = visualization_msgs::msg::Marker::CYLINDER;
    marker.action = visualization_msgs::msg::Marker::ADD;
    markers.markers.push_back(marker);

    visualization_msgs::msg::Marker marker_line;
    marker_line.header.frame_id = "map";
    marker_line.header.stamp = node_->now();
    marker_line.lifetime.sec = 0;
    marker_line.lifetime.nanosec = 100000000;
    marker_line.id = id++;
    marker_line.color.a = 1;
    marker_line.color.r = 65.0 / 255;
    marker_line.color.g = 124.0 / 255;
    marker_line.color.b = 150.0 / 255;
    marker_line.scale.x = 0.03;
    marker_line.scale.y = 0.03;
    marker_line.action = visualization_msgs::msg::Marker::ADD;
    marker_line.type = visualization_msgs::msg::Marker::LINE_STRIP;
    geometry_msgs::msg::Point p;
    p.x = cur_pose.x();
    p.y = cur_pose.y();
    p.z = 0;
    geometry_msgs::msg::Point p2;
    p2.x = cur_pose_(0, 3);
    p2.y = cur_pose_(1, 3);
    p2.z = 0;
    marker_line.points.push_back(p2);
    marker_line.points.push_back(p);
    markers.markers.push_back(marker_line);
  }
  return markers;
}

void MapOptimization::reset_optimized_map() {
  optimized_map_.clear();
  for (auto const& [id, frame] : keyframes) {
    optimized_map_[id] = frame.pose;
  }
}

void MapOptimization::reset_optimized_reflectors() {
  optimized_reflectors_.clear();
  for (auto const& [id, ref] : map) {
    optimized_reflectors_[id] = ref.position;
  }
}

void MapOptimization::optimize_thread() {
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lock(optimize_mutex);
    cv.wait_for(lock, std::chrono::seconds(1));
    auto st = std::chrono::steady_clock::now();
    std::unordered_map<int, double*> pose_params;  // 关键帧的位姿
    std::unordered_map<int, std::pair<double*, double*>>
        odom_params;  // 里程计的位姿
    std::unique_lock<std::mutex> key_lock(key_mutex);
    if (keyframes.size() < 2) continue;

    bool all_opt{false};  // 全局优化
    if (true) {
      all_opt = true;
    }
    // if (last_opt_size_ == (int)keyframes.size()) all_opt = true;
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    int N = keyframes.size();
    int start = std::max(0, N - window_size);
    if (all_opt) {
      start = 0;
    }
    std::unordered_set<int> window_ids;
    std::unordered_set<int> ref_ids;

    for (int i = start; i < N; i++) {
      window_ids.insert(keyframes[i].id);
    }

    for (const auto& [id, pose] : optimized_map_) {
      Eigen::Vector3d pos = pose.block<3, 1>(0, 3);
      Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
      Eigen::Quaterniond quat(R);
      quat.normalize();
      double* params = new double[7]{pos.x(),  pos.y(),  pos.z(), quat.x(),
                                     quat.y(), quat.z(), quat.w()};
      pose_params[id] = params;
      problem.AddParameterBlock(params, 7);
      if (window_ids.find(id) == window_ids.end()) {
        problem.SetParameterBlockConstant(params);
      } else {
        for (auto& ref_id : keyframes[id].observations) {
          if (ref_id.id != -1) {
            ref_ids.insert(ref_id.id);
          }
        }
      }
      // 设置四元数参数化
      ceres::Manifold* pose_manifold = new ceres::ProductManifold(
          new ceres::EuclideanManifold<3>(),    // 平移 tx,ty,tz
          new ceres::EigenQuaternionManifold()  // 四元数 qx,qy,qz,qw
      );
      problem.SetManifold(params, pose_manifold);

      // 固定第一个位姿作为参考系
      if (id == start) {
        problem.SetParameterBlockConstant(params);
      }
    }
    key_lock.unlock();

    std::unordered_map<int, double*> reflector_params;
    std::unique_lock<std::mutex> ref_lock(ref_mutex);
    if (all_opt) {
      ref_ids.clear();
      for (auto& x : map) {
        ref_ids.insert(x.first);
      }
    }
    for (const auto& [id, pos] : optimized_reflectors_) {
      double* params = new double[3]{pos.x(), pos.y(), pos.z()};
      reflector_params[id] = params;
      problem.AddParameterBlock(params, 3);
      if (ref_ids.find(id) == ref_ids.end()) {
        problem.SetParameterBlockConstant(params);
      }
    }
    ref_lock.unlock();
    // 观测
    key_lock.lock();
    status.last_keyframe_num = keyframes.size();
    for (auto const& [id, frame] : keyframes) {
      if (window_ids.find(id) == window_ids.end()) continue;
      for (auto const& obs : frame.observations) {
        if (obs.id == -1) continue;
        if (ref_ids.find(obs.id) == ref_ids.end()) continue;
        auto cost = ObservationResidual::Create(obs.point);
        double* p1 = pose_params[id];
        double* p2 = reflector_params[obs.id];
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.1), p1, p2);
      }
    }
    last_opt_size_ = keyframes.size();
    key_lock.unlock();

    for (auto it : odoms) {
      if (window_ids.find(it.last_id) == window_ids.end() ||
          window_ids.find(it.cur_id) == window_ids.end())
        continue;
      Eigen::Matrix4d R = it.last_pose.inverse() * it.current_pose;
      Eigen::Vector3d t = R.block<3, 1>(0, 3);
      Eigen::Matrix3d euler = R.block<3, 3>(0, 0);
      Eigen::Quaterniond q(euler);
      q.normalize();
      Eigen::Matrix<double, 6, 6> info;
      for (int i = 0; i < 6; i++) {
        for (int j = 0; j < 6; j++) {
          info(i, j) = it.info[i][j];
        }
      }
      ceres::CostFunction* cost_function = OdometryResidual::Create(
          t.x(), t.y(), t.z(), q.x(), q.y(), q.z(), q.w(), info);
      Eigen::Vector3d p1 = it.last_pose.block<3, 1>(0, 3);
      Eigen::Vector3d p2 = it.current_pose.block<3, 1>(0, 3);
      Eigen::Quaterniond q1(it.last_pose.block<3, 3>(0, 0));
      q1.normalize();
      Eigen::Quaterniond q2(it.current_pose.block<3, 3>(0, 0));
      q2.normalize();
      double* params_1 =
          new double[7]{p1.x(), p1.y(), p1.z(), q1.x(), q1.y(), q1.z(), q1.w()};
      double* params_2 =
          new double[7]{p2.x(), p2.y(), p2.z(), q2.x(), q2.y(), q2.z(), q2.w()};
      odom_params[odom_params.size()] = {params_1, params_2};
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1),
                               params_1, params_2);
    }
    //

    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.num_threads = 4;
    options.max_num_iterations = 50;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.minimizer_progress_to_stdout = false;
    options.function_tolerance = 1e-8;
    ceres::Solve(options, &problem, &summary);
    for (const auto& [id, params] : pose_params) {
      if (optimized_map_.find(id) != optimized_map_.end()) {
        auto& pose = optimized_map_[id];
        pose(0, 3) = params[0];
        pose(1, 3) = params[1];
        pose(2, 3) = params[2];

        Eigen::Quaterniond q(params[6], params[3], params[4], params[5]);
        q.normalize();
        pose.block(0, 0, 3, 3) = q.toRotationMatrix();
      }
      delete[] params;
    }

    for (const auto& [id, params] : reflector_params) {
      if (optimized_reflectors_.find(id) != optimized_reflectors_.end()) {
        auto& reflector = optimized_reflectors_[id];
        reflector.x() = params[0];
        reflector.y() = params[1];
        reflector.z() = params[2];
      }
      delete[] params;
    }
    if (status.first_optimize) {
      status.first_optimize = false;
    }
    status.last_optimize_time = std::chrono::steady_clock::now();
    auto ed = std::chrono::steady_clock::now();
    RCLCPP_INFO(
        node_->get_logger(), "Optimization finished  %d %ldms.", all_opt,
        std::chrono::duration_cast<std::chrono::milliseconds>(ed - st).count());
    map_cv.notify_one();
  }
}
}  // namespace reflector_slam