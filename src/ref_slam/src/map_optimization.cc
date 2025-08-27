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
  odom_sub_ = node_->create_subscription<nav_msgs::msg::Odometry>(
      "odom", 10,
      std::bind(&MapOptimization::odomCallback, this, std::placeholders::_1));
  laser_sub_ = node_->create_subscription<sensor_msgs::msg::LaserScan>(
      "lidar", 10,
      std::bind(&MapOptimization::laserCallback, this, std::placeholders::_1));
  marker_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(
      "reflector_markers", 10);
  reflector_extractor_ = std::make_shared<FeatureExtractor>();
  map.clear();
  optimize_thread_ = std::thread(&MapOptimization::optimize_thread, this);
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

void MapOptimization::laserCallback(
    const sensor_msgs::msg::LaserScan::SharedPtr msg) {
  auto cur_frame = reflector_extractor_->extract(msg);
  //   for (auto& x : cur_markers.markers) {
  //     x.action = visualization_msgs::msg::Marker::DELETE;
  //   }
  //   marker_pub_->publish(cur_markers);
  //   for (auto& x : cur_frame) {
  //     Eigen::Vector3d p = (odom_pose * x.point.homogeneous()).head<3>();
  //     // std::cout << p.transpose() << std::endl;
  //   }
  //   RCLCPP_INFO(node_->get_logger(), "cur_frame: %ld", cur_frame.size());
  std::unique_lock<std::mutex> lock(ref_mutex);
  if (map.empty()) {
    for (size_t i = 0; i < cur_frame.size(); ++i) {
      Reflector ref;
      ref.id = map.size();
      ref.position = cur_frame[i].point;
      map[ref.id] = ref;
    }
    Keyframe frame;
    frame.id = keyframes.size();
    frame.pose = cur_pose_;
    frame.observations = cur_frame;
    frame.timestamp = std::chrono::steady_clock::now();
    keyframes[frame.id] = frame;
    lock.unlock();
  } else {
    Eigen::Matrix4d cur_pose =
        reflector_extractor_->match(cur_frame, map, cur_pose_);
    RCLCPP_INFO_STREAM(node_->get_logger(),
                       cur_pose(0, 3) << " " << cur_pose(1, 3) << std::endl);
    Eigen::Matrix4d odom = keyframes[keyframes.size() - 1].pose.inverse() *
                           cur_pose;  // T_cur_last
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
          if ((x.second.position - pose).norm() < 0.3) {
            exit = true;
            break;
          }
        }
        if (!exit) {
          Reflector ref;
          ref.id = map.size();
          ref.position = (cur_pose_ * x.point.homogeneous()).head<3>();
          map[ref.id] = ref;
          reset_optimized_reflectors();
        }
      }
    }
    lock.unlock();
    if (t.norm() > 1 || fabs(angle) > M_PI / 6) {
      // 大于1米
      Keyframe frame;
      frame.id = keyframes.size();
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
      reset_optimized_map();
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
  while (true) {
    std::unique_lock<std::mutex> lock(optimize_mutex);
    cv.wait_for(lock, std::chrono::seconds(1),
                [&] { return need_optimized(); });
    if (!need_optimized()) {
      continue;
    }
    std::unordered_map<int, double*> pose_params;  // 关键帧的位姿
    std::unique_lock<std::mutex> key_lock(key_mutex);
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    for (const auto& [id, pose] : optimized_map_) {
      Eigen::Vector3d pos = pose.block<3, 1>(0, 3);
      Eigen::Matrix3d R = pose.block<3, 3>(0, 0);
      Eigen::Quaterniond quat(R);
      quat.normalize();
      double* params = new double[7]{pos.x(),  pos.y(),  pos.z(), quat.x(),
                                     quat.y(), quat.z(), quat.w()};
      pose_params[id] = params;
      problem.AddParameterBlock(params, 7);
      // 设置四元数参数化
      // ceres::Manifold* quat_manifold = new ceres::EigenQuaternionManifold();
      // problem.SetManifold(params, quat_manifold);

      // 固定第一个位姿作为参考系
      if (id == 0) {
        problem.SetParameterBlockConstant(params);
      }
    }
    key_lock.unlock();
    std::unordered_map<int, double*> reflector_params;
    std::unique_lock<std::mutex> ref_lock(ref_mutex);
    for (const auto& [id, pos] : optimized_reflectors_) {
      double* params = new double[3]{pos.x(), pos.y(), pos.z()};
      reflector_params[id] = params;
      problem.AddParameterBlock(params, 3);
    }
    ref_lock.unlock();
    // 观测
    key_lock.lock();
    status.last_keyframe_num = keyframes.size();
    for (auto const& [id, frame] : keyframes) {
      for (auto const& obs : frame.observations) {
        if (obs.id == -1) continue;
        auto cost = ObservationResidual::Create(obs.point);
        double* p1 = pose_params[id];
        double* p2 = reflector_params[obs.id];
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.1), p1, p2);
      }
    }
    key_lock.unlock();

    for (auto it : odoms) {
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
      double params_1[7] = {p1.x(), p1.y(), p1.z(), q1.x(),
                            q1.y(), q1.z(), q1.w()};
      double params_2[7] = {p2.x(), p2.y(), p2.z(), q2.x(),
                            q2.y(), q2.z(), q2.w()};
      problem.AddResidualBlock(cost_function, new ceres::HuberLoss(0.1),
                               params_1, params_2);
    }
    //

    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.num_threads = 4;
    options.max_num_iterations = 50;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.minimizer_progress_to_stdout = true;
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
    RCLCPP_INFO(node_->get_logger(), "Optimization finished.");
  }
}

bool MapOptimization::need_optimized() {
  std::unique_lock<std::mutex> lock(key_mutex);
  if (status.first_optimize) {
    if (keyframes.size() > 5) {
      return true;
    } else {
      return false;
    }
  }
  auto now = std::chrono::steady_clock::now();
  auto dur = std::chrono::duration_cast<std::chrono::seconds>(
      (now - status.last_optimize_time));
  if (dur.count() > 10) {
    return true;
  } else if ((keyframes.size() - status.last_keyframe_num) > 5) {
    return true;
  } else {
    return false;
  }
}

}  // namespace reflector_slam