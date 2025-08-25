#include "../include/map_optimization.hpp"

#include <tf2_ros/transform_broadcaster.h>

#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>
namespace reflector_slam {

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
  map.push_back(Reflector{Eigen::Vector3d(27.0809, 5.02901, 0), 0});
  map.push_back(Reflector{Eigen::Vector3d(27.0639, 22.6547, 0), 2});
  map.push_back(Reflector{Eigen::Vector3d(26.6122, -12.7367, 0), 3});
  map.push_back(Reflector{Eigen::Vector3d(2.76219, -4.6443, 0), 4});
  map.push_back(Reflector{Eigen::Vector3d(0.766346, -6.04852, 0), 5});
  map.push_back(Reflector{Eigen::Vector3d(2.99862, 15.3888, 0), 6});
  map.push_back(Reflector{Eigen::Vector3d(0.999057, 13.6581, 0), 7});
  map.push_back(Reflector{Eigen::Vector3d(-5.72091, 8.53991, 0), 8});
  map.push_back(Reflector{Eigen::Vector3d(2.75094, -24.71, 0), 9});
  map.push_back(Reflector{Eigen::Vector3d(1.05172, -27.0912, 0), 10});
  map.push_back(Reflector{Eigen::Vector3d(-4.70903, -17.373, 0), 11});
  map.push_back(Reflector{Eigen::Vector3d(21.2085, -27.1435, 0), 12});
  map.push_back(Reflector{Eigen::Vector3d(9.74532, -38.6915, 0), 13});
  map.push_back(Reflector{Eigen::Vector3d(-3.4929, -38.8038, 0), 14});
  map.push_back(Reflector{Eigen::Vector3d(-27.6341, 27.6691, 0), 15});
  map.push_back(Reflector{Eigen::Vector3d(-28.3731, 15.9639, 0), 16});
  map.push_back(Reflector{Eigen::Vector3d(16.6716, 37.5803, 0), 17});
  map.push_back(Reflector{Eigen::Vector3d(4.88702, 37.6613, 0), 18});
  map.push_back(Reflector{Eigen::Vector3d(-14.5391, 37.9663, 0), 19});
  map.push_back(Reflector{Eigen::Vector3d(-19.59, 8.26576, 0), 20});
  map.push_back(Reflector{Eigen::Vector3d(-34.4213, 27.5766, 0), 21});
  map.push_back(Reflector{Eigen::Vector3d(-34.4034, 16.0788, 0), 22});
  map.push_back(Reflector{Eigen::Vector3d(-41.3103, 37.7089, 0), 23});
  map.push_back(Reflector{Eigen::Vector3d(-34.3343, 8.04166, 0), 24});
  map.push_back(Reflector{Eigen::Vector3d(-34.3923, -2.58821, 0), 25});
  map.push_back(Reflector{Eigen::Vector3d(-34.4771, 15.3802, 0), 26});
  map.push_back(Reflector{Eigen::Vector3d(-34.5775, -29.2945, 0), 27});
  map.clear();
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
  transform.block<3, 3>(0, 0) = q.toRotationMatrix();
  odom_pose = lidar_to_base.inverse() * transform * lidar_to_base;
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
  if (map.empty()) {
    for (size_t i = 0; i < cur_frame.size(); ++i) {
      Reflector ref;
      ref.id = map.size();
      ref.position = cur_frame[i].point;
      map.push_back(ref);
    }
    Keyframe frame;
    frame.id = 0;
    frame.pose = cur_pose_;
    frame.observations = cur_frame;
    frame.timestamp = std::chrono::steady_clock::now();
    keyframes.push_back(frame);
  } else {
    auto conv = reflector_extractor_->match(cur_frame, map, odom_pose);
    if (conv > 0.1) {
      Eigen::Matrix4d cur_pose =
          reflector_extractor_->pre_pose_estimation(cur_frame, map);
      std::cout << cur_pose << std::endl;

      Eigen::Vector3d cur_vec = cur_pose.block<3, 1>(0, 3);
      Eigen::Vector3d last_vec = cur_pose_.block<3, 1>(0, 3);
      double diff = (cur_vec - last_vec).norm();

      double angle =
          acos((cur_vec.dot(last_vec)) / (cur_vec.norm() * last_vec.norm()));

      if (diff > 1 || fabs(angle) > 0.2) {
        // 大于1米
        Keyframe frame;
        frame.id = keyframes.size();
        frame.pose = cur_pose;
        frame.observations = cur_frame;
        frame.timestamp = std::chrono::steady_clock::now();
        keyframes.push_back(frame);
      }
      cur_pose_ = cur_pose;
      for (auto x : cur_frame) {
        if (x.id == -1) {
          Eigen::Vector3d pose = (cur_pose_ * x.point.homogeneous()).head<3>();
          bool exit = false;
          for (auto& x : map) {
            if ((x.position - pose).norm() < 0.3) {
              exit = true;
              break;
            }
          }
          if (!exit) {
            Reflector ref;
            ref.id = map.size();
            ref.position = (cur_pose_ * x.point.homogeneous()).head<3>();
            map.push_back(ref);
          }
        }
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
        transformStamped.transform.rotation.x = quat.x();
        transformStamped.transform.rotation.y = quat.y();
        transformStamped.transform.rotation.z = quat.z();
        transformStamped.transform.rotation.w = quat.w();
        tf2_ros::TransformBroadcaster transformBroadcaster(node_);
        transformBroadcaster.sendTransform(transformStamped);
      }
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

}  // namespace reflector_slam