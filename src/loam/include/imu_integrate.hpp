#ifndef IMU_INTEGRATE_HPP
#define IMU_INTEGRATE_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <deque>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>

namespace loam {
struct IMUData {
  double timestamp;
  double gyro_z;
  double acc_x;
  double acc_y;
};

class ImuIntegrate {
 public:
  ImuIntegrate(rclcpp::Node::SharedPtr node) : node_(node) {
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu", 100,
        std::bind(&ImuIntegrate::imuCallback, this, std::placeholders::_1));
  }

  Eigen::Isometry2d get_delta_pose(double start_time, double end_time);

  ~ImuIntegrate() = default;
  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);

  std::deque<IMUData> imu_buffer_;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  double bias_gyro_{0.0};
  double bias_acc_{0.0};  // 简化处理，假设x/y相同
  double max_buffer_time_{2.0};
};

inline void ImuIntegrate::imuCallback(
    const sensor_msgs::msg::Imu::SharedPtr msg) {
  IMUData imu_data;
  imu_data.timestamp = msg->header.stamp.sec + msg->header.stamp.nanosec / 1e9;
  imu_data.gyro_z = msg->angular_velocity.z;
  imu_data.acc_x = msg->linear_acceleration.x;
  imu_data.acc_y = msg->linear_acceleration.y;
  imu_buffer_.push_back(imu_data);
  while (imu_buffer_.back().timestamp - imu_buffer_.front().timestamp >
         max_buffer_time_) {
    imu_buffer_.pop_front();
  }
}

inline Eigen::Isometry2d ImuIntegrate::get_delta_pose(double start_time,
                                                      double end_time) {
  std::vector<IMUData> imu_segment;
  for (auto& imu : imu_buffer_) {
    if (imu.timestamp >= start_time && imu.timestamp <= end_time) {
      imu_segment.push_back(imu);
    }
  }

  if (imu_segment.size() < 2) {
    RCLCPP_WARN(node_->get_logger(), "imu_segment too small");
    return Eigen::Isometry2d::Identity();
  }

  double theta = 0.0;
  Eigen::Vector2d delta_p = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();

  for (size_t i = 1; i < imu_segment.size(); i++) {
    double dt = imu_segment[i].timestamp - imu_segment[i - 1].timestamp;

    // 积分角度
    theta += (imu_segment[i].gyro_z - bias_gyro_) * dt;

    // 加速度
    double ax = imu_segment[i].acc_x - bias_acc_;
    double ay = imu_segment[i].acc_y - bias_acc_;
    Eigen::Vector2d acc_body(ax, ay);

    // 旋转到世界系
    Eigen::Rotation2Dd R(theta);
    Eigen::Vector2d acc_world = R * acc_body;

    // 积分速度 & 位置
    velocity += acc_world * dt;
    delta_p += velocity * dt + 0.5 * acc_world * dt * dt;
  }

  // 构造 Isometry2d
  Eigen::Isometry2d T = Eigen::Isometry2d::Identity();
  T.linear() = Eigen::Rotation2Dd(theta).toRotationMatrix();
  T.translation() = delta_p;
  return T;
}

}  // namespace loam

#endif
