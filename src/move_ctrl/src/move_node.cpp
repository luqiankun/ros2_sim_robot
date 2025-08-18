#include <cstdio>
#include <eigen3/Eigen/Eigen>

#include "../include/move.hpp"
MoveCtrl::MoveCtrl(rclcpp::Node::SharedPtr node, const Chassis& chassis)
    : m_chassis{chassis}, node{node} {
  cmd_vel_sub = node->create_subscription<geometry_msgs::msg::Twist>(
      "cmd_vel", 10,
      std::bind(&MoveCtrl::cmd_vel_callback, this, std::placeholders::_1));
  can_pub = node->create_publisher<can_msgs::msg::Frame>(can_tx_topic, 10);
}

void MoveCtrl::cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
  //
  float rpm = 0;
  float angle = 0;
  //
  float vel = msg->linear.x;
  float omega = msg->angular.z;
  float radius = vel / (omega + 1e-8);

  if (fabs(vel) < 1e-3) {
    if (fabs(omega) < 1e-3) {
      // 停止状态
      rpm = 0.0;
      angle = 0.0;
    } else {
      // 原地旋转
      Eigen::Vector2f wheel_radius{m_chassis.x, m_chassis.y};
      Eigen::Rotation2Df rot = Eigen::Rotation2Df(M_PI / 2);
      Eigen::Vector2f wheel_vel = rot * wheel_radius;
      float cos_angle = atan2(wheel_vel.y(), wheel_vel.x());
      float vel = fabs(omega * wheel_radius.norm());
      rpm = vel / m_chassis.encoder_resolution / 2.0 / M_PI /
            m_chassis.wheel_radius * 60.0;  // m_spped;
      angle = cos_angle / m_chassis.angle_encoder_resolution;
      if (omega < 0) {
        rpm *= -1;
      }
    }
  } else {
    // 普通
    Eigen::Vector2f normal_vel{1, 0};
    normal_vel.normalize();
    Eigen::Vector2f Radius{0, 1};
    Eigen::Rotation2Df rot;
    if (omega < 0.0) {
      // 右转
      rot = Eigen::Rotation2Df(-M_PI / 2);
      Radius = rot * normal_vel;
    } else {
      // 左转
      rot = Eigen::Rotation2Df(M_PI / 2);
      Radius = rot * normal_vel;
    }
    if (Radius.norm() < sqrt(pow(m_chassis.x, 2) + pow(m_chassis.y, 2))) {
      Radius =
          Radius.normalized() * sqrt(pow(m_chassis.x, 2) + pow(m_chassis.y, 2));
    }
    Radius.normalize();
    Radius *= radius;
    Eigen::Vector<float, 2> wheel_pos{m_chassis.x, m_chassis.y};
    Eigen::Vector<float, 2> RadiusWheel = Radius - wheel_pos;
    Eigen::Vector<float, 2> normalWheel = rot.inverse() * RadiusWheel;
    normalWheel = normalWheel.normalized();
    float cos_angle = (normalWheel.dot(normal_vel)) /
                      ((normalWheel.norm() * normal_vel.norm()) + 1e-8);
    float wheel_angle = fabs(acos(cos_angle));
    float wheel_vel = (omega + 1e-8) * RadiusWheel.norm();
    if (wheel_angle > M_PI / 2) {
      normalWheel *= -1;
      wheel_vel *= -1;
    }
    wheel_angle = std::atan(normalWheel[1] / normalWheel[0]);
    rpm = wheel_vel / m_chassis.encoder_resolution / 2.0 / M_PI /
          m_chassis.wheel_radius * 60.0;  // m_spped;
    angle = wheel_angle / m_chassis.angle_encoder_resolution;
  }

  can_msgs::msg::Frame vel_msg;
  vel_msg.set__dlc(4);
  vel_msg.set__id(RPDO4 + m_chassis.vel_can_node_id);
  memcpy(vel_msg.data.data(), (void*)&rpm, 4);
  can_msgs::msg::Frame pos_msg;
  pos_msg.set__dlc(4);
  pos_msg.set__id(RPDO3 + m_chassis.angle_can_node_id);
  memcpy(pos_msg.data.data(), (void*)&angle, 4);
  can_pub->publish(vel_msg);
  can_pub->publish(pos_msg);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test");
  Chassis m_chassis{0.105, 1.01, 0, 1, M_PI * 2, 0, 2};
  MoveCtrl move_ctrl(node, m_chassis);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
