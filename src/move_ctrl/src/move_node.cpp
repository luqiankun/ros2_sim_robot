#include <cstdio>
#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2/convert.hpp>

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
    normal_vel = normal_vel.normalized() * vel;
    Eigen::Vector2f Radius{0, 1};
    Radius = Eigen::Rotation2Df(M_PI / 2) * normal_vel;
    Radius = Radius.normalized() * radius;
    if (Radius.norm() < sqrt(pow(m_chassis.x, 2) + pow(m_chassis.y, 2))) {
      Radius =
          Radius.normalized() * sqrt(pow(m_chassis.x, 2) + pow(m_chassis.y, 2));
    }
    Eigen::Vector<float, 2> wheel_pos{m_chassis.x, m_chassis.y};
    Eigen::Vector<float, 2> RadiusWheel = Radius - wheel_pos;
    Eigen::Vector<float, 2> normalWheel =
        Eigen::Rotation2Df(-M_PI / 2) * RadiusWheel;
    normalWheel = (normalWheel).normalized();
    float cos_angle = (normalWheel.dot(normal_vel)) /
                      ((normalWheel.norm() * normal_vel.norm()) + 1e-8);
    float wheel_angle = fabs(acos(cos_angle));
    float wheel_vel = fabs((omega + 1e-8) * RadiusWheel.norm());
    RCLCPP_INFO_STREAM(node->get_logger(),
                       Radius << " " << RadiusWheel << " " << normalWheel << " "
                              << normal_vel << " " << wheel_angle);
    if (wheel_angle > M_PI / 2) {
      // normalWheel *= -1;
      normalWheel = -normalWheel;
    }
    wheel_angle = std::atan2(normalWheel[1], normalWheel[0]);  // -pi ~pi
    // 防止舵轮角度超出范围(+-90)
    if (wheel_angle > M_PI / 2) {
      wheel_angle = wheel_angle - M_PI;
      wheel_vel = -wheel_vel;
    } else if (wheel_angle < -M_PI / 2) {
      wheel_angle = wheel_angle + M_PI;
      wheel_vel = -wheel_vel;
    }
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

Odomtry::Odomtry(rclcpp::Node::SharedPtr node, const Chassis& chassis)
    : m_chassis{chassis}, node{node}, tf_broadcaster(node) {
  odom_pub = node->create_publisher<nav_msgs::msg::Odometry>("odom", 10);
  can_sub = node->create_subscription<can_msgs::msg::Frame>(
      can_rx_topic, 10,
      std::bind(&Odomtry::can_callback, this, std::placeholders::_1));
  timer = node->create_wall_timer(std::chrono::milliseconds(100),
                                  std::bind(&Odomtry::timer_callback, this));
}

void Odomtry::timer_callback() {
  float radius = m_chassis.x / (tan(wheel_angle) + 1e-8) + m_chassis.y;
  float wheel_radius = m_chassis.x / (sin(wheel_angle) + 1e-8);
  float omega = wheel_vel / wheel_radius;
  float v = omega * radius;
  if (first_odometry) {
    first_odometry = false;
    last_time = node->now();
  } else {
    auto now = node->now();
    auto duration = (now - last_time).seconds();
    float v_x = v * cos(m_theta);
    float v_y = v * sin(m_theta);
    m_x += v_x * duration;
    m_y += v_y * duration;
    m_theta += omega * duration;
    // RCLCPP_INFO(node->get_logger(), "v_x: %f, v_y: %f", v_x, v_y);
    // RCLCPP_INFO(node->get_logger(), "m_x: %f, m_y: %f", m_x, m_y);
    // RCLCPP_INFO(node->get_logger(), "m_theta: %f", m_theta);
    last_time = now;
    while (m_theta > M_PI) m_theta -= 2 * M_PI;
    while (m_theta <= -M_PI) m_theta += 2 * M_PI;
    geometry_msgs::msg::Quaternion quat;
    tf2::Quaternion quat_tf;
    quat_tf.setRPY(0, 0, m_theta);
    quat.set__w(quat_tf.w());
    quat.set__x(quat_tf.x());
    quat.set__y(quat_tf.y());
    quat.set__z(quat_tf.z());
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.header.stamp = node->now();
    odom.pose.pose.position.x = m_x;
    odom.pose.pose.position.y = m_y;
    odom.pose.pose.position.z = 0;
    odom.pose.pose.orientation = quat;
    odom.twist.twist.linear.x = v_x;
    odom.twist.twist.linear.y = v_y;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = omega;
    odom_pub->publish(odom);
    {
      geometry_msgs::msg::TransformStamped transformStamped;
      transformStamped.header.stamp = node->now();
      transformStamped.header.frame_id = "odom";
      transformStamped.child_frame_id = "base_link";
      transformStamped.transform.translation.x = m_x;
      transformStamped.transform.translation.y = m_y;
      transformStamped.transform.translation.z = 0.0;
      transformStamped.transform.rotation = quat;
      tf_broadcaster.sendTransform(transformStamped);
    }
  }
}

void Odomtry::can_callback(const can_msgs::msg::Frame::SharedPtr msg) {
  if (msg->id == TPDO3 + m_chassis.angle_can_node_id) {
    // angle
    float angle = 0;
    memcpy((void*)&angle, msg->data.data() + 4, 4);
    angle = angle * m_chassis.angle_encoder_resolution;
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle <= -M_PI) angle += 2 * M_PI;
    wheel_angle = angle;
  } else if (msg->id == TPDO3 + m_chassis.vel_can_node_id) {
    // vel
    float vel = 0;
    memcpy((void*)&vel, msg->data.data(), 4);
    vel = vel * 2 * M_PI / 60 * m_chassis.encoder_resolution *
          m_chassis.wheel_radius;
    wheel_vel = vel;
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("test");
  Chassis m_chassis{0.105, 1.26, 0, 1, M_PI * 2, 1, 3};
  MoveCtrl move_ctrl(node, m_chassis);
  Odomtry odomtry(node, m_chassis);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
