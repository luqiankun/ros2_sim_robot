#ifndef MOVE_H
#define MOVE_H
#include <can_msgs/msg/frame.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
const std::string can_tx_topic = "/can0_tx";
const std::string can_rx_topic = "/can0_rx";
const uint RPDO4 = 0x500;
const uint RPDO3 = 0x400;
struct Chassis {
  float wheel_radius;  // 轮子半径，单位：mm
  float x;             // 舵轮离车辆坐标系的x坐标,单位: mm
  float y;             // 舵轮离车辆坐标系的y坐标,单位: mm
  float encoder_resolution{1};
  float angle_encoder_resolution{1};
  uint vel_can_node_id{1};
  uint angle_can_node_id{2};
};
class MoveCtrl {
 public:
  MoveCtrl(rclcpp::Node::SharedPtr node, const Chassis& chassis);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);

 private:
  const Chassis& m_chassis;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};
class Odomtry {
 public:
  Odomtry(rclcpp::Node::SharedPtr node, const Chassis& chassis);
  void can_callback();

 private:
  const Chassis& m_chassis;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
};
#endif