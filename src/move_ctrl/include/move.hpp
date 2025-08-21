#ifndef MOVE_H
#define MOVE_H
#include <tf2_ros/transform_broadcaster.h>

#include <can_msgs/msg/frame.hpp>
#include <custom_interfaces/action/fork_move.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include "rclcpp_action/rclcpp_action.hpp"
const std::string can_tx_topic = "/can0_tx";
const std::string can_rx_topic = "/can0_rx";
const uint RPDO4 = 0x500;
const uint RPDO3 = 0x400;
const uint TPDO3 = 0x380;
struct Chassis {
  float wheel_radius;  // 轮子半径，单位：mm
  float x;             // 舵轮离车辆坐标系的x坐标,单位: mm
  float y;             // 舵轮离车辆坐标系的y坐标,单位: mm
  float encoder_resolution{1};
  float angle_encoder_resolution{1};
  uint vel_can_node_id{1};
  uint angle_can_node_id{3};
  uint fork_can_node_id{2};
};
class MoveCtrl {
 public:
  MoveCtrl(rclcpp::Node::SharedPtr node, const Chassis& chassis);
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
  std::function<float()> get_wheel_vel;
  std::function<float()> get_wheel_angle;

 private:
  const Chassis& m_chassis;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub;
};
class Odomtry {
 public:
  Odomtry(rclcpp::Node::SharedPtr node, const Chassis& chassis);
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg);
  void timer_callback();
  float get_wheel_vel() const { return wheel_vel; }
  float get_wheel_angle() const { return wheel_angle; }
  float get_fork_vel() const { return fork_vel; }
  float get_fork_hight() const { return fork_hight; }

 private:
  float wheel_vel{0};
  float wheel_angle{0};
  float m_x{0};
  float m_y{0};
  float m_theta{0};
  float fork_hight{0};
  float fork_vel{0};
  const Chassis& m_chassis;
  rclcpp::Time last_time;
  bool first_odometry{true};
  rclcpp::Node::SharedPtr node;
  tf2_ros::TransformBroadcaster tf_broadcaster;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub;
  rclcpp::TimerBase::SharedPtr timer;
};

class ForkCtrl {
 public:
  using ForkMove = custom_interfaces::action::ForkMove;
  using GoalHandleForkMove = rclcpp_action::ServerGoalHandle<ForkMove>;
  ForkCtrl(rclcpp::Node::SharedPtr node, const Chassis& chassis);
  std::function<float()> get_fork_vel;
  std::function<float()> get_fork_hight;
  void fork_ctrl_callback(const std_msgs::msg::Float64::SharedPtr msg);
  void execute_callback(const std::shared_ptr<GoalHandleForkMove> goal_handle);

 private:
  const Chassis& m_chassis;
  rclcpp::Node::SharedPtr node;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr fork_vel_sub;
  rclcpp_action::Server<ForkMove>::SharedPtr fork_move_server;
};
// RCLCPP_COMPONENTS_REGISTER_NODE(ForkCtrl)

#endif