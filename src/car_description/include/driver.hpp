#ifndef __DRIVER_H_
#define __DRIVER_H_
#include <can_msgs/msg/frame.hpp>
#include <controller_manager_msgs/srv/list_controllers.hpp>
#include <controller_manager_msgs/srv/load_controller.hpp>
#include <controller_manager_msgs/srv/switch_controller.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
const std::string position_topic = "/position_controller/commands";
const std::string velocity_topic = "/velocity_controller/commands";
const std::string can_tx_topic = "/can0_tx";
const std::string can_rx_topic = "/can0_rx";

const std::string joint_state_topic = "/joint_states";
const uint TPDO3 = 0x380;
const uint RPDO4 = 0x500;
const uint RPDO3 = 0x400;

///
/// canopen tpdo
/// 0x380+node_id   |velocity  4 bytes|position  4 bytes|
/// canopen rpdo
/// 0x500+node_id  |velocity  4 bytes|
/// 0x400+node_id |position  4 bytes|
class Motor {
 public:
  Motor(rclcpp::Node::SharedPtr node, int cannode_id,
        const std::string& link_name);
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  void can_callback(const can_msgs::msg::Frame::SharedPtr msg);

 public:
  int cannode_id_{0x1};
  float velocity_{0.0};
  float position_{0.0};
  float send_velocity_{0.0};
  float send_position_{0.0};
  int index = -1;
  std::string link_name_;

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_sub_;
  rclcpp::Publisher<can_msgs::msg::Frame>::SharedPtr can_pub_;
  rclcpp::Subscription<can_msgs::msg::Frame>::SharedPtr can_sub_;
};

class Driver {
 public:
  Driver(rclcpp::Node::SharedPtr node);
  void add_motor(int cannode_id, const std::string& link_name);
  void timer_callback();
  void start();
  bool load_controller(const std::string&);
  bool configure_controller(std::string);
  bool switch_controller(std::string, std::string);
  bool is_controller_loaded(std::string);
  bool is_controller_configured(std::string);

 private:
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr
      get_controllers_;
  rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr
      switch_controller_;
  rclcpp::Client<controller_manager_msgs::srv::LoadController>::SharedPtr
      load_controller_;
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_motor_position_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      pub_motor_velocity_;
  rclcpp::TimerBase::SharedPtr pub_timer_;
  std::vector<std::shared_ptr<Motor>> motors_;
  std::map<int, std::string> pose_controller_map;
  std::map<int, std::string> vel_controller_map;
};

#endif