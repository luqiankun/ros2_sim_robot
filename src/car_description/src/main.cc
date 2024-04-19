// Copyright 2021 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

class PIDController : public rclcpp::Node {
 public:
  PIDController(const std::string& name) : rclcpp::Node(name) {
    declare_parameter("Kp", 0.6);
    declare_parameter("Ki", 0.0);
    declare_parameter("Kd", 0.0);
    param_monitor = std::make_shared<rclcpp::ParameterEventHandler>(this);
    tf_odom = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    command_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10);
    turn_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
    feedback_publisher =
        create_publisher<std_msgs::msg::Float64>("/feedback_vel", 10);
    cmd_subscriber = this->create_subscription<std_msgs::msg::Float64>(
        "/cmd", 1, [&](std_msgs::msg::Float64::ConstSharedPtr speed) {
          this->speed = speed->data;
        });
    joint_state_subscriber =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1,
            std::bind(&PIDController::feedback, this, std::placeholders::_1));
    auto cb_handle_ = param_monitor->add_parameter_callback(
        "Kp", [&](const rclcpp::Parameter& p) { Kp = p.as_double(); });
    params.push_back(cb_handle_);
    cb_handle_ = param_monitor->add_parameter_callback(
        "Ki", [&](const rclcpp::Parameter& p) { Ki = p.as_double(); });
    params.push_back(cb_handle_);
    cb_handle_ = param_monitor->add_parameter_callback(
        "Kd", [&](const rclcpp::Parameter& p) { Kd = p.as_double(); });
    params.push_back(cb_handle_);
  }
  void run() {
    timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&PIDController::process, this));
  }
  void process();
  void feedback(sensor_msgs::msg::JointState::ConstSharedPtr msg);

 private:
  bool first{true};
  double speed{0};
  double feedback_speed{0};
  double odom_x{0};
  double odom_y{0};
  double Kp{0.06};
  double Ki{1.5};
  double Kd{0.5};
  std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> params;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_monitor;
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      command_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr turn_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_publisher;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr cmd_subscriber;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom;
  std::chrono::system_clock::time_point last_feed_time;
};

void PIDController::feedback(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
  if (first) {
    first = false;
    last_feed_time = std::chrono::system_clock::now();
  } else {
    geometry_msgs::msg::TransformStamped t;
    t.header.set__frame_id("odom");
    t.header.stamp = msg->header.stamp;
    // RCLCPP_INFO_STREAM(node->get_logger(),
    //                    node->get_clock()->now().seconds());
    t.child_frame_id.append("base");
    auto v = msg->velocity.at(0);
    feedback_speed = v * 0.25;
    std_msgs::msg::Float64 res;
    res.data = feedback_speed;
    feedback_publisher->publish(res);
    auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now() - last_feed_time);
    last_feed_time = std::chrono::system_clock::now();
    auto V = v * 0.25;
    odom_x += V * dt.count() / 1000;
    t.transform.translation.set__x(odom_x);
    t.transform.translation.set__y(odom_y);
    t.transform.rotation.set__w(1);
    t.transform.rotation.set__x(0);
    t.transform.rotation.set__y(0);
    t.transform.rotation.set__z(0);

    // RCLCPP_INFO_STREAM(node->get_logger(), "V " << V << " x " <<
    // x);
    tf_odom->sendTransform(t);
  }
}

void PIDController::process() {
  // pid
  static double ek_2{0};
  static double ek_1{0};
  double ek{0};
  static double uk{0};
  ek = speed - feedback_speed;
  double delta_uk = (Kp * (ek - ek_1)) + Ki * ek + Kd * (ek - 2 * ek_1 + ek_2);
  ek_2 = ek_1;
  ek_1 = ek;
  uk += delta_uk;
  if (uk > 5) {
    uk = 5;
  }
  if (uk < -5) {
    uk = -5;
  }
  // RCLCPP_INFO(get_logger(), "err: %f duk: %f kp: %f ki: %f kd: %f uk: %f",
  // ek,
  //             delta_uk, Kp, Ki, Kd, uk);
  std_msgs::msg::Float64MultiArray commands;
  commands.data.push_back(uk);
  commands.data.push_back(uk);
  commands.data.push_back(uk);
  commands.data.push_back(uk);
  command_publisher->publish(commands);
  {
    std_msgs::msg::Float64MultiArray turns;
    std_msgs::msg::MultiArrayDimension dim;
    dim.size = 2;
    turns.layout.dim.push_back(dim);
    turns.data.push_back(0.3);
    turns.data.push_back(0.3);
    turn_publisher->publish(turns);
  }
}
int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PIDController>("cmd_vel_demo");
  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}