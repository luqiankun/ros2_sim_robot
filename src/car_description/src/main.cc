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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_ros/transform_broadcaster.h>

#include <eigen3/Eigen/Eigen>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
class JointPID {
 public:
  enum class Type { velocity = 0, position = 1 };
  JointPID() = delete;
  JointPID(rclcpp::Node::SharedPtr node, const std::string& name, int index,
           Type t)
      : node(node), type(t), name(name), write_index(index) {
    node->declare_parameter(name + "/Kp", 0.06);
    node->declare_parameter(name + "/Ki", 1.5);
    node->declare_parameter(name + "/Kd", 0.5);
    param_monitor = std::make_shared<rclcpp::ParameterEventHandler>(node);
    auto cb_handle_ = param_monitor->add_parameter_callback(
        name + "/Kp", [&](const rclcpp::Parameter& p) { Kp = p.as_double(); });
    params.push_back(cb_handle_);
    cb_handle_ = param_monitor->add_parameter_callback(
        name + "/Ki", [&](const rclcpp::Parameter& p) { Ki = p.as_double(); });
    params.push_back(cb_handle_);
    cb_handle_ = param_monitor->add_parameter_callback(
        name + "/Kd", [&](const rclcpp::Parameter& p) { Kd = p.as_double(); });
    params.push_back(cb_handle_);
    if (type == Type::velocity) {
      limit = 200;
    } else {
      limit = 1;
    }
  }
  void set_pid(double p, double i, double d) {
    Kp = p;
    Ki = i;
    Kd = d;
  }
  Type get_type() const { return type; }
  int get_index() const { return write_index; }
  void update(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    auto it = std::find(msg->name.begin(), msg->name.end(), name);
    assert(it != msg->name.end());
    int index = std::distance(msg->name.begin(), it);
    if (type == Type::velocity) {
      feedback_value = msg->velocity.at(index);
    } else if (type == Type::position) {
      feedback_value = msg->position.at(index);
    }
  }
  double get_feedback_value() const { return feedback_value; }
  void set_value(double value) { expect_value = value; }
  double process_once() {
    double ek{0};
    ek = expect_value - feedback_value;
    double delta_uk =
        (Kp * (ek - ek_1)) + Ki * ek + Kd * (ek - 2 * ek_1 + ek_2);
    ek_2 = ek_1;
    ek_1 = ek;
    uk += delta_uk;
    if (uk > limit) {
      uk = limit;
    }
    if (uk < -limit) {
      uk = -limit;
    }
    // RCLCPP_INFO(
    //     node->get_logger(),
    //     "name: %s expect:%f err: %f duk: %f kp: %f ki: %f kd: %f uk: %f",
    //     name.c_str(), expect_value, ek, delta_uk, Kp, Ki, Kd, uk);
    return uk;
  }

 private:
  rclcpp::Node::SharedPtr node;
  std::shared_ptr<rclcpp::ParameterEventHandler> param_monitor;
  std::vector<rclcpp::ParameterCallbackHandle::SharedPtr> params;
  double limit{5};
  Type type{0};
  std::string name;
  double Kp{0.6};
  double Ki{0};
  double Kd{0};
  int write_index;
  double expect_value{0};
  double feedback_value{0};
  double ek_2{0};
  double ek_1{0};
  double uk{0};
};

class CarController : public rclcpp::Node {
 public:
  CarController(const std::string& name) : rclcpp::Node(name) {
    tf_odom = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    command_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/velocity_controller/commands", 10);
    odom_publisher = create_publisher<nav_msgs::msg::Odometry>("/odom", 5);
    turn_publisher = create_publisher<std_msgs::msg::Float64MultiArray>(
        "/position_controller/commands", 10);
    clock = create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 1, [&](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
          time = msg->clock;
        });
    feedback_publisher =
        create_publisher<std_msgs::msg::Float64>("/feedback_vel", 10);
    cmd_subscriber = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd", 1, [&](geometry_msgs::msg::Twist::ConstSharedPtr speed) {
          rece_cmd(*speed);
        });
    joint_state_subscriber =
        this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 1,
            std::bind(&CarController::feedback, this, std::placeholders::_1));
  }
  void run() {
    timer = this->create_wall_timer(std::chrono::milliseconds(10),
                                    std::bind(&CarController::process, this));
  }
  void process() {
    std_msgs::msg::Float64MultiArray velocity;
    velocity.data.push_back(0);
    velocity.data.push_back(0);
    // velocity.data.push_back(0);
    // velocity.data.push_back(0);
    std_msgs::msg::Float64MultiArray position;
    position.data.push_back(0);
    position.data.push_back(0);

    for (auto& x : joints) {
      auto uk = x->process_once();
      if (x->get_type() == JointPID::Type::velocity) {
        velocity.data.at(x->get_index()) = uk;
      } else {
        position.data.at(x->get_index()) = uk;
      }
    }

    // test
    // for (auto& x : joints) {
    //   auto uk = x->process_once();
    //   if (x->get_type() == JointPID::Type::velocity && x->get_index() == 2) {
    //     velocity.set__data(std::vector<double>(4, uk));
    //   }
    // }
    //

    command_publisher->publish(velocity);
    turn_publisher->publish(position);
  }
  void feedback(sensor_msgs::msg::JointState::ConstSharedPtr msg) {
    double f1{0}, f2{0};
    double v1{0}, v2{0};
    for (auto& x : joints) {
      x->update(msg);
      if (x->get_type() == JointPID::Type::position) {
        if (x->get_index() == 0) {
          f1 = x->get_feedback_value();
        } else {
          f2 = x->get_feedback_value();
        }
      } else if (x->get_type() == JointPID::Type::velocity) {
        if (x->get_index() == 0) {
          v1 = x->get_feedback_value() * wheel_radius;
        } else if (x->get_index() == 1) {
          v2 = x->get_feedback_value() * wheel_radius;
        }
      }
    }
    double Vr = (v1 + v2) / 2;
    double f_ = (f1 + f2) / 2;
    auto vx = cos(odom_theta) * Vr;
    auto vy = sin(odom_theta) * Vr;
    auto omega = tan(f_) / l * Vr;

    // odom
    if (first) {
      first = false;
      last_feed_time = std::chrono::system_clock::now();
    } else {
      auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::system_clock::now() - last_feed_time)
                    .count();
      last_feed_time = std::chrono::system_clock::now();
      odom_x += vx * dt / 1000;
      odom_y += vy * dt / 1000;
      odom_theta += omega * dt / 1000;
      // RCLCPP_INFO(get_logger(), "x:%f y:%f theta:%f }{vx:%f vy:%f omega:%f",
      //             odom_x, odom_y, odom_theta, vx, vy, omega);
      geometry_msgs::msg::TransformStamped t;

      // Read message content and assign it to
      // corresponding tf variables
      t.header.stamp = time;
      t.header.frame_id = "odom";
      t.child_frame_id = "MR-Buggy3/Base";

      // Turtle only exists in 2D, thus we get x and y translation
      // coordinates from the message and set the z coordinate to 0
      t.transform.translation.x = odom_x;
      t.transform.translation.y = odom_y;
      t.transform.translation.z = 0;

      // For the same reason, turtle can only rotate around one axis
      // and this why we set rotation in x and y to 0 and obtain
      // rotation in z axis from the message
      tf2::Quaternion q;
      q.setRPY(0, 0, odom_theta);
      t.transform.rotation.x = q.x();
      t.transform.rotation.y = q.y();
      t.transform.rotation.z = q.z();
      t.transform.rotation.w = q.w();

      // Send the transformation
      tf_odom->sendTransform(t);
      nav_msgs::msg::Odometry odom;
      odom.header.stamp = time;
      odom.header.frame_id = "odom";
      odom.child_frame_id = "MR-Buggy3/Base";
      odom.pose.pose.position.x = odom_x;
      odom.pose.pose.position.y = odom_y;
      odom.pose.pose.position.z = 0;
      tf2::Quaternion quat;
      quat.setRPY(0, 0, odom_theta);
      odom.pose.pose.orientation.set__w(quat.w());
      odom.pose.pose.orientation.set__x(quat.x());
      odom.pose.pose.orientation.set__y(quat.y());
      odom.pose.pose.orientation.set__z(quat.z());
      odom.twist.twist.linear.x = vx;
      odom.twist.twist.linear.y = vy;
      odom.twist.twist.angular.z = omega;
      odom_publisher->publish(odom);
    }
  }
  void rece_cmd(geometry_msgs::msg::Twist vel) {
    auto v = vel.linear.x;
    auto r = l / (atan(vel.angular.z));
    auto w = v / r;
    auto vl = v - w * d;
    auto vr = v + w * d;
    auto dth = d / l * vel.angular.z * vel.angular.z;
    for (auto& x : joints) {
      if (x->get_type() == JointPID::Type::velocity) {
        if (x->get_index() == 1) {
          x->set_value(vl / wheel_radius);
        } else {
          x->set_value(vr / wheel_radius);
        }
      } else {
        if (x->get_index() == 0) {
          x->set_value(vel.angular.z - dth);
        } else {
          x->set_value(vel.angular.z + dth);
        }
      }
    }
  }

 public:
  std::vector<std::shared_ptr<JointPID>> joints;

 private:
  double d = 0.09;
  double l = 0.224;
  double wheel_radius = 0.0365;
  bool first{true};
  double odom_x{0};
  double odom_y{0};
  double odom_theta{0};
  rclcpp::TimerBase::SharedPtr timer;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr
      command_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr turn_publisher;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr feedback_publisher;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_subscriber;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_subscriber;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_odom;
  std::chrono::system_clock::time_point last_feed_time;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  builtin_interfaces::msg::Time time;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CarController>("cmd_vel_demo");
  auto rear_right = std::make_shared<JointPID>(
      node, "MR-Buggy3/RearRightWheelJoint", 0, JointPID::Type::velocity);
  auto rear_left = std::make_shared<JointPID>(
      node, "MR-Buggy3/RearLeftWheelJoint", 1, JointPID::Type::velocity);
  // auto front_right = std::make_shared<JointPID>(
  //     node, "MR-Buggy3/FrontRightWheelJoint", 2, JointPID::Type::velocity);
  // auto front_left = std::make_shared<JointPID>(
  //     node, "MR-Buggy3/FrontLeftWheelJoint", 3, JointPID::Type::velocity);
  auto front_right_steer =
      std::make_shared<JointPID>(node, "MR-Buggy3/FrontRightWheelSteeringJoint",
                                 0, JointPID::Type::position);
  auto front_left_steer =
      std::make_shared<JointPID>(node, "MR-Buggy3/FrontLeftWheelSteeringJoint",
                                 1, JointPID::Type::position);
  front_right_steer->set_pid(4, 1, 0.01);
  front_left_steer->set_pid(4, 1, 0.01);

  node->joints.push_back(rear_right);
  node->joints.push_back(rear_left);
  // node->joints.push_back(front_right);
  // node->joints.push_back(front_left);
  node->joints.push_back(front_right_steer);
  node->joints.push_back(front_left_steer);

  node->run();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}