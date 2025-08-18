#include "../include/driver.hpp"

void Motor::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  auto& names = msg->name;
  auto& velocities = msg->velocity;
  auto& positions = msg->position;
  auto it = std::find(names.begin(), names.end(), link_name_);
  if (it != names.end()) {
    index = std::distance(names.begin(), it);
    position_ = positions[index] / M_PI / 2.0;
    velocity_ = velocities[index] / M_PI / 2.0 * 60;  // rad/s->rpm
  }
  can_msgs::msg::Frame send_msg;
  send_msg.id = TPDO3 + cannode_id_;
  send_msg.dlc = 8;
  send_msg.is_rtr = false;
  send_msg.is_extended = false;
  send_msg.is_error = false;
  memcpy(send_msg.data.data(), (unsigned char*)&velocity_, 4);
  memcpy(send_msg.data.data() + 4, (unsigned char*)&position_, 4);
  can_pub_->publish(send_msg);
}
void Motor::can_callback(const can_msgs::msg::Frame::SharedPtr msg) {
  if (!msg->is_error && !msg->is_rtr) {
    if (msg->id == RPDO4 + cannode_id_) {
      // vel
      assert(msg->dlc == 4 && msg->data.size() >= 4);
      float speed{0};
      memcpy(&speed, msg->data.data(), 4);  // 明确指定拷贝 4 字节
      send_velocity_ = speed / 60.0 * 2 * M_PI;
    } else if (msg->id == RPDO3 + cannode_id_) {
      // pos
      assert(msg->dlc == 4 && msg->data.size() >= 4);
      float position{0};
      memcpy(&position, msg->data.data(), 4);  // 明确指定拷贝 4 字节
      send_position_ = position * M_PI * 2.0;
    }
  }
}
Motor::Motor(rclcpp::Node::SharedPtr node, int cannode_id,
             const std::string& link_name)
    : cannode_id_(cannode_id), link_name_(link_name), node_(node) {
  can_sub_ = node->create_subscription<can_msgs::msg::Frame>(
      can_tx_topic, 1000,
      std::bind(&Motor::can_callback, this, std::placeholders::_1));
  can_pub_ = node->create_publisher<can_msgs::msg::Frame>(can_rx_topic, 10);
  joint_state_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 100,
      std::bind(&Motor::joint_state_callback, this, std::placeholders::_1));
}

void Driver::timer_callback() {
  if (motors_.empty()) return;
  if (pose_controller_map.empty() && vel_controller_map.empty()) return;
  std_msgs::msg::Float64MultiArray vel_msg;
  for (auto& x : vel_controller_map) {
    auto it = std::find_if(
        motors_.begin(), motors_.end(),
        [&](const auto& motor) { return x.second == motor->link_name_; });
    if (it != motors_.end()) {
      vel_msg.data.push_back((*it)->send_velocity_);
    }
  }
  pub_motor_velocity_->publish(vel_msg);
  std_msgs::msg::Float64MultiArray pos_msg;
  for (auto& x : pose_controller_map) {
    auto it = std::find_if(
        motors_.begin(), motors_.end(),
        [&](const auto& motor) { return x.second == motor->link_name_; });
    if (it != motors_.end()) {
      pos_msg.data.push_back((*it)->send_position_);
    }
  }
  pub_motor_position_->publish(pos_msg);
}

void Driver::add_motor(int cannode_id, const std::string& link_name) {
  motors_.push_back(std::make_shared<Motor>(node_, cannode_id, link_name));
}

void Driver::start() {
  get_controllers_ =
      node_->create_client<controller_manager_msgs::srv::ListControllers>(
          "/controller_manager/list_controllers");
  get_controllers_->wait_for_service();
  auto request = std::make_shared<
      controller_manager_msgs::srv::ListControllers::Request>();
  get_controllers_->async_send_request(
      request,
      [&](rclcpp::Client<
          controller_manager_msgs::srv::ListControllers>::SharedFuture future) {
        auto responce = future.get();
        for (auto& x : responce->controller) {
          if (x.name == "position_controller") {
            for (auto& interface : x.claimed_interfaces) {
              auto it = interface.find('/');
              if (it != std::string::npos) {
                std::string joint = interface.substr(0, it);
                pose_controller_map.insert({pose_controller_map.size(), joint});
              }
            }
          } else if (x.name == "velocity_controller") {
            for (auto& interface : x.claimed_interfaces) {
              auto it = interface.find('/');
              if (it != std::string::npos) {
                std::string joint = interface.substr(0, it);
                vel_controller_map.insert({vel_controller_map.size(), joint});
              }
            }
          }
        }
        RCLCPP_INFO(node_->get_logger(), "get_controller success");
        pub_motor_velocity_ =
            node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                velocity_topic, 10);
        pub_motor_position_ =
            node_->create_publisher<std_msgs::msg::Float64MultiArray>(
                position_topic, 10);
        auto period = std::chrono::milliseconds{100};
        pub_timer_ = node_->create_wall_timer(
            period, std::bind(&Driver::timer_callback, this));
      });
}

Driver::Driver(rclcpp::Node::SharedPtr n) : node_{n} {}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("car_description_driver");
  auto driver = std::make_shared<Driver>(node);
  driver->add_motor(0, "main_wheel_joint");
  driver->add_motor(1, "fork_joint");
  driver->add_motor(2, "main_wheel_base_joint");
  driver->start();
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
}