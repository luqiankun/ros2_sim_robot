#include "../include/driver.hpp"

#include <builtin_interfaces/msg/duration.hpp>
#include <controller_manager_msgs/srv/configure_controller.hpp>
void Motor::joint_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  auto& names = msg->name;
  auto& velocities = msg->velocity;
  auto& positions = msg->position;
  auto it = std::find(names.begin(), names.end(), link_name_);
  if (it != names.end()) {
    index = std::distance(names.begin(), it);
    position_ = positions[index] / M_PI / 2.0;
    velocity_ = velocities[index] / M_PI / 2 * 60;  // rad/s->rpm
  } else {
    return;
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

bool Driver::is_controller_loaded(std::string name) {
  auto request = std::make_shared<
      controller_manager_msgs::srv::ListControllers::Request>();
  auto future = get_controllers_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto responce = future.get();
    for (auto& x : responce->controller) {
      if (x.name == name) {
        return true;
      }
    }
  }
  return false;
}

bool Driver::load_controller(const std::string& name) {
  if (is_controller_loaded(name)) {
    return true;
  }
  auto request =
      std::make_shared<controller_manager_msgs::srv::LoadController::Request>();
  request->name = name;
  auto future = load_controller_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    if (response->ok) {
      return true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "加载失败 %s", name.c_str());
      return false;
    }
  }
  return false;
}
bool Driver::is_controller_configured(std::string name) {
  auto request = std::make_shared<
      controller_manager_msgs::srv::ListControllers::Request>();
  auto future = get_controllers_->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node_->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto responce = future.get();
    for (auto& x : responce->controller) {
      if (x.name == name) {
        if (x.state != "unconfigured")
          return true;
        else
          return false;
      }
    }
  }
  return false;
}

bool Driver::configure_controller(std::string name) {
  if (is_controller_configured(name)) return true;
  auto config =
      node_->create_client<controller_manager_msgs::srv::ConfigureController>(
          "/controller_manager/configure_controller");
  config->wait_for_service();
  auto req = std::make_shared<
      controller_manager_msgs::srv::ConfigureController::Request>();
  req->name = name;
  auto ret = config->async_send_request(req);
  rclcpp::spin_until_future_complete(node_->get_node_base_interface(), ret);
  if (ret.get()->ok) {
    return true;
  } else {
    RCLCPP_ERROR(node_->get_logger(), "无法开启控制器: %s", name.c_str());
    return false;
  }
}
bool Driver::switch_controller(std::string name, std::string type) {
  auto request = std::make_shared<
      controller_manager_msgs::srv::SwitchController::Request>();
  if (type == "activate") {
    request->activate_controllers.push_back(name);
  } else if (type == "deactivate") {
    request->deactivate_controllers.push_back(name);
  } else {
    return false;
  }
  request->timeout.sec = 10;
  request->strictness = request->BEST_EFFORT;
  auto fu = switch_controller_->async_send_request(request);
  auto ret =
      rclcpp::spin_until_future_complete(node_->get_node_base_interface(), fu);
  if (ret == rclcpp::FutureReturnCode::SUCCESS) {
    auto response = std::shared_ptr<
        controller_manager_msgs::srv::SwitchController::Response>(
        new controller_manager_msgs::srv::SwitchController::Response);
    *response = *fu.get();
    if (response->ok) {
      return true;
    } else {
      RCLCPP_ERROR(node_->get_logger(), "切换失败");
      return false;
    }
  } else {
    return false;
  }
}

void Driver::start() {
  get_controllers_ =
      node_->create_client<controller_manager_msgs::srv::ListControllers>(
          "/controller_manager/list_controllers");
  get_controllers_->wait_for_service();
  load_controller_ =
      node_->create_client<controller_manager_msgs::srv::LoadController>(
          "/controller_manager/load_controller");
  load_controller_->wait_for_service();
  switch_controller_ =
      node_->create_client<controller_manager_msgs::srv::SwitchController>(
          "/controller_manager/switch_controller");
  switch_controller_->wait_for_service();
  if (!load_controller("position_controller")) {
    RCLCPP_ERROR(node_->get_logger(), "无法加载position_controller控制器");
    return;
  }

  if (!load_controller("velocity_controller")) {
    RCLCPP_ERROR(node_->get_logger(), "无法加载velocity_controller控制器");
    return;
  }
  if (!load_controller("joint_state_broadcaster")) {
    RCLCPP_ERROR(node_->get_logger(), "无法加载joint_state_broadcaster控制器");
    return;
  }
  if (!configure_controller("position_controller")) {
    RCLCPP_ERROR(node_->get_logger(), "无法配置position_controller控制器");
  }
  if (!configure_controller("velocity_controller")) {
    RCLCPP_ERROR(node_->get_logger(), "无法配置velocity_controller控制器");
    return;
  }
  if (!configure_controller("joint_state_broadcaster")) {
    RCLCPP_ERROR(node_->get_logger(), "无法配置joint_state_broadcaster控制器");
    return;
  }
  if (!switch_controller("position_controller", "activate")) {
    RCLCPP_ERROR(node_->get_logger(), "无法开启position_controller控制器");
    return;
  }
  if (!switch_controller("velocity_controller", "activate")) {
    RCLCPP_ERROR(node_->get_logger(), "无法开启velocity_controller控制器");
    return;
  }
  if (!switch_controller("joint_state_broadcaster", "activate")) {
    RCLCPP_ERROR(node_->get_logger(), "无法开启joint_state_broadcaster控制器");
    return;
  }
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
        RCLCPP_INFO(node_->get_logger(), "get_controller success %ld  %ld",
                    pose_controller_map.size(), vel_controller_map.size());
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
  node->set_parameter(rclcpp::Parameter("use_sim_time", true));
  auto driver = std::make_shared<Driver>(node);
  driver->add_motor(1, "main_wheel_joint");
  driver->add_motor(2, "fork_joint");
  driver->add_motor(3, "main_wheel_base_joint");
  driver->start();
  rclcpp::executors::MultiThreadedExecutor exe;
  exe.add_node(node);
  exe.spin();
  rclcpp::shutdown();
}