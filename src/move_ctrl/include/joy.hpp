#ifndef JOY_HPP
#define JOY_HPP
#include <cstdint>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <optional>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/float64.hpp>
#include <string>
#include <vector>
class Button {
 public:
  Button() = delete;
  Button(const std::string &name, uint32_t key) : key(key), name(name) {}
  void update(const std::vector<int> &data) {
    try {
      if (data.at(key)) {
        is_pressed = true;
        is_released = false;
      } else {
        if (is_pressed) {
          is_released = true;
        }
        is_pressed = false;
      }
    } catch (...) {
    }
  }
  bool get_is_pressed() { return is_pressed; }
  bool get_is_released() {
    if (is_released) {
      is_released = false;
      return true;
    } else {
      return false;
    }
  }
  std::string get_name() const { return name; }

 private:
  bool is_pressed{false};
  bool is_released{false};
  uint32_t key{};
  std::string name;
};

class Axes {
 public:
  Axes() = delete;
  Axes(const std::string &name, uint32_t key) : name(name), key(key) {}
  void update(const std::vector<float> &data) {
    try {
      value = data.at(key);
    } catch (...) {
    }
  }
  float get_value() const { return value; }
  std::string get_name() const { return name; }

 private:
  float value{0};
  std::string name;
  uint32_t key{};
};

class Joy {
 public:
  Joy() {
    btns.emplace_back("X", 3);
    btns.emplace_back("Y", 4);
    btns.emplace_back("A", 0);
    btns.emplace_back("B", 1);
    btns.emplace_back("LB", 6);
    btns.emplace_back("RB", 7);
    btns.emplace_back("BACK", 10);
    btns.emplace_back("SET", 11);
    btns.emplace_back("HOME", 12);
    btns.emplace_back("L3", 13);
    btns.emplace_back("R3", 14);
    axess.emplace_back("LH", 0);
    axess.emplace_back("LV", 1);
    axess.emplace_back("LT", 5);
    axess.emplace_back("RT", 4);
    axess.emplace_back("RH", 2);
    axess.emplace_back("RV", 3);
    axess.emplace_back("CROSSH", 6);
    axess.emplace_back("CROSSV", 7);
  }
  Joy(const std::map<std::string, uint32_t> &config_btn,
      const std::map<std::string, uint32_t> &config_axes) {
    update_config(config_btn, config_axes);
  }
  void update_config(const std::map<std::string, uint32_t> &config_btn,
                     const std::map<std::string, uint32_t> &config_axes) {
    btns.clear();
    axess.clear();
    for (auto &x : config_btn) {
      btns.emplace_back(x.first, x.second);
    }
    for (auto &x : config_axes) {
      axess.emplace_back(x.first, x.second);
    }
  }
  void update(const std::vector<int> &btn_data,
              const std::vector<float> &axes_data) {
    for (auto &x : btns) {
      x.update(btn_data);
    }
    for (auto &x : axess) {
      x.update(axes_data);
    }
  }
  std::optional<bool> get_btn_pressed(const std::string &btn_name) {
    for (auto &x : btns) {
      if (x.get_name() == btn_name) {
        auto res = x.get_is_pressed();
        return std::optional<bool>(res);
      }
    }
    return std::nullopt;
  }
  std::optional<bool> get_btn_released(const std::string &btn_name) {
    for (auto &x : btns) {
      if (x.get_name() == btn_name) {
        auto res = x.get_is_released();
        return std::optional<bool>(res);
      }
    }
    return std::nullopt;
  }
  std::optional<float> get_axes_value(const std::string &axes_name) {
    for (auto &x : axess) {
      if (x.get_name() == axes_name) {
        auto res = x.get_value();
        return std::optional<float>(res);
      }
    }
    return std::nullopt;
  }

 private:
  std::vector<Button> btns;
  std::vector<Axes> axess;
};

class JoyStick {
 public:
  JoyStick() = delete;
  JoyStick(rclcpp::Node::SharedPtr node) : node(node) {}
  ~JoyStick() {
    pub_th.join();
    pub_fork_th.join();
  }
  void init();
  bool is_pub();
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void update();
  bool is_fork_pub();

 private:
  std::shared_ptr<Joy> joy;
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr fork_pub;

  float max_linear_vel{2};
  float max_angular_vel{1};
  float time_out{0.3};
  float fork_vel{0};
  geometry_msgs::msg::Twist cmd_vel;
  bool time_out_flag{false};
  rclcpp::Time start_time;
  rclcpp::Time end_time;
  bool time_out_flag_fork{false};
  rclcpp::Time start_time_fork;
  rclcpp::Time end_time_fork;
  float slider{0.1};
  const int PUB_RATE{50};
  const std::string cmd_vel_topic{"cmd_vel"};
  const std::string joy_topic{"joy"};
  std::thread pub_th;
  std::mutex mtx;
  std::condition_variable cv;
  std::thread pub_fork_th;
  std::mutex mtx_fork;
  std::condition_variable cv_fork;
  rclcpp::TimerBase::SharedPtr timer;
};

inline void JoyStick::init() {
  joy = std::make_shared<Joy>();
  joy_sub = node->create_subscription<sensor_msgs::msg::Joy>(
      joy_topic, 10,
      std::bind(&JoyStick::joy_callback, this, std::placeholders::_1));
  fork_pub = node->create_publisher<std_msgs::msg::Float64>("fork_vel", 10);
  cmd_pub =
      node->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
  timer = node->create_timer(std::chrono::milliseconds(100), [&] {
    update();
    if (fabs(cmd_vel.linear.x) > 1e-3 || fabs(cmd_vel.angular.z) > 1e-3) {
      cv.notify_one();
    }
    if (fabs(fork_vel) > 1e-3) {
      cv_fork.notify_one();
    }
  });
  pub_th = std::thread([&] {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lck(mtx);
      cv.wait(lck, [&] { return is_pub(); });
      cmd_pub->publish(cmd_vel);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / PUB_RATE));
    }
  });
  pub_fork_th = std::thread([&] {
    while (rclcpp::ok()) {
      std::unique_lock<std::mutex> lck_fork(mtx_fork);
      cv_fork.wait(lck_fork, [&] { return is_fork_pub(); });
      std_msgs::msg::Float64 msg;
      msg.data = fork_vel;
      fork_pub->publish(msg);
      std::this_thread::sleep_for(std::chrono::milliseconds(1000 / PUB_RATE));
    }
  });
}

inline bool JoyStick::is_pub() {
  bool res = true;
  if (fabs(cmd_vel.linear.x) < 1e-3 && fabs(cmd_vel.angular.z) < 1e-3) {
    if (!time_out_flag) {
      time_out_flag = true;
      start_time = node->now();
    }
    end_time = node->now();
    if ((end_time - start_time).seconds() > time_out) {
      res = false;
    }
  } else {
    time_out_flag = false;
  }
  return res;
}

inline bool JoyStick::is_fork_pub() {
  bool res = true;
  if (fabs(fork_vel) < 1e-3) {
    if (!time_out_flag_fork) {
      time_out_flag_fork = true;
      start_time_fork = node->now();
    }
    end_time_fork = node->now();
    if ((end_time_fork - start_time_fork).seconds() > time_out) {
      res = false;
    }
  } else {
    time_out_flag_fork = false;
  }
  return res;
}

inline void JoyStick::update() {
  auto Y = joy->get_btn_released("Y");
  if (Y.has_value()) {
    if (Y.value()) {
      slider += 0.1;
      if (slider > 1) {
        slider = 1;
      }
    }
  }
  auto A = joy->get_btn_released("A");
  if (A.has_value()) {
    if (A.value()) {
      slider -= 0.1;
      if (slider < 0) {
        slider = 0;
      }
    }
  }

  auto LV = joy->get_axes_value("LV");
  auto RH = joy->get_axes_value("RH");
  cmd_vel.linear.x = LV.value_or(0) * slider * max_linear_vel;  //
  cmd_vel.angular.z = RH.value_or(0) * slider * max_angular_vel;
  //   ROS_INFO_STREAM(vel.linear.x << "\t" << vel.linear.y << "\t"
  //                                << vel.angular.z);
  auto LB = joy->get_btn_pressed("LB");
  if (LB.has_value()) {
    bool press = LB.value();
    auto RT = joy->get_axes_value("RT");
    if (RT.has_value()) {
      fork_vel = (1 - RT.value()) / 4;
    }
    if (press) {
      fork_vel *= -1;
    }
  }
}
inline void JoyStick::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
  joy->update(msg->buttons, msg->axes);
}

#endif