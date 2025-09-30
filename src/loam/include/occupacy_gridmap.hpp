#pragma once
#include <yaml-cpp/yaml.h>

#include <atomic>
#include <deque>
#include <fstream>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

#include "common.hpp"  // 你的 KeyFrame / Reflector / Corner 定义

namespace loam {
class MapPublisher {
 public:
  MapPublisher(rclcpp::Node::SharedPtr node, double map_resolution = 0.05,
               int map_width = 2000, int map_height = 2000,
               double origin_x = -50, double origin_y = -50,
               size_t window_size = 10)
      : node_(node),
        stop_thread_(false),
        window_size_(window_size),
        map_resolution_(map_resolution),
        map_width_(map_width),
        map_height_(map_height),
        origin_x_(origin_x),
        origin_y_(origin_y) {
    pub_ = node_->create_publisher<sensor_msgs::msg::Image>("map_image", 1);
    map_image_ =
        cv::Mat(map_height_, map_width_, CV_8UC3, cv::Scalar(128, 128, 128));
    occupancy_grid_ = cv::Mat(map_height_, map_width_, CV_32FC1, cv::Scalar(0));

    worker_ = std::thread([this]() { this->updateLoop(); });
  }
  void Stop() {
    if (!is_start) return;
    std::lock_guard<std::mutex> lock(stop_mtx_);  // 防止 Stop 多线程调用
    if (stopped_) return;  // 已经停止过了，直接返回
    stop_thread_ = true;

    // join 线程，如果 joinable 才 join
    if (worker_.joinable()) {
      worker_.join();
      stopped_ = true;
    }

    cv::Mat img;
    generateMapImage(img);
    cv::imwrite("map.png", img);

    YAML::Emitter out;
    out << YAML::BeginMap;
    out << YAML::Key << "image" << YAML::Value << "map.png";
    out << YAML::Key << "resolution" << YAML::Value << map_resolution_;
    out << YAML::Key << "origin" << YAML::Value << YAML::Flow
        << std::vector<double>{origin_x_, origin_y_, 0};
    out << YAML::Key << "negate" << YAML::Value << 0;
    out << YAML::Key << "occupied_thresh" << YAML::Value << 0.65;
    out << YAML::Key << "free_thresh" << YAML::Value << 0.196;

    // 反光板部分
    out << YAML::Key << "reflectors" << YAML::Value << YAML::BeginSeq;
    for (auto& r : ref_map) {
      out << YAML::Flow << std::vector<double>{r.x(), r.y()};
    }
    out << YAML::EndSeq;

    out << YAML::EndMap;

    std::ofstream fout("map.yaml");
    fout << out.c_str();
  }
  ~MapPublisher() { Stop(); }
  void updateReflectorMap(const std::vector<Eigen::Vector2d>& r) {
    std::lock_guard<std::mutex> lock(ref_mtx_);
    ref_map = r;
  }
  // 插入新的关键帧（写 back_buffer）
  void insertKeyFrame(const KeyFrame& kf) {
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    key_frames_back_[kf.id] = kf;
    loc_window_.push_back(kf.id);
    if (loc_window_.size() > window_size_) loc_window_.pop_front();
  }

  // 更新关键帧
  void updateKeyFrame(const KeyFrame& kf) {
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    if (key_frames_back_.find(kf.id) != key_frames_back_.end()) {
      if (kf.version > key_frames_back_[kf.id].version)
        key_frames_back_[kf.id] = kf;
    }
  }

  // 批量更新关键帧
  void updateKeyFrames(const std::vector<KeyFrame>& kfs) {
    std::lock_guard<std::mutex> lock(buffer_mtx_);
    for (const auto& kf : kfs)
      if (key_frames_back_.find(kf.id) != key_frames_back_.end()) {
        if (kf.version > key_frames_back_[kf.id].version)
          key_frames_back_[kf.id] = kf;
      }
  }

  // 切换局部/全局
  void setLocalMode(bool local) { local_mode_ = local; }

 private:
  void updateLoop() {
    is_start = true;
    while (!stop_thread_ && rclcpp::ok()) {
      try {
        cv::Mat img;
        generateMapImage(img);
        publishImage(img);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      } catch (std::runtime_error& e) {
        std::cerr << "updateLoop error: " << e.what() << std::endl;
      }
    }
  }

  void generateMapImage(cv::Mat& out_img);

  void publishImage(const cv::Mat& img) {
    sensor_msgs::msg::Image msg;
    msg.height = img.rows;
    msg.width = img.cols;
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = img.cols * 3;
    msg.data.resize(img.rows * img.cols * 3);
    std::memcpy(msg.data.data(), img.data, img.total() * img.elemSize());
    pub_->publish(msg);
  }

 private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
  cv::Mat map_image_;
  cv::Mat occupancy_grid_;
  std::atomic<bool> stop_thread_;
  std::mutex stop_mtx_;
  bool stopped_ = false;
  std::thread worker_;
  std::map<int, KeyFrame> key_frames_front_;
  std::map<int, KeyFrame> key_frames_back_;
  std::mutex buffer_mtx_;
  std::mutex ref_mtx_;
  bool is_start{false};
  std::deque<int> loc_window_;
  size_t window_size_;
  bool local_mode_ = false;
  std::vector<Eigen::Vector2d> ref_map;
  double map_resolution_;
  int map_width_;
  int map_height_;
  double origin_x_;
  double origin_y_;
};

}  // namespace loam
