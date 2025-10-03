#ifndef COMMON_HPP
#define COMMON_HPP
#include <Eigen/Eigen>
#include <deque>
#include <mutex>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <shared_mutex>
namespace loam {
const double REFLECTOR_INFORMATION_WEIGHT = 10000;
const double CORNER_INFORMATION_WEIGHT = 400;
const double ODOM_INFORMATION_WEIGHT = 1600;

struct Reflector {
  Eigen::Vector2d pos{0, 0};
  int id{-1};
};
using ReflectorObs = Reflector;  // 观测点
struct Corner {
  Eigen::Vector2d pos{0, 0};
  double curvature{-1};
};  //
using CornerObs = Corner;
struct Line {
  Eigen::Vector2d start{0, 0};
  Eigen::Vector2d end{0, 0};
  Eigen::Vector2d n_vec{0, 1};
  int num_points{0};
};
using LineObs = Line;

struct KeyFrame {
  int id{-1};
  Eigen::Matrix3d cov{Eigen::Matrix3d::Zero()};
  Eigen::Isometry2d pose{Eigen::Isometry2d::Identity()};
  double time_stamp{};
  std::vector<Reflector> reflectors;
  std::vector<Corner> corners;
  std::vector<Line> lines;
  sensor_msgs::msg::LaserScan::SharedPtr scan{nullptr};
  Eigen::VectorXd descriptor;  // 描述子向量
  uint64_t version{0};
  void computeDescriptor() {
    descriptor.resize(6);
    if (!reflectors.empty()) {
      Eigen::Vector2d refl_mean(0, 0);
      for (auto& p : reflectors) refl_mean += p.pos;
      refl_mean /= reflectors.size();
      descriptor(0) = refl_mean.x();
      descriptor(1) = refl_mean.y();
      descriptor(2) = reflectors.size();
    } else {
      descriptor(0) = descriptor(1) = descriptor(2) = 0;
    }

    if (!corners.empty()) {
      Eigen::Vector2d corner_mean(0, 0);
      for (auto& p : corners) corner_mean += p.pos;
      corner_mean /= corners.size();
      descriptor(3) = corner_mean.x();
      descriptor(4) = corner_mean.y();
      descriptor(5) = corners.size();
    } else {
      descriptor(3) = descriptor(4) = descriptor(5) = 0;
    }
  }
};

using RefMap = std::vector<Reflector>;
using CornerMap = std::vector<Corner>;
using LineMap = std::vector<Line>;
using RefMatchRes = std::vector<std::pair<Eigen::Vector2d, Eigen::Vector2d>>;
using CornerMatchRes = RefMatchRes;
using RefObsArray = std::vector<ReflectorObs>;
using CornerObsArray = std::vector<CornerObs>;
using LineObsArray = std::vector<LineObs>;
class LaserMapping;
class MapManager {
 public:
  explicit MapManager(size_t window_size = 10) : window_size_(window_size) {}

  // 添加新的关键帧到缓存（线程安全）
  void addKeyFrameToCache(const KeyFrame& kf) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    cache_frames_[kf.id] = kf;
  }

  // 将缓存关键帧加入全局关键帧池，并更新滑动窗口
  std::map<int, KeyFrame> integrateCacheFrames(LaserMapping* mapping);

  // 获取局部地图特征集合（滑动窗口），线程安全
  void getLocalMapFeatures(std::vector<Reflector>& out_reflectors,
                           std::vector<Corner>& out_corners,
                           std::vector<Line>& out_lines) {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    out_reflectors.clear();
    out_corners.clear();
    out_lines.clear();

    for (int id : loc_window_) {
      const auto& kf = key_frames_.at(id);
      out_reflectors.insert(out_reflectors.end(), kf.reflectors.begin(),
                            kf.reflectors.end());
      out_corners.insert(out_corners.end(), kf.corners.begin(),
                         kf.corners.end());
      out_lines.insert(out_lines.end(), kf.lines.begin(), kf.lines.end());
    }
  }

  // 访问全局关键帧（只读，线程不安全）
  std::map<int, KeyFrame> getAllKeyFrames() {
    // std::shared_lock<std::shared_mutex> lock(mtx_);
    return key_frames_;
  }
  size_t getKeyFrameSize() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return key_frames_.size();
  }
  // 访问最新的关键帧
  KeyFrame getLatestKeyFrame() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return key_frames_[key_frames_.size() - 1];
  }
  // 访问关键帧（只读，线程不安全）
  KeyFrame getKeyFrame(int id) {
    // std::shared_lock<std::shared_mutex> lock(mtx_);
    if (key_frames_.find(id) == key_frames_.end()) {
      return KeyFrame();
    }
    return key_frames_[id];
  }
  // 访问缓存关键帧（只读，线程安全）
  std::map<int, KeyFrame> getCacheFrames() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return cache_frames_;
  }
  // 更新关键帧（优化后）
  void updateKeyFrame(const KeyFrame& kf) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    if (key_frames_.find(kf.id) == key_frames_.end()) return;
    key_frames_[kf.id] = kf;
  }

  // 批量更新（推荐）
  void updateKeyFrames(const std::vector<KeyFrame>& frames) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    for (const auto& kf : frames) {
      if (key_frames_.find(kf.id) == key_frames_.end()) {
        continue;
      }
      if (key_frames_[kf.id].version < kf.version) key_frames_[kf.id] = kf;
    }
  }

  // 批量更新，右值引用版本（避免拷贝）
  void updateKeyFrames(std::vector<KeyFrame>&& frames) {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    for (auto& kf : frames) {
      if (key_frames_.find(kf.id) == key_frames_.end()) {
        continue;
      }
      if (key_frames_[kf.id].version < kf.version) {
        key_frames_[kf.id] = std::move(kf);
      }
      key_frames_[kf.id] = std::move(kf);
    }
  }

  // 获取滑动窗口关键帧ID（只读，线程安全）
  std::deque<int> getLocalWindow() {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    return loc_window_;
  }
  // 获取局部地图关键帧集合（只读，线程安全）
  std::vector<KeyFrame> getLocalKeyFrames() const {
    std::shared_lock<std::shared_mutex> lock(mtx_);
    std::vector<KeyFrame> local_kfs;
    local_kfs.reserve(loc_window_.size());
    for (int id : loc_window_) {
      auto it = key_frames_.find(id);
      if (it != key_frames_.end()) {
        local_kfs.push_back(it->second);  // 拷贝出去，保证线程安全
      }
    }
    return local_kfs;
  }
  // 清空所有数据
  void clearAll() {
    std::unique_lock<std::shared_mutex> lock(mtx_);
    key_frames_.clear();
    cache_frames_.clear();
    loc_window_.clear();
  }

 private:
  std::map<int, KeyFrame> key_frames_;    // 全局关键帧
  std::map<int, KeyFrame> cache_frames_;  // 缓存关键帧
  std::deque<int> loc_window_;            // 局部滑动窗口关键帧ID
  size_t window_size_;                    // 滑动窗口大小

  mutable std::shared_mutex mtx_;  // 保护所有数据的线程安全
};

}  // namespace loam
#endif