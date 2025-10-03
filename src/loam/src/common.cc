#include "../include/common.hpp"

#include "../include/laser_mapping.hpp"
namespace loam {
std::map<int, KeyFrame> MapManager::integrateCacheFrames(
    LaserMapping* mapping) {
  std::unique_lock<std::shared_mutex> lock(mtx_);
  for (auto& [id, kf] : cache_frames_) {
    KeyFrame last;
    if (!key_frames_.empty()) last = key_frames_[key_frames_.size() - 1];
    key_frames_[id] = kf;
    mapping->add_kf_to_global_optimizer(kf, last);
    mapping->add_kf_to_local_optimizer(kf, last);
    loc_window_.push_back(id);
    if (loc_window_.size() > window_size_) {
      loc_window_.pop_front();
    }
  }
  for (auto& [id, kf] : cache_frames_) {
    mapping->add_loop_closure_edge(kf);
  }
  cache_frames_.clear();
  return getAllKeyFrames();
}
}  // namespace loam