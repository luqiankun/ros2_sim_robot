#ifndef COMMON_H
#define COMMON_H

#include <Eigen/Dense>
#include <chrono>
#include <string>
#include <unordered_map>
#include <vector>
namespace reflector_slam {

// 反光板观测（传感器坐标系）
struct Observation {
  Eigen::Vector3d point;  // 3D坐标
  int id;                 // 匹配到的地图ID（-1表示未匹配）
  double confidence;      // 观测置信度（0~1）
};

// 地图中的反光板（世界坐标系）
struct Reflector {
  Eigen::Vector3d position;  // 位置
  int id;                    // 唯一ID
};

// 关键帧
struct Keyframe {
  int id;                // 关键帧ID
  Eigen::Matrix4d pose;  // 传感器位姿（T_world_sensor）
  std::chrono::steady_clock::time_point timestamp;  // 时间戳
  std::vector<Observation> observations;            // 观测数据
};

}  // namespace reflector_slam

#endif  // COMMON_H