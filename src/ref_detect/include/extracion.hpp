#ifndef EXTRACION_HPP
#define EXTRACION_HPP
#include <sensor_msgs/msg/laser_scan.hpp>
#include <unordered_set>

template <typename T>
struct comp {
  bool operator()(const T& lhs, const T& rhs) const {
    return lhs.index < rhs.index;
  }
};
template <class T>
struct Hash {
  size_t operator()(const T& r1) const {
    return std::hash<uint32_t>()(r1.index);
  }
  friend bool operator==(T& lhs, T& rhs) { return lhs.index == rhs.index; }
};
class FitPost;
class Post;
class PostPoint;
using Feature2D = FitPost;
using Feature2DList = std::vector<Feature2D>;
bool extrac(sensor_msgs::msg::LaserScan::ConstSharedPtr msg,
            std::vector<PostPoint>&);

bool association(std::vector<PostPoint>&, std::vector<Post>&);

class DSU {
 private:
  std::vector<int> root;

 public:
  DSU(int n) {
    root.resize(n);
    for (int i = 0; i < n; i++) root[i] = i;
  }
  int find(int x) {
    if (x != root[x]) {         // 有父节点
      root[x] = find(root[x]);  // 递归，直至到达根节点
    }
    return root[x];  // 返回根节点
  }
  void merge(int x, int y) { root[find(x)] = find(y); }
};

#endif