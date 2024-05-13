#ifndef REF_MATCH_HPP
#define REF_MATCH_HPP
#include <Eigen/Eigen>
#include <boost/algorithm/string.hpp>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <vector>
class FitPost;
class Point {
 public:
  Point() = delete;
  Point(double x, double y, int index) : x(x), y(y), index(index) {}
  double x;
  double y;
  double index;
  bool operator==(const Point &a) {
    return a.index == this->index && a.x == this->x && a.y == this->y;
  }
  bool operator!=(const Point &a) { return !(*this == a); }
};
class Edge {
 public:
  Edge(Point a, Point b) : A(a), B(b) {
    Eigen::Vector2d AB(b.x - a.x, b.y - a.y);
    len = AB.norm();
  }
  Point A;
  Point B;
  double len;
};
class Cells {
 public:
  struct {
    bool operator()(Edge &a, Edge &b) { return a.len < b.len; }
  } cmp;
  Cells(const std::string &path) {
    // 读取point
    std::vector<Point> points;
    std::fstream file;
    if (std::filesystem::exists(path)) {
      file.open(path, std::iostream::in);
      if (file.is_open()) {
        std::string line;
        while (std::getline(file, line)) {
          if (!line.empty()) {
            std::vector<std::string> res;
            boost::split(res, line, boost::is_any_of("|"));
            assert(res.size() == 3);
            Point p(std::stof(res.at(1)), std::stof(res.at(2)),
                    std::stoi(res.at(0)));
            std::cerr << "-- " << p.index << " " << p.x << " " << p.y << "\n";
            points.push_back(p);
          }
          line.clear();
        }
      }
      file.close();
    }
    assert(!points.empty());
    // 生成edge
    for (size_t i = 0; i < points.size(); i++) {
      auto A = points.at(i);
      for (size_t j = i + 1; j < points.size(); j++) {
        auto B = points.at(j);
        auto e = Edge(A, B);
        edges.push_back(e);
      }
    }
    std::sort(edges.begin(), edges.end(), cmp);
  }
  std::optional<std::array<Point, 3>> match(std::array<FitPost, 3> src);
  Eigen::Matrix<double, 3, 3> rough_location(std::array<Point, 3> parent,
                                             std::array<FitPost, 3> child);

 public:
  double tolerance{0.01};
  std::vector<Edge> edges;
};
#endif