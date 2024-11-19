#include "../include/match.hpp"

#include "../include/detecter.hpp"
std::optional<std::array<Point, 3>> Cells::match(std::array<FitPost, 3> src) {
  FitPost p1 = src[0];
  FitPost p2 = src[1];
  FitPost p3 = src[2];
  Eigen::Vector2d p1p2(p2.x - p1.x, p2.y - p1.y);
  Eigen::Vector2d p1p3(p3.x - p1.x, p3.y - p1.y);
  Eigen::Vector2d p2p3(p3.x - p2.x, p3.y - p2.y);
  std::vector<int> index;

  {
    int left = 0;
    int right = edges.size();
    while (left < right && (right - left) > 1) {
      int mid = (left + right) / 2;
      double coss = edges.at(mid).len - p1p2.norm();
      if (coss > 0) {
        right = mid;
      } else if (coss < 0) {
        left = mid;
      }
      if (std::fabs(coss) < tolerance) {
        index.push_back(edges.at(mid).A.index);
        index.push_back(edges.at(mid).B.index);
        // std::cerr << "----p1p2--------" << edges.at(mid).A.index << " "
        //           << edges.at(mid).B.index << "\n";
      }
    }
  }
  {
    int left = 0;
    int right = edges.size();
    while (left < right && (right - left) > 1) {
      int mid = (left + right) / 2;
      double coss = edges.at(mid).len - p1p3.norm();
      if (coss > 0) {
        right = mid;
      } else if (coss < 0) {
        left = mid;
      }
      if (std::fabs(coss) < tolerance) {
        index.push_back(edges.at(mid).A.index);
        index.push_back(edges.at(mid).B.index);
        // std::cerr << "----p1p3--------" << edges.at(mid).A.index << " "
        //           << edges.at(mid).B.index << "\n";
      }
      // std::cerr << "------p1p3------" << coss << "\n";
    }
  }
  {
    int left = 0;
    int right = edges.size();
    while (left < right && (right - left) > 1) {
      int mid = (left + right) / 2;
      double coss = edges.at(mid).len - p2p3.norm();
      if (coss > 0) {
        right = mid;
      } else if (coss < 0) {
        left = mid;
      }
      if (std::fabs(coss) < tolerance) {
        index.push_back(edges.at(mid).A.index);
        index.push_back(edges.at(mid).B.index);
        // std::cerr << "----p2p3--------" << edges.at(mid).A.index << " "
        //           << edges.at(mid).B.index << "\n";
      }
      // std::cerr << "------p2p3------" << coss << "\n";
    }
  }
  // std::sort(index.begin(), index.end());

  // int little = index.front();
  // int big = index.back();
  std::vector<int> ids;
  for (auto& i : index) {
    auto c = std::count(index.begin(), index.end(), i);
    // std::cerr << i << " count " << c << "\n";
    if (c == 2) {
      ids.push_back(i);
    }
  }
  std::sort(ids.begin(), ids.end());
  ids.erase(unique(ids.begin(), ids.end()), ids.end());
  if (ids.size() < 3) {
    return std::nullopt;
  } else {
    // std::cerr << "^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^" << ids.size()
    //           << "\n";
    std::vector<Point> ps;
    for (auto& x : ids) {
      // std::cerr << "[---------          ]" << x << "\n";
      for (auto& e : edges) {
        if (e.A.index == x) {
          ps.push_back(e.A);
          // std::cerr << e.A.index << " " << e.A.x << " " << e.A.y << "\n";
          break;
        } else if (e.B.index == x) {
          ps.push_back(e.B);
          // std::cerr << e.B.index << " " << e.B.x << " " << e.B.y << "\n";
          break;
        }
      }
    }
    for (size_t i = 0; i < ps.size(); i++) {
      for (size_t j = i + 1; j < ps.size(); j++) {
        for (size_t k = j + 1; k < ps.size(); k++) {
          Eigen::Vector2d m_1_2(ps.at(j).x - ps.at(i).x,
                                ps.at(j).y - ps.at(i).y);
          Eigen::Vector2d m_1_3(ps.at(k).x - ps.at(i).x,
                                ps.at(k).y - ps.at(i).y);
          Eigen::Vector2d m_2_3(ps.at(k).x - ps.at(j).x,
                                ps.at(k).y - ps.at(j).y);
          double len_ = m_1_2.norm() + m_1_3.norm() + m_2_3.norm();
          double len = p1p2.norm() + p1p3.norm() + p2p3.norm();
          if (fabs(len - len_) > tolerance * 3) {
            continue;
          } else {
            if (fabs(p1p2.norm() - m_1_2.norm()) < tolerance) {
              // p1p2=m1_2
              if (fabs(p1p3.norm() - m_1_3.norm()) < tolerance) {
                // p1p3=m13
                // p1=m1 p2=m2 p3=m3
                return std::array<Point, 3>{ps.at(i), ps.at(j), ps.at(k)};
              } else {
                // p1p3=m23
                // p1=m2 p2=m1 p3=m3
                return std::array<Point, 3>{ps.at(j), ps.at(i), ps.at(k)};
              }
            } else if (fabs(p1p2.norm() - m_1_3.norm()) < tolerance) {
              // p1p2=m1m3
              if (fabs(p1p3.norm() - m_1_2.norm()) < tolerance) {
                // p1p3=m1m2
                // p1=m1 p2=m3 p3=m2
                return std::array<Point, 3>{ps.at(i), ps.at(k), ps.at(j)};
              } else {
                // p1p3=m2m3
                // p1=m3 p2=m1 p3=m2
                return std::array<Point, 3>{ps.at(k), ps.at(i), ps.at(j)};
              }
            } else {
              // p1p2=m2m3
              if (fabs(p1p3.norm() - m_1_2.norm()) < tolerance) {
                // p1p3=m1m2
                // p1=m2 p2=m3 p3=m1
                return std::array<Point, 3>{ps.at(j), ps.at(k), ps.at(i)};
              } else {
                // p1p3=m1m3
                // p1=m3 p2=m2 p3=m1
                return std::array<Point, 3>{ps.at(k), ps.at(j), ps.at(i)};
              }
            }
          }
        }
      }
    }
    return std::nullopt;
  }
}

Eigen::Matrix<double, 3, 3> Cells::rough_location(
    std::array<Point, 3> parent, std::array<FitPost, 3> child) {
  Point m1 = parent[0];
  Point m2 = parent[1];
  Point m3 = parent[2];

  FitPost p1 = child[0];
  double theta1 = atan2(p1.y, p1.x);
  FitPost p2 = child[1];
  double theta2 = atan2(p2.y, p2.x);
  FitPost p3 = child[2];
  double theta3 = atan2(p3.y, p3.x);
  Eigen::Matrix<double, 2, 2> A = Eigen::Matrix<double, 2, 2>::Identity();
  A(0, 0) = 2 * (m1.x - m1.x);
  A(0, 1) = 2 * (m3.y - m3.y);
  A(1, 0) = 2 * (m2.x - m3.x);
  A(1, 1) = 2 * (m2.y - m3.y);
  Eigen::Vector2d B;
  B.x() = m1.x * m1.x - m3.x * m3.x + m1.y * m1.y - m3.y * m3.y + p3.x * p3.x +
          p3.y * p3.y - p1.x * p1.x - p1.y * p1.y;
  B.y() = m2.x * m2.x - m3.x * m3.x + m2.y * m2.y - m3.y * m3.y + p3.x * p3.x +
          p3.y * p3.y - p2.x * p2.x - p2.y * p2.y;
  Eigen::Vector2d X = (A.transpose() * A).inverse() * A.transpose() * B;
  double theta_1 = atan2(p1.x - X.x(), p1.y - X.y()) - theta1;
  double theta_2 = atan2(p2.x - X.x(), p2.y - X.y()) - theta2;
  double theta_3 = atan2(p3.x - X.x(), p3.y - X.y()) - theta3;
  double theta = (theta_1 + theta_2 + theta_3) / 3;
  Eigen::Matrix<double, 3, 3> res = Eigen::Matrix<double, 3, 3>::Identity();
  res(0, 0) = cos(theta);
  res(0, 1) = -sin(theta);
  res(1, 0) = sin(theta);
  res(1, 1) = cos(theta);
  res(0, 2) = X.x();
  res(1, 2) = X.y();
  return res;
}
