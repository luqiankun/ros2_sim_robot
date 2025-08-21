#ifndef REF_DETECT_HPP
#define REF_DETECT_HPP
#include <ceres/ceres.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "./extracion.hpp"
/**
 * @brief 提取的反光柱
 *
 */
class FitPost {
 public:
  double x{0};
  double y{0};
  uint32_t index{0};
  double radius{0};
};
/**
 * @brief 高反光点
 *
 */
class PostPoint {
 public:
  uint32_t index{};
  double x{};
  double y{};
  double f{};
  double theta{};
};
/**
 * @brief 反光柱点云
 *
 */
class Post {
 public:
  Post() : index(0) {}
  std::vector<PostPoint> points;
  uint32_t index;
};
using Feature2D = FitPost;
using Feature2DList = std::vector<Feature2D>;
static bool InsertToFeatureGraph(
    std::vector<std::pair<double, double>>& feature_points_wait_insert,
    std::vector<int>& feature_points_assigned_id,
    std::unordered_map<int, std::pair<double, double>>&,
    std::unordered_map<int, std::unordered_map<int, double>>&);

class RefDetecter {
 public:
  struct Cost {
    Cost(double x, double y) : _x(x), _y(y) {}
    template <typename T>
    bool operator()(const T* const abc, T* resdual) const {
      T coss = (T)(_x) * (T)(_x) + (T)(_y) * (T)(_y) + abc[0] * (T)(_x) +
               abc[1] * (T)(_y) + abc[2];
      resdual[0] = coss;
      return true;
    }
    const double _x, _y;
  };
  class CostParse : public ceres::SizedCostFunction<1, 3> {
   public:
    CostParse(double x, double y)
        : ceres::SizedCostFunction<1, 3>(), _x(x), _y(y) {}
    bool Evaluate(double const* const* parameters, double* residuals,
                  double** jacobians) const override {
      double a = parameters[0][0];
      double b = parameters[0][1];
      double c = parameters[0][2];
      residuals[0] = _x * _x + _y * _y + a * _x + b * _y + c;
      if (jacobians != nullptr) {
        jacobians[0][0] = _x;
        jacobians[0][1] = _y;
        jacobians[0][2] = 1;
      }
      return true;
    }
    const double _x, _y;
  };

  class CostC : public ceres::SizedCostFunction<1, 2> {
   public:
    CostC(double x, double y, double r)
        : ceres::SizedCostFunction<1, 2>(), _x(x), _y(y), _r(r) {}
    bool Evaluate(double const* const* parameters, double* residuals,
                  double** jacobians) const override {
      double a = parameters[0][0];
      double b = parameters[0][1];
      double c = a * a + b * b - _r * _r;
      residuals[0] = _x * _x + _y * _y + a * _x + b * _y + c;
      if (jacobians != nullptr) {
        jacobians[0][0] = _x;
        jacobians[0][1] = _y;
      }
      return true;
    }
    const double _x, _y, _r;
  };
  void scan_cb(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    // auto st = std::chrono::system_clock::now();
    std::vector<Post> temp_posts;
    std::vector<PostPoint> scan_points;
    extrac(msg, scan_points);
    // 高反点关联
    association(scan_points, temp_posts);
    // ceres 拟合
    fit_post(temp_posts, fit_posts);
    // auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    //               std::chrono::system_clock::now() - st)
    //               .count();
    // RCLCPP_INFO(node->get_logger(), "size :%ld", fit_posts.size());
    cur_feature_graph.clear();
    cur_feature_points.clear();
    for (auto& x : fit_posts) {
      cur_feature_points[x.index] = std::pair<double, double>{x.x, x.y};
    }
    std::vector<std::pair<double, double>> temp_input;
    std::vector<int> ids;
    for (auto fpt : cur_feature_points) {
      temp_input.push_back(fpt.second);
      ids.push_back(fpt.first);
    }
    InsertToFeatureGraph(temp_input, ids, cur_feature_points,
                         cur_feature_graph);
    // RCLCPP_INFO(node->get_logger(), "size :%ld", cur_feature_graph.size());
    {
      visualization_msgs::msg::MarkerArray markers;
      auto feature_graph = cur_feature_graph;
      auto feature_points = cur_feature_points;
      int num = 0;
      for (auto& graph : feature_graph) {
        float r{1}, g{0.5}, b{0.5};
        // r = static_cast<float>((rand() % (255))) / 255;
        // g = static_cast<float>((rand() % (255))) / 255;
        // b = static_cast<float>((rand() % (255))) / 255;
        for (auto& x : graph.second) {
          visualization_msgs::msg::Marker marker;
          marker.header.frame_id = "main_2d_lidar_link";
          // marker.header.stamp = time_now->now();
          marker.lifetime.sec = 0;
          marker.lifetime.nanosec = 101000000;
          marker.color.a = 0.7;
          marker.color.r = r;
          marker.color.g = g;
          marker.color.b = b;
          marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
          marker.action = visualization_msgs::msg::Marker::ADD;
          marker.id = num++;
          geometry_msgs::msg::Point st;
          geometry_msgs::msg::Point ed;
          st.x = feature_points[graph.first].first;
          st.y = feature_points[graph.first].second;
          ed.x = feature_points[x.first].first;
          ed.y = feature_points[x.first].second;
          marker.points.push_back(st);
          marker.points.push_back(ed);
          marker.scale.x = 0.01;
          marker.scale.y = 0.01;
          markers.markers.push_back(marker);
        }
        publisher_fit->publish(markers);
      }
    }
    // auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    //               std::chrono::system_clock::now() - st)
    //               .count();
    // RCLCPP_INFO(node->get_logger(), "time :%d", dt);
  }

  void fit_post(std::vector<Post>& posts, Feature2DList& fit_posts) {
    Feature2DList temp_fit_posts;
    // ceres 解析求导
    for (auto& x : posts) {
      ceres::Problem problem;
      ceres::Solver::Options options;
      ceres::Solver::Summary summary;
      double X{0}, Y{0};
      double ABC[3];
      options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;
      options.minimizer_progress_to_stdout = false;
      for (auto& p : x.points) {
        ceres::CostFunction* cost = new CostParse(p.x, p.y);
        problem.AddResidualBlock(cost, new ceres::HuberLoss(0.5), ABC);
        // RCLCPP_INFO(node->get_logger(), "add (%f , %f)", p.x, p.y);
      }
      ceres::Solve(options, &problem, &summary);
      X = -ABC[0] / 2;
      Y = -ABC[1] / 2;
      auto sqr = sqrt(X * X + Y * Y - ABC[2]);
      FitPost fit_post;
      fit_post.x = X;
      fit_post.y = Y;
      fit_post.index = x.index;
      fit_post.radius = sqr;
      bool exist{false};
      for (auto& x : temp_fit_posts) {
        auto err = std::hypot(x.x - fit_post.x, x.y - fit_post.y);
        if (err <= 0.005) {
          exist = true;
          RCLCPP_ERROR(node->get_logger(), "******************************");
          break;
        }
        if (fit_post.radius > 0.04 || fit_post.radius < 0.03) {
          exist = true;
          break;
        }
      }
      if (!exist) {
        // RCLCPP_INFO(node->get_logger(),
        //             "fit_post index:%d x:%f y%f radius: % f ", x.index, X, Y,
        //             sqr);
        temp_fit_posts.push_back(fit_post);
      }
      // RCLCPP_INFO(node->get_logger(), "fit_post index:%d x:%f y%f radius: % f
      // ",
      //             x.index, X, Y, sqr);
    }

    // 几何法
    // for (auto& x : posts) {
    //   double fai{0};
    //   for (auto& p : x.points) {
    //     fai += p.theta;
    //   }
    //   fai = fai / x.points.size();
    //   double Lb = 0;
    //   for (auto& p : x.points) {
    //     double fai_i = p.theta - fai;
    //     double l = sqrt(p.x * p.x + p.y * p.y);
    //     double A = l * cos(fai_i);
    //     double B = l * sin(fai_i) / 0.05;
    //     if (B >= 1) {
    //       B = 1;
    //     } else if (B <= -1) {
    //       B = -1;
    //     }
    //     double C = std::asin(B);

    //     double OrB_i = A + 0.05 * cos(C);
    //     Lb = Lb + OrB_i;
    //   }
    //   auto LB = Lb / x.points.size();
    //   FitPost fit_post;
    //   fit_post.index = x.index;
    //   fit_post.radius = 0.05;
    //   fit_post.x = LB * cos(fai);
    //   fit_post.y = LB * sin(fai);
    //   temp_fit_posts.insert(fit_post);
    // }
    fit_posts = temp_fit_posts;
    std::sort(fit_posts.begin(), fit_posts.end(), comp<FitPost>());
  }

  RefDetecter(rclcpp::Node::SharedPtr n) : node(n) {
    clock = node->create_subscription<rosgraph_msgs::msg::Clock>(
        "/clock", 1, [&](rosgraph_msgs::msg::Clock::ConstSharedPtr msg) {
          time_now = msg->clock;
        });
    subscriber = node->create_subscription<sensor_msgs::msg::LaserScan>(
        topic, 1,
        std::bind(&RefDetecter::scan_cb, this, std::placeholders::_1));
    publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "/fit_posts", 1);
    publisher_fit =
        node->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/feature_graph_fit", 1);
    timer = node->create_wall_timer(std::chrono::milliseconds(100), [&] {
      //
      visualization_msgs::msg::MarkerArray markers;
      std::vector<FitPost> temp_fit_posts;
      temp_fit_posts = fit_posts;
      for (auto& x : temp_fit_posts) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "main_2d_lidar_link";
        // marker.header.stamp = time_now;
        marker.lifetime.sec = 0;
        marker.lifetime.nanosec = 110000000;
        marker.color.a = 0.7;
        marker.color.r = 35.0 / 255;
        marker.color.g = 204.0 / 255;
        marker.color.b = 150.0 / 255;
        marker.scale.x = x.radius * 2;
        marker.scale.y = x.radius * 2;
        marker.scale.z = 0.3;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.set__x(x.x);
        marker.pose.position.set__y(x.y);
        marker.pose.position.set__z(0.2);
        marker.set__id(x.index);
        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.text = "fit_post_" + std::to_string(x.index);
        markers.markers.push_back(marker);
      }
      publisher->publish(markers);
    });
  }

 private:
  std::string topic{"/lidar"};
  rclcpp::Node::SharedPtr node;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      publisher_fit;
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::TimerBase::SharedPtr timer;
  builtin_interfaces::msg::Time time_now;
  std::vector<FitPost> fit_posts;
  std::unordered_map<int, std::pair<double, double>>
      cur_feature_points;  // 索引 坐标
  std::unordered_map<int, std::unordered_map<int, double>>
      cur_feature_graph;  // 索引 索引 距离
};
inline bool InsertToFeatureGraph(
    std::vector<std::pair<double, double>>& feature_points_wait_insert,
    std::vector<int>& feature_points_assigned_id,
    std::unordered_map<int, std::pair<double, double>>& feature_points,
    std::unordered_map<int, std::unordered_map<int, double>>& feature_graph) {
  double const ceil_threshold = 50;
  int cnt = 0;
  for (auto& x : feature_points_wait_insert) {
    int fid = feature_points_assigned_id[cnt++];
    assert(feature_points.size() > 0);
    for (auto fnode : feature_points) {
      int id = fnode.first;
      if (id == fid) {
        continue;
      }
      auto fpt_graph = fnode.second;
      double dist =
          (x.first - fpt_graph.first) * (x.first - fpt_graph.first) +
          (x.second - fpt_graph.second) * (x.second - fpt_graph.second);
      //   RCLCPP_INFO(node->get_logger(), "dist %f", dist);
      if (dist < ceil_threshold * ceil_threshold) {
        feature_graph[fid][id] = dist;
        feature_graph[id][fid] = dist;
        // RCLCPP_INFO(node->get_logger(), "insert %d %d", fid, id);
      }
    }
  }
  //   RCLCPP_INFO(node->get_logger(), "graph size %ld", feature_graph.size());
  return true;
}
#endif
