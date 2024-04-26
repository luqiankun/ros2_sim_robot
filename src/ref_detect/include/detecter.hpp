#ifndef DETECR_HPP
#define DETECR_HPP
#include <ceres/ceres.h>

#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
class RefDetecter {
 public:
  struct PostPoint {
    uint32_t index;
    double x;
    double y;
    double f;
    double theta;
  };
  template <typename T>
  struct comp {
    bool operator()(const T& lhs, const T& rhs) const {
      return lhs.index < rhs.index;
    }
  };
  struct Post {
    std::set<PostPoint, comp<PostPoint>> points;
    uint32_t index;
  };
  struct FitPost {
    double x;
    double y;
    uint32_t index;
    double radius;
  };
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

  void scan_cb(sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
    // auto st = std::chrono::system_clock::now();
    std::set<Post, comp<Post>> temp_posts;
    uint32_t max = msg->ranges.size();
    double step = msg->angle_increment;
    bool flag{false};
    uint32_t beg_index{0};
    uint32_t end_index{0};
    for (uint32_t i = 0; i < max; ++i) {
      if (!std::isinf(msg->ranges.at(i)) && !std::isnan(msg->ranges.at(i)) &&
          msg->intensities.at(i) > 500) {
        if (!flag) {
          flag = true;
          beg_index = i;
        }
      } else {
        if (flag) {
          flag = false;
          end_index = i;
          if (end_index - beg_index > 6) {
            // 提取
            Post post;
            post.index = temp_posts.size();
            for (auto j = beg_index; j < end_index; j++) {
              PostPoint point;
              double theta = msg->angle_min + j * step;
              point.x = cos(theta) * msg->ranges.at(j);
              point.y = sin(theta) * msg->ranges.at(j);
              point.index = j;
              point.f = msg->intensities.at(j);
              point.theta = theta;
              post.points.insert(point);
            }
            temp_posts.insert(post);
          }
          beg_index = i;
        } else {
          beg_index = i;
          end_index = i;
        }
      }
    }
    fit_post(temp_posts, fit_posts);
    // auto dt = std::chrono::duration_cast<std::chrono::milliseconds>(
    //               std::chrono::system_clock::now() - st)
    //               .count();
    // RCLCPP_INFO(node->get_logger(), "time:%ld", dt);
  }

  void fit_post(std::set<Post, comp<Post>>& posts,
                std::set<FitPost, comp<FitPost>>& fit_posts) {
    std::set<FitPost, comp<FitPost>> temp_fit_posts;
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
      temp_fit_posts.insert(fit_post);
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
    timer = node->create_wall_timer(std::chrono::milliseconds(100), [&] {
      //
      visualization_msgs::msg::MarkerArray markers;
      std::set<FitPost, comp<FitPost>> temp_fit_posts;
      temp_fit_posts = fit_posts;
      for (auto& x : temp_fit_posts) {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "MR-Buggy3/Base";
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
  rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock;
  rclcpp::TimerBase::SharedPtr timer;
  builtin_interfaces::msg::Time time_now;
  std::set<FitPost, comp<FitPost>> fit_posts;
};
#endif
