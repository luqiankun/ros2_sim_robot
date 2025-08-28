cmake_minimum_required(VERSION 3.8)
project(ref_slam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Ceres REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(YAML-CPP REQUIRED)
file(GLOB_RECURSE SOURCE_SRCS src/*.cpp src/*.cc)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
add_executable(ref_slam_node
  ${SOURCE_SRCS}
)

target_include_directories(ref_slam_node PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
  ref_slam_node
  "rclcpp"
  "geometry_msgs"
  "nav_msgs"
  "tf2"
  "tf2_ros"
  "custom_interfaces"
  "sensor_msgs"
  "visualization_msgs"
  "std_srvs"
)

target_link_libraries(ref_slam_node
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
install(TARGETS ref_slam_node
  DESTINATION lib/${PROJECT_NAME})
ament_package()
