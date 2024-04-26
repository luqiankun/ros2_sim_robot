cmake_minimum_required(VERSION 3.8)
project(ref_detect)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
add_compile_options(-std=c++17)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(Ceres REQUIRED)
include_directories("/usr/include/eigen3")
add_executable(ref_detect src/main.cc)
ament_target_dependencies(
  ref_detect
  rclcpp
  std_msgs
  tf2_ros
  tf2
  sensor_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
)
target_link_libraries(ref_detect ${CERES_LIBRARIES})
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
install(TARGETS ref_detect
DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
DESTINATION share/${PROJECT_NAME}
)
ament_package()
