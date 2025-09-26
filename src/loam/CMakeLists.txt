cmake_minimum_required(VERSION 3.8)
project(loam)

cmake_policy(SET CMP0072 NEW)
set(OpenGL_GL_PREFERENCE LEGACY)
find_package(OpenGL REQUIRED)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_BUILD_TYPE Debug)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# find dependencies
find_package(OpenMP REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(g2o REQUIRED)
find_package(YAML-CPP REQUIRED)
file(GLOB_RECURSE SOURCE_SRCS src/*.cpp src/*.cc)
include_directories(${EIGEN3_INCLUDE_DIRS})
include_directories(${YAML_CPP_INCLUDE_DIR})
# add_executable(loam_node
#   ${SOURCE_SRCS}
# )

# target_include_directories(loam_node PUBLIC
#   ${EIGEN3_INCLUDE_DIRS}
# )
# ament_target_dependencies(
#   loam_node
#   "rclcpp"
#   "geometry_msgs"
#   "nav_msgs"
#   "tf2"
#   "tf2_ros"
#   "custom_interfaces"
#   "sensor_msgs"
#   "visualization_msgs"
#   "std_srvs"
# )

# target_link_libraries(loam_node
#   g2o::g2o
#   ${OpenCV_LIBRARIES}
#   ${YAML_CPP_LIBRARIES}
#   OpenMP::OpenMP_CXX
# )



#### TESTING

add_executable(feature_ext_test test/feature_ext_test.cc src/feature_ext.cc src/laser_mapping.cc src/common.cc)
add_executable(laser_odom_test test/laser_odom_test.cc src/feature_ext.cc src/laser_mapping.cc src/common.cc)

target_include_directories(feature_ext_test PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
target_include_directories(laser_odom_test PUBLIC
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(
  feature_ext_test
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
ament_target_dependencies(
  laser_odom_test
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

target_link_libraries(feature_ext_test
  g2o::core
  g2o::stuff
  g2o::types_slam2d
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenMP::OpenMP_CXX
)
target_link_libraries(laser_odom_test
  g2o::core
  g2o::stuff
  g2o::types_slam2d
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  OpenMP::OpenMP_CXX
)
install(TARGETS feature_ext_test
  DESTINATION lib/${PROJECT_NAME})
  install(TARGETS laser_odom_test
  DESTINATION lib/${PROJECT_NAME})
### 
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
# install(TARGETS loam_node
#   DESTINATION lib/${PROJECT_NAME})
# install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
ament_package()
