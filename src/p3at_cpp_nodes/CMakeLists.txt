cmake_minimum_required(VERSION 3.8)
project(p3at_cpp_nodes)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(std_srvs REQUIRED)
find_package(p3at_interface REQUIRED)


add_executable(odom_node2 src/odom_node2.cpp)
ament_target_dependencies(odom_node2 rclcpp geometry_msgs nav_msgs tf2_ros tf2 sensor_msgs)

add_executable(joy_control src/joy_control.cpp)
ament_target_dependencies(joy_control rclcpp sensor_msgs rosbag2_interfaces
                          std_msgs slam_toolbox geometry_msgs std_srvs)
add_executable(test_lidar src/test_lidar.cpp)
ament_target_dependencies(test_lidar rclcpp sensor_msgs)

add_executable(explore src/explore.cpp)
ament_target_dependencies(explore rclcpp geometry_msgs sensor_msgs tf2 p3at_interface std_msgs)

add_executable(goal_client src/goal_client.cpp)
ament_target_dependencies(goal_client rclcpp rclcpp_action nav2_msgs geometry_msgs
                          std_srvs p3at_interface tf2 std_msgs)

install(TARGETS
  goal_client
  joy_control
  odom_node2
  test_lidar
  explore
  DESTINATION lib/${PROJECT_NAME}
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

ament_package()
