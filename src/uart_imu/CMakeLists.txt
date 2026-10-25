cmake_minimum_required(VERSION 3.8)
project(uart_imu)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(uart_imu_node src/uart_imu_node.cpp)
target_compile_features(uart_imu_node PUBLIC c_std_99 cxx_std_17)

ament_target_dependencies(uart_imu_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2_ros
)

install(TARGETS
  uart_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
