cmake_minimum_required(VERSION 3.5)
project(tf_publisher)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
    )

add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp tf2 tf2_ros std_msgs sensor_msgs geometry_msgs)
add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp tf2 tf2_ros std_msgs sensor_msgs geometry_msgs)
add_executable(tf_convert src/tf_convert.cpp)
ament_target_dependencies(tf_convert rclcpp tf2 tf2_ros std_msgs sensor_msgs geometry_msgs)

install(TARGETS
  pub
  sub
  tf_convert
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
