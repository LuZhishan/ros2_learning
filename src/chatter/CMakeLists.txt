cmake_minimum_required(VERSION 3.5)
project(chatter)

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


add_executable(pub src/pub.cpp)
ament_target_dependencies(pub rclcpp std_msgs)
add_executable(sub_pub src/sub_pub.cpp)
ament_target_dependencies(sub_pub rclcpp std_msgs)
add_executable(sub src/sub.cpp)
ament_target_dependencies(sub rclcpp std_msgs)


install(TARGETS
  pub
  sub_pub
  sub

  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
