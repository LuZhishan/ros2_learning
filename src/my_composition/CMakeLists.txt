cmake_minimum_required(VERSION 3.5)

project(my_composition)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)

# 根据源文件生成库文件，并添加依赖项
add_library(talker_component SHARED
    src/talker_component.cpp)
ament_target_dependencies(talker_component
    rclcpp
    rclcpp_components
    std_msgs
)
add_library(listener_component SHARED
    src/listener_component.cpp)
ament_target_dependencies(listener_component
    rclcpp
    rclcpp_components
    std_msgs
)

# 将生成的库文件转化成插件
rclcpp_components_register_node(talker_component 
    PLUGIN "my_composition::Talker"
    EXECUTABLE talker_node
)
rclcpp_components_register_node(listener_component 
    PLUGIN "my_composition::Listener"
    EXECUTABLE listener_node
)

# 安装插件
install(TARGETS
    talker_component
    listener_component
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

# 配置launch文件夹
install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}
)
  
ament_package()