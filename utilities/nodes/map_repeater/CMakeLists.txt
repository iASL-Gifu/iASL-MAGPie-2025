cmake_minimum_required(VERSION 3.8)
project(map_repeater)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)

add_executable(map_repeater_node src/map_repeater_node.cpp)
ament_target_dependencies(map_repeater_node rclcpp nav_msgs)

install(TARGETS
  map_repeater_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
