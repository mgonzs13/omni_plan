cmake_minimum_required(VERSION 3.5)
project(easy_plan_popf)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(pluginlib REQUIRED)
find_package(easy_plan REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  pluginlib::pluginlib
  easy_plan::easy_plan
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  rclcpp_action
  pluginlib
  easy_plan
)

# plugins
add_library(popf_planner SHARED src/easy_plan_popf/popf_planner.cpp)
target_link_libraries(popf_planner PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
install(TARGETS popf_planner
  LIBRARY DESTINATION lib
)

ament_package()