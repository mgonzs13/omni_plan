cmake_minimum_required(VERSION 3.5)
project(easy_plan_vhpop)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(easy_plan REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  easy_plan::easy_plan
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  ament_index_cpp
  pluginlib
  easy_plan
)

# plugins
pluginlib_export_plugin_description_file(easy_plan plugins.xml)

add_library(vhpop_planner SHARED src/easy_plan_vhpop/vhpop_planner.cpp)
target_link_libraries(vhpop_planner PUBLIC ${DEPENDENCIES})
install(TARGETS vhpop_planner
  LIBRARY DESTINATION lib
)

# Install planner directory
install(DIRECTORY
  planner
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for VhpopPlanner
  ament_add_gtest(test_vhpop_planner test/test_vhpop_planner.cpp TIMEOUT 300)
  target_include_directories(test_vhpop_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_vhpop_planner
    vhpop_planner
    ${DEPENDENCIES}
  )
endif()

ament_package()