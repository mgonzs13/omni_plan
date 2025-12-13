cmake_minimum_required(VERSION 3.5)
project(easy_plan_knowledge_graph)

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
find_package(yasmin_ros REQUIRED)
find_package(knowledge_graph REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  pluginlib::pluginlib
  easy_plan::easy_plan
  yasmin_ros::yasmin_ros
  knowledge_graph::knowledge_graph
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  rclcpp_action
  pluginlib
  easy_plan
  yasmin_ros
  knowledge_graph
)

# plugins
add_library(kg_pddl_manager SHARED src/easy_plan_knowledge_graph/kg_pddl_manager.cpp)
target_link_libraries(kg_pddl_manager PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
install(TARGETS kg_pddl_manager
  LIBRARY DESTINATION lib
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_kg_pddl_manager test/test_kg_pddl_manager.cpp)
  target_include_directories(test_kg_pddl_manager PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_kg_pddl_manager
    kg_pddl_manager
    ${DEPENDENCIES}
  )
endif()

ament_package()