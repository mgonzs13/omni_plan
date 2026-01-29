cmake_minimum_required(VERSION 3.5)
project(omni_plan_knowledge_graph)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(knowledge_graph REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  pluginlib::pluginlib
  omni_plan::omni_plan
  knowledge_graph::knowledge_graph
)

# plugins
pluginlib_export_plugin_description_file(omni_plan plugins.xml)

add_library(kg_pddl_manager SHARED src/omni_plan_knowledge_graph/kg_pddl_manager.cpp)
target_link_libraries(kg_pddl_manager PUBLIC ${DEPENDENCIES})
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