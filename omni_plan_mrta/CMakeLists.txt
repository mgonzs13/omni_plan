cmake_minimum_required(VERSION 3.5)
project(omni_plan_mrta)

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
find_package(yasmin_ros REQUIRED)

include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  pluginlib::pluginlib
  omni_plan::omni_plan
  yasmin_ros::yasmin_ros
)

# Register plugin description files
pluginlib_export_plugin_description_file(omni_plan planner_plugins.xml)
pluginlib_export_plugin_description_file(omni_plan_mrta allocator_plugins.xml)

# Single shared library containing the planner and all allocator plugins
add_library(omni_plan_mrta SHARED
  src/omni_plan_mrta/mrta_planner.cpp
  src/omni_plan_mrta/allocators/round_robin_allocator.cpp
  src/omni_plan_mrta/allocators/ssi_affinity_allocator.cpp
  src/omni_plan_mrta/allocators/greedy_auction_allocator.cpp
  src/omni_plan_mrta/allocators/cbba_allocator.cpp
)
target_include_directories(omni_plan_mrta PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(omni_plan_mrta PUBLIC ${DEPENDENCIES})

install(TARGETS omni_plan_mrta
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_mrta test/test_mrta.cpp)
  target_include_directories(test_mrta PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_mrta
    omni_plan_mrta
    ${DEPENDENCIES}
  )
endif()

ament_package()
