cmake_minimum_required(VERSION 3.5)
project(omni_plan_dispatcher)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Determine BehaviorTree.CPP version from ROS distro
if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined.")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(omni_plan REQUIRED)
find_package(omni_plan_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(include)

set(DEPENDENCIES
  omni_plan::omni_plan
  ${omni_plan_msgs_TARGETS}
  pluginlib::pluginlib
  rclcpp::rclcpp
)

# Register all three plugins with omni_plan as the host package.
pluginlib_export_plugin_description_file(omni_plan plugins.xml)

# ── Sequential dispatcher ──────────────────────────────────────────────────
add_library(sequential_plan_dispatcher SHARED
  src/omni_plan_dispatcher/sequential_plan_dispatcher.cpp
)
target_include_directories(sequential_plan_dispatcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(sequential_plan_dispatcher PUBLIC ${DEPENDENCIES})
install(TARGETS sequential_plan_dispatcher LIBRARY DESTINATION lib)

# ── Parallel dispatcher ────────────────────────────────────────────────────
add_library(parallel_plan_dispatcher SHARED
  src/omni_plan_dispatcher/parallel_plan_dispatcher.cpp
)
target_include_directories(parallel_plan_dispatcher PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(parallel_plan_dispatcher PUBLIC ${DEPENDENCIES})
install(TARGETS parallel_plan_dispatcher LIBRARY DESTINATION lib)

# ── Install headers ────────────────────────────────────────────────────────
install(DIRECTORY include/
  DESTINATION include
)

# ── Testing ────────────────────────────────────────────────────────────────
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_plan_dispatchers
    test/test_plan_dispatchers.cpp
  )
  target_include_directories(test_plan_dispatchers PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_plan_dispatchers
    sequential_plan_dispatcher
    parallel_plan_dispatcher
    ${DEPENDENCIES}
  )
endif()

ament_package()
