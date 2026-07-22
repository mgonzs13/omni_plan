cmake_minimum_required(VERSION 3.5)
project(omni_plan_lpg)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  pluginlib::pluginlib
  omni_plan::omni_plan
)

# plugins
pluginlib_export_plugin_description_file(omni_plan plugins.xml)

add_library(lpg_planner SHARED src/omni_plan_lpg/lpg_planner.cpp)
target_link_libraries(lpg_planner PUBLIC ${DEPENDENCIES})
install(TARGETS lpg_planner
  LIBRARY DESTINATION lib
)

# Install bin directory
install(DIRECTORY
  bin
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for LpgPlanner
  ament_add_gtest(test_lpg_planner test/test_lpg_planner.cpp)
  target_include_directories(test_lpg_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_lpg_planner
    lpg_planner
    ${DEPENDENCIES}
  )
endif()

ament_package()
