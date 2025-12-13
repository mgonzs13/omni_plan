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

add_library(popf_validator SHARED src/easy_plan_popf/popf_validator.cpp)
target_link_libraries(popf_validator PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)
install(TARGETS popf_validator
  LIBRARY DESTINATION lib
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for PopfPlanner
  ament_add_gtest(test_popf_planner test/test_popf_planner.cpp)
  target_include_directories(test_popf_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_popf_planner
    popf_planner
    ${DEPENDENCIES}
  )

  # Test for PopfValidator
  ament_add_gtest(test_popf_validator test/test_popf_validator.cpp)
  target_include_directories(test_popf_validator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_popf_validator
    popf_validator
    ${DEPENDENCIES}
  )
endif()

ament_package()