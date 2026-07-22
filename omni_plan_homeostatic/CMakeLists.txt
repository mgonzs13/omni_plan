cmake_minimum_required(VERSION 3.5)
project(omni_plan_homeostatic)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(poirot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin_ros REQUIRED)

include_directories(src)
include_directories(include)

set(DEPENDENCIES
  pluginlib::pluginlib
  omni_plan::omni_plan
  ${poirot_msgs_TARGETS}
  rclcpp::rclcpp
  yasmin_ros::yasmin_ros
)

# Core library
add_library(omni_plan_homeostatic SHARED
  src/homeostatic_planner_selector.cpp
)
target_link_libraries(omni_plan_homeostatic PUBLIC ${DEPENDENCIES})
target_include_directories(omni_plan_homeostatic PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS omni_plan_homeostatic
  EXPORT omni_plan_homeostaticTargets
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params/
)

# Plugins
pluginlib_export_plugin_description_file(omni_plan plugins.xml)

add_library(homeostatic_planner SHARED
  src/homeostatic_planner.cpp
)
target_link_libraries(homeostatic_planner PUBLIC
  ${DEPENDENCIES} omni_plan_homeostatic
)
install(TARGETS homeostatic_planner
  LIBRARY DESTINATION lib
)

ament_export_targets(omni_plan_homeostaticTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(omni_plan)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_homeostatic_planner
    test/test_homeostatic_planner.cpp
  )
  target_include_directories(test_homeostatic_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_homeostatic_planner
    omni_plan_homeostatic
    ${DEPENDENCIES}
  )
endif()

ament_package()
