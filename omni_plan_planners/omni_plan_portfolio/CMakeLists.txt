cmake_minimum_required(VERSION 3.5)
project(omni_plan_portfolio)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(omni_plan_cache REQUIRED)
find_package(poirot REQUIRED)
find_package(poirot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin_ros REQUIRED)

include_directories(src)
include_directories(include)

set(DEPENDENCIES
  pluginlib::pluginlib
  omni_plan::omni_plan
  poirot::poirot
  ${poirot_msgs_TARGETS}
  rclcpp::rclcpp
  yasmin_ros::yasmin_ros
)

# Core library
add_library(omni_plan_portfolio SHARED
  src/portfolio_planner_selector.cpp
)
target_link_libraries(omni_plan_portfolio PUBLIC ${DEPENDENCIES})
target_include_directories(omni_plan_portfolio PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS omni_plan_portfolio
  EXPORT omni_plan_portfolioTargets
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

add_library(portfolio_planner SHARED
  src/portfolio_planner.cpp
)
target_include_directories(portfolio_planner PUBLIC
  ${omni_plan_cache_INCLUDE_DIRS}
)
target_link_libraries(portfolio_planner PUBLIC
  ${DEPENDENCIES} omni_plan_portfolio ${omni_plan_cache_LIBRARIES}
)
install(TARGETS portfolio_planner
  LIBRARY DESTINATION lib
)

ament_export_targets(omni_plan_portfolioTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(omni_plan)
ament_export_dependencies(omni_plan_cache)
ament_export_dependencies(poirot)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_portfolio_planner
    test/test_portfolio_planner.cpp
  )
  target_include_directories(test_portfolio_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_portfolio_planner
    omni_plan_portfolio
    portfolio_planner
    ${DEPENDENCIES}
  )
endif()

ament_package()
