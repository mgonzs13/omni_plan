cmake_minimum_required(VERSION 3.5)
project(omni_plan_cache)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(rclcpp REQUIRED)
find_package(yasmin_ros REQUIRED)

set(DEPENDENCIES
  pluginlib::pluginlib
  omni_plan::omni_plan
  OpenSSL::Crypto
  rclcpp::rclcpp
  yasmin_ros::yasmin_ros
)

pluginlib_export_plugin_description_file(omni_plan plugins.xml)

add_library(cache_planner SHARED
  src/omni_plan_cache/cache_planner.cpp
  src/omni_plan_cache/detail/sha256.cpp
  src/omni_plan_cache/detail/structural_keyer.cpp
  src/omni_plan_cache/detail/relevance_analyzer.cpp
  src/omni_plan_cache/detail/plan_adapter.cpp
  src/omni_plan_cache/detail/component_composer.cpp
  src/omni_plan_cache/detail/plan_cache.cpp
)
target_include_directories(cache_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(cache_planner PUBLIC ${DEPENDENCIES})
install(TARGETS cache_planner
  EXPORT cache_plannerTargets
  LIBRARY DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_include_directories(include)
ament_export_libraries(cache_planner)
ament_export_targets(cache_plannerTargets)
ament_export_dependencies(omni_plan)
ament_export_dependencies(rclcpp)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_cache_planner test/test_cache_planner.cpp)
  target_include_directories(test_cache_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_cache_planner
    cache_planner
    ${DEPENDENCIES}
  )

  foreach(detail_test
      test_cache_detail_sha256
      test_cache_detail_relevance
      test_cache_detail_structural
      test_cache_detail_adapter
      test_cache_detail_composer
      test_cache_detail_plan_cache)
    ament_add_gtest(${detail_test} test/${detail_test}.cpp)
    target_include_directories(${detail_test} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${detail_test}
      cache_planner
      ${DEPENDENCIES}
    )
  endforeach()

  ament_add_gtest(test_cache_planner_extras test/test_cache_planner_extras.cpp)
  target_include_directories(test_cache_planner_extras PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_cache_planner_extras
    cache_planner
    ${DEPENDENCIES}
  )

  add_executable(benchmark
    benchmark/benchmark.cpp
  )
  target_include_directories(benchmark PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(benchmark
    cache_planner
    ${DEPENDENCIES}
  )
endif()

ament_package()
