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

include_directories(include)

set(DEPENDENCIES
  pluginlib::pluginlib
  omni_plan::omni_plan
  OpenSSL::Crypto
)

pluginlib_export_plugin_description_file(omni_plan plugins.xml)

add_library(cache_planner SHARED src/omni_plan_cache/cache_planner.cpp)
target_link_libraries(cache_planner PUBLIC ${DEPENDENCIES})
install(TARGETS cache_planner
  LIBRARY DESTINATION lib
)

install(DIRECTORY
  params
  DESTINATION share/${PROJECT_NAME}/
)

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
endif()

ament_package()
