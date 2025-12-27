cmake_minimum_required(VERSION 3.5)
project(easy_plan_val)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(easy_plan REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  easy_plan::easy_plan
)

# plugins
pluginlib_export_plugin_description_file(easy_plan plugins.xml)

add_library(val_validator SHARED src/easy_plan_val/val_validator.cpp)
target_link_libraries(val_validator PUBLIC ${DEPENDENCIES})
install(TARGETS val_validator
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

    # Test for ValValidator
  ament_add_gtest(test_val_validator test/test_val_validator.cpp)
  target_include_directories(test_val_validator PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_val_validator
    val_validator
    ${DEPENDENCIES}
  )
endif()

ament_package()