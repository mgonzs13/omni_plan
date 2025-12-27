cmake_minimum_required(VERSION 3.5)
project(easy_plan_bt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(easy_plan REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  easy_plan::easy_plan
  ${behaviortree_cpp_v3_TARGETS}
)
set(DEPENDENCIES_PACKAGE
  easy_plan
  behaviortree_cpp_v3
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/easy_plan_bt/bt_action.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} ${DEPENDENCIES})

# Install library
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES_PACKAGE})

ament_package()