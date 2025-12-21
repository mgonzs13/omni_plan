cmake_minimum_required(VERSION 3.5)
project(easy_plan_yasmin)

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
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(easy_plan REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_factory REQUIRED)
find_package(yasmin_viewer REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  easy_plan::easy_plan
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  yasmin_factory::yasmin_factory
  yasmin_viewer::yasmin_viewer
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  ament_index_cpp
  pluginlib
  easy_plan
  yasmin
  yasmin_ros
  yasmin_factory
  yasmin_viewer
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/easy_plan_yasmin/yasmin_action.cpp
  src/easy_plan_yasmin/yasmin_factory_action.cpp
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