cmake_minimum_required(VERSION 3.5)
project(easy_plan_bt)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO is not defined.")
endif()

if("$ENV{ROS_DISTRO}" STREQUAL "foxy" OR "$ENV{ROS_DISTRO}" STREQUAL "galactic" OR "$ENV{ROS_DISTRO}" STREQUAL "humble" OR "$ENV{ROS_DISTRO}" STREQUAL "iron")
  message(STATUS "Using BehaviorTree.CPP V3")
  set(BT_LIB_NAME behaviortree_cpp_v3)
  set(BT_TARGET ${behaviortree_cpp_v3_TARGETS})
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBTV3")
else()
  message(STATUS "Using BehaviorTree.CPP V4")
  set(BT_LIB_NAME behaviortree_cpp)
  set(BT_TARGET behaviortree_cpp::behaviortree_cpp)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DBTV4")
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(easy_plan REQUIRED)
find_package(${BT_LIB_NAME} REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  easy_plan::easy_plan
)
set(DEPENDENCIES_PACKAGE
  easy_plan
  ${BT_LIB_NAME}
)

if(BT_LIB_NAME STREQUAL "behaviortree_cpp_v3")
  # For v3, use ament_target_dependencies
else()
  set(DEPENDENCIES ${DEPENDENCIES} ${BT_TARGET})
endif()

# Create library
add_library(${PROJECT_NAME} SHARED
  src/easy_plan_bt/bt_action.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})

if(BT_LIB_NAME STREQUAL "behaviortree_cpp_v3")
  ament_target_dependencies(${PROJECT_NAME} PUBLIC ${BT_LIB_NAME})
endif()

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

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for BtAction class
  ament_add_gtest(test_bt_action test/test_bt_action.cpp)
  target_include_directories(test_bt_action PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_bt_action
    ${PROJECT_NAME}
    ${DEPENDENCIES}
  )
endif()

ament_package()