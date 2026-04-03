cmake_minimum_required(VERSION 3.10)
project(omni_plan_tui)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(omni_plan_msgs REQUIRED)
find_package(yasmin_msgs REQUIRED)
find_package(Curses REQUIRED)

add_executable(omni_plan_tui
  src/main.cpp
  src/tui_node.cpp
  src/tui_renderer.cpp
  src/data_manager.cpp
)

target_include_directories(omni_plan_tui PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CURSES_INCLUDE_DIR}
)

target_link_libraries(omni_plan_tui
  ${CURSES_LIBRARIES}
  rclcpp::rclcpp
  ${omni_plan_msgs_TARGETS}
  ${yasmin_msgs_TARGETS}
)

install(TARGETS omni_plan_tui
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
