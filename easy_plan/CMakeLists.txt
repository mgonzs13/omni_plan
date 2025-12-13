cmake_minimum_required(VERSION 3.5)
project(easy_plan)

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
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  pluginlib::pluginlib
  yasmin::yasmin
  yasmin_ros::yasmin_ros
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  rclcpp_action
  pluginlib
  yasmin
  yasmin_ros
)

# plugins
add_library(easy_plan_core SHARED 
  src/easy_plan/plan.cpp
  src/easy_plan/pddl_manager.cpp
  src/easy_plan/pddl/action.cpp
  src/easy_plan/pddl/expression.cpp
)
target_link_libraries(easy_plan_core PUBLIC ${DEPENDENCIES})
install(TARGETS easy_plan_core
  LIBRARY DESTINATION lib
)

add_library(get_params_state SHARED src/easy_plan/states/get_params_state.cpp)
target_link_libraries(get_params_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS get_params_state
  LIBRARY DESTINATION lib
)

add_library(generate_pddl_state SHARED src/easy_plan/states/generate_pddl_state.cpp)
target_link_libraries(generate_pddl_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS generate_pddl_state
  LIBRARY DESTINATION lib
)

add_library(plan_state SHARED src/easy_plan/states/plan_state.cpp)
target_link_libraries(plan_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS plan_state
  LIBRARY DESTINATION lib
)

add_library(validate_plan_state SHARED src/easy_plan/states/validate_plan_state.cpp)
target_link_libraries(validate_plan_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS validate_plan_state
  LIBRARY DESTINATION lib
)

add_library(execute_plan_state SHARED src/easy_plan/states/execute_plan_state.cpp)
target_link_libraries(execute_plan_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS execute_plan_state
  LIBRARY DESTINATION lib
)

add_library(idle_state SHARED src/easy_plan/states/idle_state.cpp)
target_link_libraries(idle_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS idle_state
  LIBRARY DESTINATION lib
)

add_library(load_plugins_state SHARED src/easy_plan/states/load_plugins_state.cpp)
target_link_libraries(load_plugins_state PUBLIC ${DEPENDENCIES} easy_plan_core)
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS load_plugins_state
  LIBRARY DESTINATION lib
)

# Install state_machines directory
install(DIRECTORY state_machines
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()