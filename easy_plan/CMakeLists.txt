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
find_package(rclcpp REQUIRED)
find_package(easy_plan_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(poirot REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  rclcpp::rclcpp
  ${easy_plan_msgs_TARGETS}
  pluginlib::pluginlib
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  poirot::poirot
)
set(DEPENDENCIES_PACKAGE
  rclcpp
  easy_plan_msgs
  pluginlib
  yasmin
  yasmin_ros
  poirot
)

# plugins
add_library(easy_plan SHARED 
  src/easy_plan/pddl_manager.cpp
  src/easy_plan/plan_validator.cpp
  src/easy_plan/planner.cpp
  src/easy_plan/pddl/plan.cpp
  src/easy_plan/pddl/domain.cpp
  src/easy_plan/pddl/problem.cpp
  src/easy_plan/pddl/action.cpp
  src/easy_plan/pddl/timing_predicate.cpp
  src/easy_plan/pddl/predicate.cpp
  src/easy_plan/pddl/object.cpp
)
target_link_libraries(easy_plan PUBLIC ${DEPENDENCIES})
target_include_directories(easy_plan PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
install(TARGETS easy_plan
  EXPORT easy_planTargets
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include
)

# plugins
pluginlib_export_plugin_description_file(yasmin plugins.xml)

add_library(get_params_state SHARED src/easy_plan/states/get_params_state.cpp)
target_link_libraries(get_params_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS get_params_state
  LIBRARY DESTINATION lib
)

add_library(generate_pddl_state SHARED src/easy_plan/states/generate_pddl_state.cpp)
target_link_libraries(generate_pddl_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS generate_pddl_state
  LIBRARY DESTINATION lib
)

add_library(plan_state SHARED src/easy_plan/states/plan_state.cpp)
target_link_libraries(plan_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS plan_state
  LIBRARY DESTINATION lib
)

add_library(validate_plan_state SHARED src/easy_plan/states/validate_plan_state.cpp)
target_link_libraries(validate_plan_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS validate_plan_state
  LIBRARY DESTINATION lib
)

add_library(execute_plan_state SHARED src/easy_plan/states/execute_plan_state.cpp)
target_link_libraries(execute_plan_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS execute_plan_state
  LIBRARY DESTINATION lib
)

add_library(idle_state SHARED src/easy_plan/states/idle_state.cpp)
target_link_libraries(idle_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS idle_state
  LIBRARY DESTINATION lib
)

add_library(load_plugins_state SHARED src/easy_plan/states/load_plugins_state.cpp)
target_link_libraries(load_plugins_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS load_plugins_state
  LIBRARY DESTINATION lib
)

add_library(publish_pddl_state SHARED src/easy_plan/states/publish_pddl_state.cpp)
target_link_libraries(publish_pddl_state PUBLIC ${DEPENDENCIES} easy_plan)
install(TARGETS publish_pddl_state
  LIBRARY DESTINATION lib
)

# Install state_machines directory
install(DIRECTORY state_machines
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for PDDL classes
  ament_add_gtest(test_pddl_classes test/test_pddl_classes.cpp)
  target_include_directories(test_pddl_classes PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_pddl_classes
    easy_plan
    ${DEPENDENCIES}
  )

  # Test for Plan class
  ament_add_gtest(test_plan test/test_plan.cpp)
  target_include_directories(test_plan PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_plan
    easy_plan
    ${DEPENDENCIES}
  )

  # Test for Action class
  ament_add_gtest(test_action test/test_action.cpp)
  target_include_directories(test_action PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_action
    easy_plan
    ${DEPENDENCIES}
  )

endif()

ament_export_targets(easy_planTargets HAS_LIBRARY_TARGET)
ament_export_dependencies(${DEPENDENCIES_PACKAGE})

ament_package()