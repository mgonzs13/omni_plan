cmake_minimum_required(VERSION 3.5)
project(omni_plan_tests)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(omni_plan_knowledge_base REQUIRED)
find_package(omni_plan REQUIRED)

set(DEPENDENCIES
  rclcpp::rclcpp
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  omni_plan::omni_plan
  omni_plan_knowledge_base::knowledge_base_client
)

# =============================================================================
# Mock Action Plugins Library
# =============================================================================
# Build mock action plugins as a shared library for plugin loading
add_library(${PROJECT_NAME} SHARED
  src/mock_move_action.cpp
  src/mock_charge_action.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC ${DEPENDENCIES})


# Export plugins
pluginlib_export_plugin_description_file(omni_plan plugins.xml)

# Install library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install params directory
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)

# =============================================================================
# Testing
# =============================================================================
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Integration test for full planning pipeline
  ament_add_gtest(test_planning_pipeline test/test_planning_pipeline.cpp)
  target_include_directories(test_planning_pipeline PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_planning_pipeline ${DEPENDENCIES})

  # Integration test for PDDL generation
  ament_add_gtest(test_pddl_generation test/test_pddl_generation.cpp)
  target_include_directories(test_pddl_generation PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_pddl_generation ${DEPENDENCIES})

  # Full system integration test with knowledge base and POPF planner
  ament_add_gtest(test_full_system test/test_full_system.cpp)
  target_include_directories(test_full_system PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_full_system ${DEPENDENCIES})
endif()

ament_package()
