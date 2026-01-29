cmake_minimum_required(VERSION 3.5)
project(omni_plan_knowledge_base)

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
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(omni_plan_msgs REQUIRED)

# Include directories
include_directories(include)

# KnowledgeBase library
add_library(omni_plan_knowledge_base SHARED
  src/omni_plan_knowledge_base/knowledge_base.cpp
)
target_link_libraries(omni_plan_knowledge_base
    omni_plan::omni_plan
    ${omni_plan_msgs_TARGETS}
)
target_include_directories(omni_plan_knowledge_base PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# KnowledgeBaseClient library
add_library(knowledge_base_client SHARED
  src/omni_plan_knowledge_base/knowledge_base_client.cpp
)
target_link_libraries(knowledge_base_client
  rclcpp::rclcpp
  omni_plan::omni_plan
  ${omni_plan_msgs_TARGETS}
)
target_include_directories(knowledge_base_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# KnowledgeBaseNode library
add_library(knowledge_base_node_lib SHARED
  src/omni_plan_knowledge_base/knowledge_base_node.cpp
)
target_link_libraries(knowledge_base_node_lib
  omni_plan_knowledge_base
  rclcpp::rclcpp
  omni_plan::omni_plan
  ${omni_plan_msgs_TARGETS}
)
target_include_directories(knowledge_base_node_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# KnowledgeBaseNode executable
add_executable(knowledge_base_node
  src/omni_plan_knowledge_base/main.cpp
)
target_link_libraries(knowledge_base_node
  knowledge_base_node_lib
)
install(TARGETS knowledge_base_node
  DESTINATION lib/${PROJECT_NAME}
)

# KbPddlManager plugin library
add_library(kb_pddl_manager SHARED
  src/omni_plan_knowledge_base/kb_pddl_manager.cpp
)
target_link_libraries(kb_pddl_manager
  knowledge_base_client
  rclcpp::rclcpp
  pluginlib::pluginlib
  omni_plan::omni_plan
  ${omni_plan_msgs_TARGETS}
)
target_include_directories(kb_pddl_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Install plugin library
install(TARGETS kb_pddl_manager knowledge_base_client omni_plan_knowledge_base
  EXPORT export_knowledge_base_client
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install include directory
install(DIRECTORY include/
  DESTINATION include
)

# Install plugin description file
pluginlib_export_plugin_description_file(omni_plan plugins.xml)

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for KnowledgeBase
  ament_add_gtest(test_knowledge_base test/test_knowledge_base.cpp)
  target_include_directories(test_knowledge_base PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_knowledge_base
    omni_plan_knowledge_base
    rclcpp::rclcpp
    omni_plan::omni_plan
  )

  # Test for KnowledgeBaseNode
  ament_add_gtest(test_knowledge_base_node
    test/test_knowledge_base_node.cpp
  )
  target_include_directories(test_knowledge_base_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_knowledge_base_node
    knowledge_base_node_lib
    knowledge_base_client
    rclcpp::rclcpp
    omni_plan::omni_plan
    ${omni_plan_msgs_TARGETS}
  )
endif()

# Export targets
ament_export_targets(export_knowledge_base_client HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp omni_plan omni_plan_msgs)

ament_package()
