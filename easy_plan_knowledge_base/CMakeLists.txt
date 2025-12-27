cmake_minimum_required(VERSION 3.5)
project(easy_plan_knowledge_base)

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
find_package(easy_plan REQUIRED)
find_package(easy_plan_msgs REQUIRED)

# Include directories
include_directories(include)

# KnowledgeBase library
add_library(knowledge_base SHARED
  src/easy_plan_knowledge_base/knowledge_base.cpp
)
target_link_libraries(knowledge_base
    easy_plan::easy_plan
    ${easy_plan_msgs_TARGETS}
)

# KnowledgeBaseClient library
add_library(knowledge_base_client SHARED
  src/easy_plan_knowledge_base/knowledge_base_client.cpp
)
target_link_libraries(knowledge_base_client
  rclcpp::rclcpp
  easy_plan::easy_plan
  ${easy_plan_msgs_TARGETS}
)

# KnowledgeBaseNode library
add_library(knowledge_base_node_lib SHARED
  src/easy_plan_knowledge_base/knowledge_base_node.cpp
)
target_link_libraries(knowledge_base_node_lib
  knowledge_base
  rclcpp::rclcpp
  easy_plan::easy_plan
  ${easy_plan_msgs_TARGETS}
)

# KnowledgeBaseNode executable
add_executable(knowledge_base_node
  src/easy_plan_knowledge_base/main.cpp
)
target_link_libraries(knowledge_base_node
  knowledge_base_node_lib
)
install(TARGETS knowledge_base_node
  DESTINATION lib/${PROJECT_NAME}
)

# KbPddlManager plugin library
add_library(kb_pddl_manager SHARED
  src/easy_plan_knowledge_base/kb_pddl_manager.cpp
)
target_link_libraries(kb_pddl_manager
  knowledge_base_client
  rclcpp::rclcpp
  pluginlib::pluginlib
  easy_plan::easy_plan
  ${easy_plan_msgs_TARGETS}
)

# Install plugin library
install(TARGETS kb_pddl_manager knowledge_base_client
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
pluginlib_export_plugin_description_file(easy_plan plugins.xml)

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
    knowledge_base
    rclcpp::rclcpp
    easy_plan::easy_plan
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
    easy_plan::easy_plan
    ${easy_plan_msgs_TARGETS}
  )
endif()

# Export targets
ament_export_targets(export_knowledge_base_client HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(rclcpp easy_plan easy_plan_msgs)

ament_package()
