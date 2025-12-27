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
find_package(rclcpp_components REQUIRED)
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
    rclcpp::rclcpp
    easy_plan::easy_plan
    ${easy_plan_msgs_TARGETS}
  )
endif()

ament_package()
