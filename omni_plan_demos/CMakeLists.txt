cmake_minimum_required(VERSION 3.5)
project(omni_plan_demos)

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
find_package(pluginlib REQUIRED)
find_package(omni_plan REQUIRED)
find_package(omni_plan_yasmin REQUIRED)
find_package(omni_plan_bt REQUIRED)
find_package(omni_plan_msgs REQUIRED)
find_package(omni_plan_knowledge_base REQUIRED)
find_package(knowledge_graph REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  ament_index_cpp::ament_index_cpp
  pluginlib::pluginlib
  omni_plan::omni_plan
  omni_plan_yasmin::omni_plan_yasmin
  omni_plan_bt::omni_plan_bt
  knowledge_graph::knowledge_graph
)

# plugins
add_library(ask_charge_action SHARED src/ask_charge_action.cpp)
target_link_libraries(ask_charge_action PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS ask_charge_action
  LIBRARY DESTINATION lib
)

add_library(charge_action SHARED src/charge_action.cpp)
target_link_libraries(charge_action PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS charge_action
  LIBRARY DESTINATION lib
)

add_library(move_action SHARED src/move_action.cpp)
target_link_libraries(move_action PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS move_action
  LIBRARY DESTINATION lib
)

add_library(move_yasmin_action SHARED src/move_yasmin_action.cpp)
target_link_libraries(move_yasmin_action PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS move_yasmin_action
  LIBRARY DESTINATION lib
)

add_library(move_state SHARED src/move_state.cpp)
target_link_libraries(move_state PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(yasmin plugins.xml)
install(TARGETS move_state
  LIBRARY DESTINATION lib
)

add_library(move_yasmin_factory_action SHARED src/move_yasmin_factory_action.cpp)
target_link_libraries(move_yasmin_factory_action PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS move_yasmin_factory_action
  LIBRARY DESTINATION lib
)

add_library(move_bt_action SHARED src/move_bt_action.cpp)
target_link_libraries(move_bt_action PUBLIC ${DEPENDENCIES})
pluginlib_export_plugin_description_file(omni_plan plugins.xml)
install(TARGETS move_bt_action
  LIBRARY DESTINATION lib
)

add_library(move_bt_node SHARED src/move_bt_node.cpp)
target_link_libraries(move_bt_node PUBLIC ${DEPENDENCIES})
target_compile_definitions(move_bt_node PRIVATE BT_PLUGIN_EXPORT)


# Create demo executable
add_executable(knowledge_graph_demo src/knowledge_graph_demo.cpp)
target_link_libraries(knowledge_graph_demo
  rclcpp::rclcpp
  knowledge_graph::knowledge_graph
  ${knowledge_graph_msgs_TARGETS}
)

# Create knowledge base demo executable
add_executable(knowledge_base_demo src/knowledge_base_demo.cpp)
target_include_directories(knowledge_base_demo PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${omni_plan_knowledge_base_INCLUDE_DIRS}
)
target_link_libraries(knowledge_base_demo
  rclcpp::rclcpp
  ${omni_plan_msgs_TARGETS}
  ${omni_plan_knowledge_base_LIBRARIES}
)
ament_target_dependencies(knowledge_base_demo
  omni_plan_knowledge_base
)

install(TARGETS
  knowledge_graph_demo
  knowledge_base_demo
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY params launch state_machines behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()