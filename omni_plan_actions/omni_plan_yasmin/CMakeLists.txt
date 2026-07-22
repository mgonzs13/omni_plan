cmake_minimum_required(VERSION 3.5)
project(omni_plan_yasmin)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(omni_plan REQUIRED)
find_package(yasmin REQUIRED)
find_package(yasmin_ros REQUIRED)
find_package(yasmin_factory REQUIRED)
find_package(yasmin_viewer REQUIRED)

# C++
include_directories(src)
include_directories(include)

set(DEPENDENCIES
  omni_plan::omni_plan
  yasmin::yasmin
  yasmin_ros::yasmin_ros
  yasmin_factory::yasmin_factory
  yasmin_viewer::yasmin_viewer
)
set(DEPENDENCIES_PACKAGE
  omni_plan
  yasmin
  yasmin_ros
  yasmin_factory
  yasmin_viewer
)

# Create library
add_library(${PROJECT_NAME} SHARED
  src/omni_plan_yasmin/yasmin_action.cpp
  src/omni_plan_yasmin/yasmin_factory_action.cpp
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

# Testing
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  # Test for YasminAction class
  ament_add_gtest(test_yasmin_action test/test_yasmin_action.cpp)
  target_include_directories(test_yasmin_action PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_yasmin_action
    ${PROJECT_NAME}
    ${DEPENDENCIES}
  )

  # Test for YasminFactoryAction class
  ament_add_gtest(test_yasmin_factory_action test/test_yasmin_factory_action.cpp)
  target_include_directories(test_yasmin_factory_action PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_yasmin_factory_action
    ${PROJECT_NAME}
    ${DEPENDENCIES}
  )
endif()

ament_package()