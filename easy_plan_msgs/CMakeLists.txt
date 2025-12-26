cmake_minimum_required(VERSION 3.5)
project(easy_plan_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Action.msg"
  "msg/PlanAction.msg"
  "msg/Plan.msg"
  "msg/Predicate.msg"
  "msg/Object.msg"
  "msg/Domain.msg"
  "msg/Problem.msg"
)

ament_package()
