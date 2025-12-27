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
  "msg/KnowledgeUpdate.msg"
  "srv/GetTypes.srv"
  "srv/AddType.srv"
  "srv/RemoveType.srv"
  "srv/GetObjects.srv"
  "srv/AddObject.srv"
  "srv/RemoveObject.srv"
  "srv/GetPredicates.srv"
  "srv/AddPredicate.srv"
  "srv/RemovePredicate.srv"
  "srv/GetFacts.srv"
  "srv/AddFact.srv"
  "srv/RemoveFact.srv"
  "srv/GetGoals.srv"
  "srv/AddGoal.srv"
  "srv/RemoveGoal.srv"
  "srv/ClearKnowledgeBase.srv"
)

ament_package()
