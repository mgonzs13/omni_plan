cmake_minimum_required(VERSION 3.5)
project(omni_plan_msgs)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Predicate.msg"
  "msg/Domain.msg"
  "msg/Object.msg"
  "msg/Problem.msg"
  "msg/Action.msg"
  "msg/PlanAction.msg"
  "msg/Plan.msg"
  "msg/PDDL.msg"
  "msg/KnowledgeUpdate.msg"
  "srv/GetTypes.srv"
  "srv/AddType.srv"
  "srv/AddTypes.srv"
  "srv/RemoveType.srv"
  "srv/RemoveTypes.srv"
  "srv/GetObjects.srv"
  "srv/AddObject.srv"
  "srv/AddObjects.srv"
  "srv/RemoveObject.srv"
  "srv/RemoveObjects.srv"
  "srv/GetPredicates.srv"
  "srv/AddPredicate.srv"
  "srv/AddPredicates.srv"
  "srv/RemovePredicate.srv"
  "srv/RemovePredicates.srv"
  "srv/GetFacts.srv"
  "srv/AddFact.srv"
  "srv/AddFacts.srv"
  "srv/RemoveFact.srv"
  "srv/RemoveFacts.srv"
  "srv/GetGoals.srv"
  "srv/AddGoal.srv"
  "srv/AddGoals.srv"
  "srv/RemoveGoal.srv"
  "srv/RemoveGoals.srv"
  "srv/ClearKnowledgeBase.srv"
)

ament_package()
