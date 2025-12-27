// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include <functional>

#include "easy_plan_knowledge_base/knowledge_base_exceptions.hpp"
#include "easy_plan_knowledge_base/knowledge_base_node.hpp"

using namespace easy_plan_knowledge_base;
using std::placeholders::_1;
using std::placeholders::_2;

KnowledgeBaseNode::KnowledgeBaseNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("knowledge_base_node", options),
      knowledge_base_(std::make_shared<KnowledgeBase>()) {

  // Initialize publisher for knowledge updates
  this->knowledge_update_pub_ =
      this->create_publisher<easy_plan_msgs::msg::KnowledgeUpdate>(
          "knowledge_updates", 10);

  // Create type services
  this->get_types_srv_ = this->create_service<easy_plan_msgs::srv::GetTypes>(
      "get_types",
      std::bind(&KnowledgeBaseNode::get_types_callback, this, _1, _2));
  this->add_type_srv_ = this->create_service<easy_plan_msgs::srv::AddType>(
      "add_type",
      std::bind(&KnowledgeBaseNode::add_type_callback, this, _1, _2));
  this->remove_type_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveType>(
          "remove_type",
          std::bind(&KnowledgeBaseNode::remove_type_callback, this, _1, _2));

  // Create object services
  this->get_objects_srv_ =
      this->create_service<easy_plan_msgs::srv::GetObjects>(
          "get_objects",
          std::bind(&KnowledgeBaseNode::get_objects_callback, this, _1, _2));
  this->add_object_srv_ = this->create_service<easy_plan_msgs::srv::AddObject>(
      "add_object",
      std::bind(&KnowledgeBaseNode::add_object_callback, this, _1, _2));
  this->remove_object_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveObject>(
          "remove_object",
          std::bind(&KnowledgeBaseNode::remove_object_callback, this, _1, _2));

  // Create predicate services
  this->get_predicates_srv_ =
      this->create_service<easy_plan_msgs::srv::GetPredicates>(
          "get_predicates",
          std::bind(&KnowledgeBaseNode::get_predicates_callback, this, _1, _2));
  this->add_predicate_srv_ =
      this->create_service<easy_plan_msgs::srv::AddPredicate>(
          "add_predicate",
          std::bind(&KnowledgeBaseNode::add_predicate_callback, this, _1, _2));
  this->remove_predicate_srv_ =
      this->create_service<easy_plan_msgs::srv::RemovePredicate>(
          "remove_predicate",
          std::bind(&KnowledgeBaseNode::remove_predicate_callback, this, _1,
                    _2));

  // Create fact services
  this->get_facts_srv_ = this->create_service<easy_plan_msgs::srv::GetFacts>(
      "get_facts",
      std::bind(&KnowledgeBaseNode::get_facts_callback, this, _1, _2));
  this->add_fact_srv_ = this->create_service<easy_plan_msgs::srv::AddFact>(
      "add_fact",
      std::bind(&KnowledgeBaseNode::add_fact_callback, this, _1, _2));
  this->remove_fact_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveFact>(
          "remove_fact",
          std::bind(&KnowledgeBaseNode::remove_fact_callback, this, _1, _2));

  // Create goal services
  this->get_goals_srv_ = this->create_service<easy_plan_msgs::srv::GetGoals>(
      "get_goals",
      std::bind(&KnowledgeBaseNode::get_goals_callback, this, _1, _2));
  this->add_goal_srv_ = this->create_service<easy_plan_msgs::srv::AddGoal>(
      "add_goal",
      std::bind(&KnowledgeBaseNode::add_goal_callback, this, _1, _2));
  this->remove_goal_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveGoal>(
          "remove_goal",
          std::bind(&KnowledgeBaseNode::remove_goal_callback, this, _1, _2));

  // Create clear service
  this->clear_srv_ =
      this->create_service<easy_plan_msgs::srv::ClearKnowledgeBase>(
          "clear", std::bind(&KnowledgeBaseNode::clear_callback, this, _1, _2));

  // Create batch services
  this->add_types_srv_ = this->create_service<easy_plan_msgs::srv::AddTypes>(
      "add_types",
      std::bind(&KnowledgeBaseNode::add_types_callback, this, _1, _2));
  this->remove_types_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveTypes>(
          "remove_types",
          std::bind(&KnowledgeBaseNode::remove_types_callback, this, _1, _2));

  this->add_objects_srv_ =
      this->create_service<easy_plan_msgs::srv::AddObjects>(
          "add_objects",
          std::bind(&KnowledgeBaseNode::add_objects_callback, this, _1, _2));
  this->remove_objects_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveObjects>(
          "remove_objects",
          std::bind(&KnowledgeBaseNode::remove_objects_callback, this, _1, _2));

  this->add_predicates_srv_ =
      this->create_service<easy_plan_msgs::srv::AddPredicates>(
          "add_predicates",
          std::bind(&KnowledgeBaseNode::add_predicates_callback, this, _1, _2));
  this->remove_predicates_srv_ =
      this->create_service<easy_plan_msgs::srv::RemovePredicates>(
          "remove_predicates",
          std::bind(&KnowledgeBaseNode::remove_predicates_callback, this, _1,
                    _2));

  this->add_facts_srv_ = this->create_service<easy_plan_msgs::srv::AddFacts>(
      "add_facts",
      std::bind(&KnowledgeBaseNode::add_facts_callback, this, _1, _2));
  this->remove_facts_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveFacts>(
          "remove_facts",
          std::bind(&KnowledgeBaseNode::remove_facts_callback, this, _1, _2));

  this->add_goals_srv_ = this->create_service<easy_plan_msgs::srv::AddGoals>(
      "add_goals",
      std::bind(&KnowledgeBaseNode::add_goals_callback, this, _1, _2));
  this->remove_goals_srv_ =
      this->create_service<easy_plan_msgs::srv::RemoveGoals>(
          "remove_goals",
          std::bind(&KnowledgeBaseNode::remove_goals_callback, this, _1, _2));

  RCLCPP_INFO(this->get_logger(), "KnowledgeBaseNode initialized");
}

void KnowledgeBaseNode::publish_knowledge_update(int8_t operation,
                                                 int8_t entity_type) {
  easy_plan_msgs::msg::KnowledgeUpdate update_msg;
  update_msg.operation = operation;
  update_msg.entity_type = entity_type;
  this->knowledge_update_pub_->publish(update_msg);
}

// ==================== Type Service Callbacks ====================
void KnowledgeBaseNode::get_types_callback(
    const std::shared_ptr<easy_plan_msgs::srv::GetTypes::Request> /*request*/,
    std::shared_ptr<easy_plan_msgs::srv::GetTypes::Response> response) {
  // Get types from knowledge base and convert from set to vector
  response->types =
      std::vector<std::string>(this->knowledge_base_->get_types().begin(),
                               this->knowledge_base_->get_types().end());
}

void KnowledgeBaseNode::add_type_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddType::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddType::Response> response) {
  response->success = this->knowledge_base_->add_type(request->type);

  if (response->success) {
    // Publish knowledge update
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::ADD,
                                   easy_plan_msgs::msg::KnowledgeUpdate::TYPE);
  }
}

void KnowledgeBaseNode::remove_type_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveType::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveType::Response> response) {
  response->success = this->knowledge_base_->remove_type(request->type);

  if (response->success) {
    // Publish knowledge update
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
                                   easy_plan_msgs::msg::KnowledgeUpdate::TYPE);
  }
}

// ==================== Object Service Callbacks ====================
void KnowledgeBaseNode::get_objects_callback(
    const std::shared_ptr<easy_plan_msgs::srv::GetObjects::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::GetObjects::Response> response) {
  std::set<easy_plan::pddl::Object> objects;
  if (request->type.empty()) {
    objects = this->knowledge_base_->get_objects();
  } else {
    objects = this->knowledge_base_->get_objects_by_type(request->type);
  }

  for (const auto &obj : objects) {
    response->objects.push_back(obj.to_msg());
  }
}

void KnowledgeBaseNode::add_object_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddObject::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddObject::Response> response) {
  try {
    auto obj = msg_to_object(request->object);
    response->success = this->knowledge_base_->add_object(obj);

    if (response->success) {
      // Publish knowledge update
      this->publish_knowledge_update(
          easy_plan_msgs::msg::KnowledgeUpdate::ADD,
          easy_plan_msgs::msg::KnowledgeUpdate::OBJECT);
    }

  } catch (const easy_plan_knowledge_base::TypeNotFoundException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add object: %s", e.what());
    response->success = false;
  }
}

void KnowledgeBaseNode::remove_object_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveObject::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveObject::Response> response) {
  auto obj = msg_to_object(request->object);
  response->success = this->knowledge_base_->remove_object(obj);

  if (response->success) {
    // Publish knowledge update
    this->publish_knowledge_update(
        easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
        easy_plan_msgs::msg::KnowledgeUpdate::OBJECT);
  }
}

// ==================== Predicate Service Callbacks ====================
void KnowledgeBaseNode::get_predicates_callback(
    const std::shared_ptr<easy_plan_msgs::srv::GetPredicates::Request>
    /*request*/,
    std::shared_ptr<easy_plan_msgs::srv::GetPredicates::Response> response) {
  auto predicates = this->knowledge_base_->get_predicates();
  for (const auto &pred : predicates) {
    response->predicates.push_back(pred.to_msg());
  }
}

void KnowledgeBaseNode::add_predicate_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddPredicate::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddPredicate::Response> response) {
  try {
    auto pred = msg_to_predicate(request->predicate);
    response->success = this->knowledge_base_->add_predicate(pred);

    if (response->success) {
      // Publish knowledge update
      this->publish_knowledge_update(
          easy_plan_msgs::msg::KnowledgeUpdate::ADD,
          easy_plan_msgs::msg::KnowledgeUpdate::PREDICATE);
    }

  } catch (const easy_plan_knowledge_base::TypeNotFoundException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add predicate: %s", e.what());
    response->success = false;
  }
}

void KnowledgeBaseNode::remove_predicate_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemovePredicate::Request>
        request,
    std::shared_ptr<easy_plan_msgs::srv::RemovePredicate::Response> response) {
  auto pred = msg_to_predicate(request->predicate);
  response->success = this->knowledge_base_->remove_predicate(pred);

  if (response->success) {
    // Publish knowledge update
    this->publish_knowledge_update(
        easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
        easy_plan_msgs::msg::KnowledgeUpdate::PREDICATE);
  }
}

// ==================== Fact Service Callbacks ====================
void KnowledgeBaseNode::get_facts_callback(
    const std::shared_ptr<easy_plan_msgs::srv::GetFacts::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::GetFacts::Response> response) {
  std::set<easy_plan::pddl::Predicate> facts;
  if (request->name.empty()) {
    facts = this->knowledge_base_->get_facts();
  } else {
    facts = this->knowledge_base_->get_facts_by_name(request->name);
  }

  for (const auto &fact : facts) {
    response->facts.push_back(fact.to_msg());
  }
}

void KnowledgeBaseNode::add_fact_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddFact::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddFact::Response> response) {
  try {
    auto fact = msg_to_predicate(request->fact);
    response->success = this->knowledge_base_->add_fact(fact);

    if (response->success) {
      // Publish knowledge update
      this->publish_knowledge_update(
          easy_plan_msgs::msg::KnowledgeUpdate::ADD,
          easy_plan_msgs::msg::KnowledgeUpdate::FACT);
    }

  } catch (const easy_plan_knowledge_base::PredicateNotFoundException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add fact: %s", e.what());
    response->success = false;
  } catch (const easy_plan_knowledge_base::ObjectNotFoundException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add fact: %s", e.what());
    response->success = false;
  }
}

void KnowledgeBaseNode::remove_fact_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveFact::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveFact::Response> response) {
  auto fact = msg_to_predicate(request->fact);
  response->success = this->knowledge_base_->remove_fact(fact);

  if (response->success) {
    // Publish knowledge update
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
                                   easy_plan_msgs::msg::KnowledgeUpdate::FACT);
  }
}

// ==================== Goal Service Callbacks ====================
void KnowledgeBaseNode::get_goals_callback(
    const std::shared_ptr<easy_plan_msgs::srv::GetGoals::Request> /*request*/,
    std::shared_ptr<easy_plan_msgs::srv::GetGoals::Response> response) {
  auto goals = this->knowledge_base_->get_goals();
  for (const auto &goal : goals) {
    response->goals.push_back(goal.to_msg());
  }
}

void KnowledgeBaseNode::add_goal_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddGoal::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddGoal::Response> response) {
  try {
    auto goal = msg_to_predicate(request->goal);
    response->success = this->knowledge_base_->add_goal(goal);

    if (response->success) {
      // Publish knowledge update
      this->publish_knowledge_update(
          easy_plan_msgs::msg::KnowledgeUpdate::ADD,
          easy_plan_msgs::msg::KnowledgeUpdate::GOAL);
    }

  } catch (const easy_plan_knowledge_base::PredicateNotFoundException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add goal: %s", e.what());
    response->success = false;
  } catch (const easy_plan_knowledge_base::ObjectNotFoundException &e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to add goal: %s", e.what());
    response->success = false;
  }
}

void KnowledgeBaseNode::remove_goal_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveGoal::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveGoal::Response> response) {
  auto goal = msg_to_predicate(request->goal);
  response->success = this->knowledge_base_->remove_goal(goal);

  if (response->success) {
    // Publish knowledge update
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
                                   easy_plan_msgs::msg::KnowledgeUpdate::GOAL);
  }
}

// ==================== Batch Type Service Callbacks ====================
void KnowledgeBaseNode::add_types_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddTypes::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddTypes::Response> response) {
  response->success = true;
  for (const auto &type : request->types) {
    if (!this->knowledge_base_->add_type(type)) {
      response->success = false;
    }
  }

  if (response->success && !request->types.empty()) {
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::ADD,
                                   easy_plan_msgs::msg::KnowledgeUpdate::TYPE);
  }
}

void KnowledgeBaseNode::remove_types_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveTypes::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveTypes::Response> response) {
  response->success = true;
  for (const auto &type : request->types) {
    if (!this->knowledge_base_->remove_type(type)) {
      response->success = false;
    }
  }

  if (response->success && !request->types.empty()) {
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
                                   easy_plan_msgs::msg::KnowledgeUpdate::TYPE);
  }
}

// ==================== Batch Object Service Callbacks ====================
void KnowledgeBaseNode::add_objects_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddObjects::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddObjects::Response> response) {
  response->success = true;
  for (const auto &obj_msg : request->objects) {
    try {
      auto obj = this->msg_to_object(obj_msg);
      if (!this->knowledge_base_->add_object(obj)) {
        response->success = false;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add object: %s", e.what());
      response->success = false;
    }
  }

  if (response->success && !request->objects.empty()) {
    this->publish_knowledge_update(
        easy_plan_msgs::msg::KnowledgeUpdate::ADD,
        easy_plan_msgs::msg::KnowledgeUpdate::OBJECT);
  }
}

void KnowledgeBaseNode::remove_objects_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveObjects::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveObjects::Response> response) {
  response->success = true;
  for (const auto &obj_msg : request->objects) {
    auto obj = this->msg_to_object(obj_msg);
    if (!this->knowledge_base_->remove_object(obj)) {
      response->success = false;
    }
  }

  if (response->success && !request->objects.empty()) {
    this->publish_knowledge_update(
        easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
        easy_plan_msgs::msg::KnowledgeUpdate::OBJECT);
  }
}

// ==================== Batch Predicate Service Callbacks ====================
void KnowledgeBaseNode::add_predicates_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddPredicates::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddPredicates::Response> response) {
  response->success = true;
  for (const auto &pred_msg : request->predicates) {
    auto pred = this->msg_to_predicate(pred_msg);
    if (!this->knowledge_base_->add_predicate(pred)) {
      response->success = false;
    }
  }

  if (response->success && !request->predicates.empty()) {
    this->publish_knowledge_update(
        easy_plan_msgs::msg::KnowledgeUpdate::ADD,
        easy_plan_msgs::msg::KnowledgeUpdate::PREDICATE);
  }
}

void KnowledgeBaseNode::remove_predicates_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemovePredicates::Request>
        request,
    std::shared_ptr<easy_plan_msgs::srv::RemovePredicates::Response> response) {
  response->success = true;
  for (const auto &pred_msg : request->predicates) {
    auto pred = this->msg_to_predicate(pred_msg);
    if (!this->knowledge_base_->remove_predicate(pred)) {
      response->success = false;
    }
  }

  if (response->success && !request->predicates.empty()) {
    this->publish_knowledge_update(
        easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
        easy_plan_msgs::msg::KnowledgeUpdate::PREDICATE);
  }
}

// ==================== Batch Fact Service Callbacks ====================
void KnowledgeBaseNode::add_facts_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddFacts::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddFacts::Response> response) {
  response->success = true;
  for (const auto &fact_msg : request->facts) {
    try {
      auto fact = this->msg_to_predicate(fact_msg);
      if (!this->knowledge_base_->add_fact(fact)) {
        response->success = false;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add fact: %s", e.what());
      response->success = false;
    }
  }

  if (response->success && !request->facts.empty()) {
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::ADD,
                                   easy_plan_msgs::msg::KnowledgeUpdate::FACT);
  }
}

void KnowledgeBaseNode::remove_facts_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveFacts::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveFacts::Response> response) {
  response->success = true;
  for (const auto &fact_msg : request->facts) {
    auto fact = this->msg_to_predicate(fact_msg);
    if (!this->knowledge_base_->remove_fact(fact)) {
      response->success = false;
    }
  }

  if (response->success && !request->facts.empty()) {
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
                                   easy_plan_msgs::msg::KnowledgeUpdate::FACT);
  }
}

// ==================== Batch Goal Service Callbacks ====================
void KnowledgeBaseNode::add_goals_callback(
    const std::shared_ptr<easy_plan_msgs::srv::AddGoals::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::AddGoals::Response> response) {
  response->success = true;
  for (const auto &goal_msg : request->goals) {
    try {
      auto goal = this->msg_to_predicate(goal_msg);
      if (!this->knowledge_base_->add_goal(goal)) {
        response->success = false;
      }
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to add goal: %s", e.what());
      response->success = false;
    }
  }

  if (response->success && !request->goals.empty()) {
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::ADD,
                                   easy_plan_msgs::msg::KnowledgeUpdate::GOAL);
  }
}

void KnowledgeBaseNode::remove_goals_callback(
    const std::shared_ptr<easy_plan_msgs::srv::RemoveGoals::Request> request,
    std::shared_ptr<easy_plan_msgs::srv::RemoveGoals::Response> response) {
  response->success = true;
  for (const auto &goal_msg : request->goals) {
    auto goal = this->msg_to_predicate(goal_msg);
    if (!this->knowledge_base_->remove_goal(goal)) {
      response->success = false;
    }
  }

  if (response->success && !request->goals.empty()) {
    this->publish_knowledge_update(easy_plan_msgs::msg::KnowledgeUpdate::REMOVE,
                                   easy_plan_msgs::msg::KnowledgeUpdate::GOAL);
  }
}

// ==================== Clear Service Callback ====================
void KnowledgeBaseNode::clear_callback(
    const std::shared_ptr<easy_plan_msgs::srv::ClearKnowledgeBase::Request>
        request,
    std::shared_ptr<easy_plan_msgs::srv::ClearKnowledgeBase::Response>
        response) {
  (void)request;
  this->knowledge_base_->clear();
  response->success = true;
}

// ==================== Helper Methods ====================

easy_plan::pddl::Object
KnowledgeBaseNode::msg_to_object(const easy_plan_msgs::msg::Object &msg) const {
  return easy_plan::pddl::Object(msg.name, msg.type);
}

easy_plan::pddl::Predicate KnowledgeBaseNode::msg_to_predicate(
    const easy_plan_msgs::msg::Predicate &msg) const {
  std::vector<std::string> args(msg.arguments.begin(), msg.arguments.end());
  return easy_plan::pddl::Predicate(msg.name, args, msg.negated);
}
