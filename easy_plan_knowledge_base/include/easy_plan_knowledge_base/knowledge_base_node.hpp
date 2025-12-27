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

#ifndef EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_NODE_HPP_
#define EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_NODE_HPP_

#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "easy_plan_knowledge_base/knowledge_base.hpp"

#include "easy_plan_msgs/msg/knowledge_update.hpp"
#include "easy_plan_msgs/srv/add_fact.hpp"
#include "easy_plan_msgs/srv/add_facts.hpp"
#include "easy_plan_msgs/srv/add_goal.hpp"
#include "easy_plan_msgs/srv/add_goals.hpp"
#include "easy_plan_msgs/srv/add_object.hpp"
#include "easy_plan_msgs/srv/add_objects.hpp"
#include "easy_plan_msgs/srv/add_predicate.hpp"
#include "easy_plan_msgs/srv/add_predicates.hpp"
#include "easy_plan_msgs/srv/add_type.hpp"
#include "easy_plan_msgs/srv/add_types.hpp"
#include "easy_plan_msgs/srv/clear_knowledge_base.hpp"
#include "easy_plan_msgs/srv/get_facts.hpp"
#include "easy_plan_msgs/srv/get_goals.hpp"
#include "easy_plan_msgs/srv/get_objects.hpp"
#include "easy_plan_msgs/srv/get_predicates.hpp"
#include "easy_plan_msgs/srv/get_types.hpp"
#include "easy_plan_msgs/srv/remove_fact.hpp"
#include "easy_plan_msgs/srv/remove_facts.hpp"
#include "easy_plan_msgs/srv/remove_goal.hpp"
#include "easy_plan_msgs/srv/remove_goals.hpp"
#include "easy_plan_msgs/srv/remove_object.hpp"
#include "easy_plan_msgs/srv/remove_objects.hpp"
#include "easy_plan_msgs/srv/remove_predicate.hpp"
#include "easy_plan_msgs/srv/remove_predicates.hpp"
#include "easy_plan_msgs/srv/remove_type.hpp"
#include "easy_plan_msgs/srv/remove_types.hpp"

namespace easy_plan_knowledge_base {

/**
 * @class KnowledgeBaseNode
 * @brief ROS 2 node that exposes KnowledgeBase functionality through services.
 * @details This node provides ROS 2 service interfaces for managing a knowledge
 * base containing types, objects, predicates, facts, and goals.
 */
class KnowledgeBaseNode : public rclcpp::Node {
public:
  /**
   * @brief Constructor for KnowledgeBaseNode.
   * @param options Node options for configuration.
   */
  explicit KnowledgeBaseNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions());

private:
  /**
   * @brief Publishes a knowledge update message.
   * @param operation The operation performed (ADD, REMOVE, CLEAR).
   * @param entity_type The type of entity affected (TYPE, OBJECT, etc.).
   */
  void publish_knowledge_update(int8_t operation, int8_t entity_type);

  // ==================== Service Callbacks ====================
  // Type services
  void get_types_callback(
      const std::shared_ptr<easy_plan_msgs::srv::GetTypes::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::GetTypes::Response> response);

  void add_type_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddType::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddType::Response> response);

  void remove_type_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveType::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveType::Response> response);

  // Batch type services
  void add_types_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddTypes::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddTypes::Response> response);

  void remove_types_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveTypes::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveTypes::Response> response);

  // Object services
  void get_objects_callback(
      const std::shared_ptr<easy_plan_msgs::srv::GetObjects::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::GetObjects::Response> response);

  void add_object_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddObject::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddObject::Response> response);

  void remove_object_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveObject::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveObject::Response> response);

  // Batch object services
  void add_objects_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddObjects::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddObjects::Response> response);

  void remove_objects_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveObjects::Request>
          request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveObjects::Response> response);

  // Predicate services
  void get_predicates_callback(
      const std::shared_ptr<easy_plan_msgs::srv::GetPredicates::Request>
          request,
      std::shared_ptr<easy_plan_msgs::srv::GetPredicates::Response> response);

  void add_predicate_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddPredicate::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddPredicate::Response> response);

  void remove_predicate_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemovePredicate::Request>
          request,
      std::shared_ptr<easy_plan_msgs::srv::RemovePredicate::Response> response);

  // Batch predicate services
  void add_predicates_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddPredicates::Request>
          request,
      std::shared_ptr<easy_plan_msgs::srv::AddPredicates::Response> response);

  void remove_predicates_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemovePredicates::Request>
          request,
      std::shared_ptr<easy_plan_msgs::srv::RemovePredicates::Response>
          response);

  // Fact services
  void get_facts_callback(
      const std::shared_ptr<easy_plan_msgs::srv::GetFacts::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::GetFacts::Response> response);

  void add_fact_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddFact::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddFact::Response> response);

  void remove_fact_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveFact::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveFact::Response> response);

  // Batch fact services
  void add_facts_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddFacts::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddFacts::Response> response);

  void remove_facts_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveFacts::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveFacts::Response> response);

  // Goal services
  void get_goals_callback(
      const std::shared_ptr<easy_plan_msgs::srv::GetGoals::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::GetGoals::Response> response);

  void add_goal_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddGoal::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddGoal::Response> response);

  void remove_goal_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveGoal::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveGoal::Response> response);

  // Batch goal services
  void add_goals_callback(
      const std::shared_ptr<easy_plan_msgs::srv::AddGoals::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::AddGoals::Response> response);

  void remove_goals_callback(
      const std::shared_ptr<easy_plan_msgs::srv::RemoveGoals::Request> request,
      std::shared_ptr<easy_plan_msgs::srv::RemoveGoals::Response> response);

  // Clear service
  void clear_callback(
      const std::shared_ptr<easy_plan_msgs::srv::ClearKnowledgeBase::Request>
          request,
      std::shared_ptr<easy_plan_msgs::srv::ClearKnowledgeBase::Response>
          response);

  // ==================== Helper Methods ====================
  /**
   * @brief Converts a message Object to a pddl Object.
   * @param msg The message to convert.
   * @return The converted pddl Object.
   */
  easy_plan::pddl::Object
  msg_to_object(const easy_plan_msgs::msg::Object &msg) const;

  /**
   * @brief Converts a message Predicate to a pddl Predicate.
   * @param msg The message to convert.
   * @return The converted pddl Predicate.
   */
  easy_plan::pddl::Predicate
  msg_to_predicate(const easy_plan_msgs::msg::Predicate &msg) const;

  // ==================== Member Variables ====================
  /// The underlying knowledge base.
  std::shared_ptr<KnowledgeBase> knowledge_base_;

  // Publisher for knowledge updates
  rclcpp::Publisher<easy_plan_msgs::msg::KnowledgeUpdate>::SharedPtr
      knowledge_update_pub_;

  // Service servers
  rclcpp::Service<easy_plan_msgs::srv::GetTypes>::SharedPtr get_types_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddType>::SharedPtr add_type_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveType>::SharedPtr remove_type_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddTypes>::SharedPtr add_types_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveTypes>::SharedPtr
      remove_types_srv_;

  rclcpp::Service<easy_plan_msgs::srv::GetObjects>::SharedPtr get_objects_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddObject>::SharedPtr add_object_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveObject>::SharedPtr
      remove_object_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddObjects>::SharedPtr add_objects_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveObjects>::SharedPtr
      remove_objects_srv_;

  rclcpp::Service<easy_plan_msgs::srv::GetPredicates>::SharedPtr
      get_predicates_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddPredicate>::SharedPtr
      add_predicate_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemovePredicate>::SharedPtr
      remove_predicate_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddPredicates>::SharedPtr
      add_predicates_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemovePredicates>::SharedPtr
      remove_predicates_srv_;

  rclcpp::Service<easy_plan_msgs::srv::GetFacts>::SharedPtr get_facts_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddFact>::SharedPtr add_fact_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveFact>::SharedPtr remove_fact_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddFacts>::SharedPtr add_facts_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveFacts>::SharedPtr
      remove_facts_srv_;

  rclcpp::Service<easy_plan_msgs::srv::GetGoals>::SharedPtr get_goals_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddGoal>::SharedPtr add_goal_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveGoal>::SharedPtr remove_goal_srv_;
  rclcpp::Service<easy_plan_msgs::srv::AddGoals>::SharedPtr add_goals_srv_;
  rclcpp::Service<easy_plan_msgs::srv::RemoveGoals>::SharedPtr
      remove_goals_srv_;

  rclcpp::Service<easy_plan_msgs::srv::ClearKnowledgeBase>::SharedPtr
      clear_srv_;
};

} // namespace easy_plan_knowledge_base

#endif // EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_NODE_HPP_
