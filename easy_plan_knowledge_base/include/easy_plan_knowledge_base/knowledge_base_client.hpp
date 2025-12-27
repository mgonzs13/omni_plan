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

#ifndef EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_CLIENT_HPP_
#define EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_CLIENT_HPP_

#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/predicate.hpp"

#include "easy_plan_msgs/msg/knowledge_update.hpp"
#include "easy_plan_msgs/msg/object.hpp"
#include "easy_plan_msgs/msg/predicate.hpp"
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
 * @class KnowledgeBaseClient
 * @brief Client interface for interacting with the knowledge base.
 * @details Provides a simplified API for all knowledge base operations with
 * automatic executor management in a separate thread.
 */
class KnowledgeBaseClient {
public:
  using KnowledgeUpdateCallback = std::function<void(
      const easy_plan_msgs::msg::KnowledgeUpdate::SharedPtr)>;

  /**
   * @brief Constructor.
   * @param node_name Name for the client node.
   */
  explicit KnowledgeBaseClient(const std::string &node_name = "kb_client");

  /**
   * @brief Destructor - stops executor thread.
   */
  ~KnowledgeBaseClient();

  // ==================== Type Operations ====================
  bool add_type(const std::string &type);
  bool add_types(const std::vector<std::string> &types);
  bool remove_type(const std::string &type);
  bool remove_types(const std::vector<std::string> &types);
  std::vector<std::string> get_types();

  // ==================== Object Operations ====================
  bool add_object(const std::string &name, const std::string &type);
  bool add_object(const easy_plan::pddl::Object &object);
  bool add_objects(const std::vector<easy_plan::pddl::Object> &objects);
  bool remove_object(const std::string &name, const std::string &type);
  bool remove_object(const easy_plan::pddl::Object &object);
  bool remove_objects(const std::vector<easy_plan::pddl::Object> &objects);
  std::vector<easy_plan::pddl::Object> get_objects();

  // ==================== Predicate Operations ====================
  bool add_predicate(const std::string &name,
                     const std::vector<std::string> &args);
  bool add_predicate(const easy_plan::pddl::Predicate &predicate);
  bool
  add_predicates(const std::vector<easy_plan::pddl::Predicate> &predicates);
  bool remove_predicate(const std::string &name,
                        const std::vector<std::string> &args);
  bool remove_predicate(const easy_plan::pddl::Predicate &predicate);
  bool
  remove_predicates(const std::vector<easy_plan::pddl::Predicate> &predicates);
  std::vector<easy_plan::pddl::Predicate> get_predicates();

  // ==================== Fact Operations ====================
  bool add_fact(const std::string &name, const std::vector<std::string> &args);
  bool add_fact(const easy_plan::pddl::Predicate &fact);
  bool add_facts(const std::vector<easy_plan::pddl::Predicate> &facts);
  bool remove_fact(const std::string &name,
                   const std::vector<std::string> &args);
  bool remove_fact(const easy_plan::pddl::Predicate &fact);
  bool remove_facts(const std::vector<easy_plan::pddl::Predicate> &facts);
  std::vector<easy_plan::pddl::Predicate>
  get_facts(const std::string &name = "");

  // ==================== Goal Operations ====================
  bool add_goal(const std::string &name, const std::vector<std::string> &args);
  bool add_goal(const easy_plan::pddl::Predicate &goal);
  bool add_goals(const std::vector<easy_plan::pddl::Predicate> &goals);
  bool remove_goal(const std::string &name,
                   const std::vector<std::string> &args);
  bool remove_goal(const easy_plan::pddl::Predicate &goal);
  bool remove_goals(const std::vector<easy_plan::pddl::Predicate> &goals);
  std::vector<easy_plan::pddl::Predicate> get_goals();
  bool has_goals();

  // ==================== Callback Management ====================
  /**
   * @brief Add a callback for knowledge update notifications.
   * @param callback Function to call when knowledge is updated.
   */
  void add_knowledge_update_callback(KnowledgeUpdateCallback callback);

private:
  /**
   * @brief Internal callback that forwards to registered callbacks.
   */
  void knowledge_update_callback(
      const easy_plan_msgs::msg::KnowledgeUpdate::SharedPtr msg);

  /**
   * @brief Convert pddl::Object to msg::Object.
   */
  easy_plan_msgs::msg::Object
  object_to_msg(const easy_plan::pddl::Object &object) const;

  /**
   * @brief Convert msg::Object to pddl::Object.
   */
  easy_plan::pddl::Object
  msg_to_object(const easy_plan_msgs::msg::Object &msg) const;

  /**
   * @brief Convert pddl::Predicate to msg::Predicate.
   */
  easy_plan_msgs::msg::Predicate
  predicate_to_msg(const easy_plan::pddl::Predicate &predicate) const;

  /**
   * @brief Convert msg::Predicate to pddl::Predicate.
   */
  easy_plan::pddl::Predicate
  msg_to_predicate(const easy_plan_msgs::msg::Predicate &msg) const;

  rclcpp::Node::SharedPtr node_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // Service clients
  rclcpp::Client<easy_plan_msgs::srv::GetTypes>::SharedPtr get_types_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddType>::SharedPtr add_type_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddTypes>::SharedPtr add_types_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveType>::SharedPtr
      remove_type_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveTypes>::SharedPtr
      remove_types_client_;

  rclcpp::Client<easy_plan_msgs::srv::GetObjects>::SharedPtr
      get_objects_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddObject>::SharedPtr add_object_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddObjects>::SharedPtr
      add_objects_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveObject>::SharedPtr
      remove_object_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveObjects>::SharedPtr
      remove_objects_client_;

  rclcpp::Client<easy_plan_msgs::srv::GetPredicates>::SharedPtr
      get_predicates_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddPredicate>::SharedPtr
      add_predicate_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddPredicates>::SharedPtr
      add_predicates_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemovePredicate>::SharedPtr
      remove_predicate_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemovePredicates>::SharedPtr
      remove_predicates_client_;

  rclcpp::Client<easy_plan_msgs::srv::GetFacts>::SharedPtr get_facts_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddFact>::SharedPtr add_fact_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddFacts>::SharedPtr add_facts_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveFact>::SharedPtr
      remove_fact_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveFacts>::SharedPtr
      remove_facts_client_;

  rclcpp::Client<easy_plan_msgs::srv::GetGoals>::SharedPtr get_goals_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddGoal>::SharedPtr add_goal_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddGoals>::SharedPtr add_goals_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveGoal>::SharedPtr
      remove_goal_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveGoals>::SharedPtr
      remove_goals_client_;

  // Subscription
  rclcpp::Subscription<easy_plan_msgs::msg::KnowledgeUpdate>::SharedPtr
      knowledge_update_sub_;

  // Callbacks
  std::vector<KnowledgeUpdateCallback> callbacks_;
};

} // namespace easy_plan_knowledge_base

#endif // EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_CLIENT_HPP_
