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
  /**
   * @brief Adds a type to the knowledge base.
   * @param type The type name to add.
   * @return True if successful, false otherwise.
   */
  bool add_type(const std::string &type);

  /**
   * @brief Adds multiple types to the knowledge base.
   * @param types The type names to add.
   * @return True if successful, false otherwise.
   */
  bool add_types(const std::vector<std::string> &types);

  /**
   * @brief Removes a type from the knowledge base.
   * @param type The type name to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_type(const std::string &type);

  /**
   * @brief Removes multiple types from the knowledge base.
   * @param types The type names to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_types(const std::vector<std::string> &types);

  /**
   * @brief Gets all types from the knowledge base.
   * @return A vector of all type names.
   */
  std::vector<std::string> get_types();

  // ==================== Object Operations ====================
  /**
   * @brief Adds an object to the knowledge base.
   * @param name The object name.
   * @param type The object type.
   * @return True if successful, false otherwise.
   */
  bool add_object(const std::string &name, const std::string &type);

  /**
   * @brief Adds an object to the knowledge base.
   * @param object The object to add.
   * @return True if successful, false otherwise.
   */
  bool add_object(const easy_plan::pddl::Object &object);

  /**
   * @brief Adds multiple objects to the knowledge base.
   * @param objects The objects to add.
   * @return True if successful, false otherwise.
   */
  bool add_objects(const std::vector<easy_plan::pddl::Object> &objects);

  /**
   * @brief Removes an object from the knowledge base.
   * @param name The object name.
   * @param type The object type.
   * @return True if successful, false otherwise.
   */
  bool remove_object(const std::string &name, const std::string &type);

  /**
   * @brief Removes an object from the knowledge base.
   * @param object The object to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_object(const easy_plan::pddl::Object &object);

  /**
   * @brief Removes multiple objects from the knowledge base.
   * @param objects The objects to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_objects(const std::vector<easy_plan::pddl::Object> &objects);

  /**
   * @brief Gets all objects from the knowledge base.
   * @return A vector of all objects.
   */
  std::vector<easy_plan::pddl::Object> get_objects();

  // ==================== Predicate Operations ====================
  /**
   * @brief Adds a predicate definition to the knowledge base.
   * @param name The predicate name.
   * @param args The argument types.
   * @return True if successful, false otherwise.
   */
  bool add_predicate(const std::string &name,
                     const std::vector<std::string> &args);

  /**
   * @brief Adds a predicate definition to the knowledge base.
   * @param predicate The predicate to add.
   * @return True if successful, false otherwise.
   */
  bool add_predicate(const easy_plan::pddl::Predicate &predicate);

  /**
   * @brief Adds multiple predicate definitions to the knowledge base.
   * @param predicates The predicates to add.
   * @return True if successful, false otherwise.
   */
  bool
  add_predicates(const std::vector<easy_plan::pddl::Predicate> &predicates);

  /**
   * @brief Removes a predicate definition from the knowledge base.
   * @param name The predicate name.
   * @param args The argument types.
   * @return True if successful, false otherwise.
   */
  bool remove_predicate(const std::string &name,
                        const std::vector<std::string> &args);

  /**
   * @brief Removes a predicate definition from the knowledge base.
   * @param predicate The predicate to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_predicate(const easy_plan::pddl::Predicate &predicate);

  /**
   * @brief Removes multiple predicate definitions from the knowledge base.
   * @param predicates The predicates to remove.
   * @return True if successful, false otherwise.
   */
  bool
  remove_predicates(const std::vector<easy_plan::pddl::Predicate> &predicates);

  /**
   * @brief Gets all predicate definitions from the knowledge base.
   * @return A vector of all predicates.
   */
  std::vector<easy_plan::pddl::Predicate> get_predicates();

  // ==================== Fact Operations ====================
  /**
   * @brief Adds a fact to the knowledge base.
   * @param name The fact name.
   * @param args The fact arguments.
   * @return True if successful, false otherwise.
   */
  bool add_fact(const std::string &name, const std::vector<std::string> &args);

  /**
   * @brief Adds a fact to the knowledge base.
   * @param fact The fact to add.
   * @return True if successful, false otherwise.
   */
  bool add_fact(const easy_plan::pddl::Predicate &fact);

  /**
   * @brief Adds multiple facts to the knowledge base.
   * @param facts The facts to add.
   * @return True if successful, false otherwise.
   */
  bool add_facts(const std::vector<easy_plan::pddl::Predicate> &facts);

  /**
   * @brief Removes a fact from the knowledge base.
   * @param name The fact name.
   * @param args The fact arguments.
   * @return True if successful, false otherwise.
   */
  bool remove_fact(const std::string &name,
                   const std::vector<std::string> &args);

  /**
   * @brief Removes a fact from the knowledge base.
   * @param fact The fact to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_fact(const easy_plan::pddl::Predicate &fact);

  /**
   * @brief Removes multiple facts from the knowledge base.
   * @param facts The facts to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_facts(const std::vector<easy_plan::pddl::Predicate> &facts);

  /**
   * @brief Gets facts from the knowledge base.
   * @param name Optional predicate name to filter facts (default: all facts).
   * @return A vector of facts.
   */
  std::vector<easy_plan::pddl::Predicate>
  get_facts(const std::string &name = "");

  // ==================== Goal Operations ====================
  /**
   * @brief Adds a goal to the knowledge base.
   * @param name The goal name.
   * @param args The goal arguments.
   * @return True if successful, false otherwise.
   */
  bool add_goal(const std::string &name, const std::vector<std::string> &args);

  /**
   * @brief Adds a goal to the knowledge base.
   * @param goal The goal to add.
   * @return True if successful, false otherwise.
   */
  bool add_goal(const easy_plan::pddl::Predicate &goal);

  /**
   * @brief Adds multiple goals to the knowledge base.
   * @param goals The goals to add.
   * @return True if successful, false otherwise.
   */
  bool add_goals(const std::vector<easy_plan::pddl::Predicate> &goals);

  /**
   * @brief Removes a goal from the knowledge base.
   * @param name The goal name.
   * @param args The goal arguments.
   * @return True if successful, false otherwise.
   */
  bool remove_goal(const std::string &name,
                   const std::vector<std::string> &args);

  /**
   * @brief Removes a goal from the knowledge base.
   * @param goal The goal to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_goal(const easy_plan::pddl::Predicate &goal);

  /**
   * @brief Removes multiple goals from the knowledge base.
   * @param goals The goals to remove.
   * @return True if successful, false otherwise.
   */
  bool remove_goals(const std::vector<easy_plan::pddl::Predicate> &goals);

  /**
   * @brief Gets all goals from the knowledge base.
   * @return A vector of all goals.
   */
  std::vector<easy_plan::pddl::Predicate> get_goals();

  /**
   * @brief Checks if there are any goals in the knowledge base.
   * @return True if goals exist, false otherwise.
   */
  bool has_goals();

  // ==================== Callback Management ====================
  /**
   * @brief Add a callback for knowledge update notifications.
   * @param callback Function to call when knowledge is updated.
   */
  void add_knowledge_update_callback(KnowledgeUpdateCallback callback);

  /**
   * @brief Clears the entire knowledge base.
   * @return True if successful, false otherwise.
   */
  bool clear();

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

  /// @brief ROS 2 node for the client.
  rclcpp::Node::SharedPtr node_;
  /// @brief Single-threaded executor for handling service calls.
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  /// @brief Thread running the executor.
  std::thread executor_thread_;

  /// @brief Service client for getting types.
  rclcpp::Client<easy_plan_msgs::srv::GetTypes>::SharedPtr get_types_client_;
  /// @brief Service client for adding a type.
  rclcpp::Client<easy_plan_msgs::srv::AddType>::SharedPtr add_type_client_;
  /// @brief Service client for adding multiple types.
  rclcpp::Client<easy_plan_msgs::srv::AddTypes>::SharedPtr add_types_client_;
  /// @brief Service client for removing a type.
  rclcpp::Client<easy_plan_msgs::srv::RemoveType>::SharedPtr
      remove_type_client_;
  /// @brief Service client for removing multiple types.
  rclcpp::Client<easy_plan_msgs::srv::RemoveTypes>::SharedPtr
      remove_types_client_;

  /// @brief Service client for getting objects.
  rclcpp::Client<easy_plan_msgs::srv::GetObjects>::SharedPtr
      get_objects_client_;
  /// @brief Service client for adding an object.
  rclcpp::Client<easy_plan_msgs::srv::AddObject>::SharedPtr add_object_client_;
  /// @brief Service client for adding multiple objects.
  rclcpp::Client<easy_plan_msgs::srv::AddObjects>::SharedPtr
      add_objects_client_;
  /// @brief Service client for removing an object.
  rclcpp::Client<easy_plan_msgs::srv::RemoveObject>::SharedPtr
      remove_object_client_;
  /// @brief Service client for removing multiple objects.
  rclcpp::Client<easy_plan_msgs::srv::RemoveObjects>::SharedPtr
      remove_objects_client_;

  /// @brief Service client for getting predicates.
  rclcpp::Client<easy_plan_msgs::srv::GetPredicates>::SharedPtr
      get_predicates_client_;
  /// @brief Service client for adding a predicate.
  rclcpp::Client<easy_plan_msgs::srv::AddPredicate>::SharedPtr
      add_predicate_client_;
  /// @brief Service client for adding multiple predicates.
  rclcpp::Client<easy_plan_msgs::srv::AddPredicates>::SharedPtr
      add_predicates_client_;
  /// @brief Service client for removing a predicate.
  rclcpp::Client<easy_plan_msgs::srv::RemovePredicate>::SharedPtr
      remove_predicate_client_;
  /// @brief Service client for removing multiple predicates.
  rclcpp::Client<easy_plan_msgs::srv::RemovePredicates>::SharedPtr
      remove_predicates_client_;

  /// @brief Service client for getting facts.
  rclcpp::Client<easy_plan_msgs::srv::GetFacts>::SharedPtr get_facts_client_;
  /// @brief Service client for adding a fact.
  rclcpp::Client<easy_plan_msgs::srv::AddFact>::SharedPtr add_fact_client_;
  /// @brief Service client for adding multiple facts.
  rclcpp::Client<easy_plan_msgs::srv::AddFacts>::SharedPtr add_facts_client_;
  /// @brief Service client for removing a fact.
  rclcpp::Client<easy_plan_msgs::srv::RemoveFact>::SharedPtr
      remove_fact_client_;
  /// @brief Service client for removing multiple facts.
  rclcpp::Client<easy_plan_msgs::srv::RemoveFacts>::SharedPtr
      remove_facts_client_;

  /// @brief Service client for getting goals.
  rclcpp::Client<easy_plan_msgs::srv::GetGoals>::SharedPtr get_goals_client_;
  /// @brief Service client for adding a goal.
  rclcpp::Client<easy_plan_msgs::srv::AddGoal>::SharedPtr add_goal_client_;
  /// @brief Service client for adding multiple goals.
  rclcpp::Client<easy_plan_msgs::srv::AddGoals>::SharedPtr add_goals_client_;
  /// @brief Service client for removing a goal.
  rclcpp::Client<easy_plan_msgs::srv::RemoveGoal>::SharedPtr
      remove_goal_client_;
  /// @brief Service client for removing multiple goals.
  rclcpp::Client<easy_plan_msgs::srv::RemoveGoals>::SharedPtr
      remove_goals_client_;

  /// @brief Service client for clearing the knowledge base.
  rclcpp::Client<easy_plan_msgs::srv::ClearKnowledgeBase>::SharedPtr
      clear_client_;

  /// @brief Subscription for knowledge update notifications.
  rclcpp::Subscription<easy_plan_msgs::msg::KnowledgeUpdate>::SharedPtr
      knowledge_update_sub_;

  /// @brief List of registered knowledge update callbacks.
  std::vector<KnowledgeUpdateCallback> callbacks_;
};

} // namespace easy_plan_knowledge_base

#endif // EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_CLIENT_HPP_
