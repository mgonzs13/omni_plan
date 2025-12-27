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

#ifndef EASY_PLAN_KNOWLEDGE_BASE__KB_PDDL_MANAGER_HPP_
#define EASY_PLAN_KNOWLEDGE_BASE__KB_PDDL_MANAGER_HPP_

#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/pddl_manager.hpp"

#include "rclcpp/rclcpp.hpp"

#include "easy_plan_msgs/msg/knowledge_update.hpp"
#include "easy_plan_msgs/srv/add_fact.hpp"
#include "easy_plan_msgs/srv/get_facts.hpp"
#include "easy_plan_msgs/srv/get_goals.hpp"
#include "easy_plan_msgs/srv/get_objects.hpp"
#include "easy_plan_msgs/srv/get_predicates.hpp"
#include "easy_plan_msgs/srv/get_types.hpp"
#include "easy_plan_msgs/srv/remove_fact.hpp"

namespace easy_plan_knowledge_base {

/**
 * @class KbPddlManager
 * @brief PDDL manager that uses ROS 2 services to communicate with the
 * knowledge base.
 * @details This class implements the PddlManager interface by retrieving
 * domain and problem information from a knowledge base through ROS 2 service
 * calls.
 */
class KbPddlManager : public easy_plan::PddlManager {
public:
  /**
   * @brief Constructor that optionally subscribes to knowledge updates.
   * @param add_callback Whether to subscribe to knowledge update messages.
   */
  explicit KbPddlManager();

  /**
   * @brief Destructor that stops the executor thread.
   */
  ~KbPddlManager() override;

  /**
   * @brief Generates PDDL domain and problem from the knowledge base.
   * @return A pair containing the generated Domain and Problem objects.
   */
  std::pair<easy_plan::pddl::Domain, easy_plan::pddl::Problem>
  get_pddl() const override;

  /**
   * @brief Checks if there are any goals in the knowledge base.
   * @return True if goals exist, false otherwise.
   */
  bool has_goals() const override;

  /**
   * @brief Checks if a predicate exists as a fact in the knowledge base.
   * @param predicate The predicate to check.
   * @return True if the predicate exists, false otherwise.
   */
  bool
  predicate_exists(const easy_plan::pddl::Predicate &predicate) const override;

  /**
   * @brief Checks if a predicate is part of the goals.
   * @param predicate The predicate to check.
   * @return True if the predicate is a goal, false otherwise.
   */
  bool
  predicate_is_goal(const easy_plan::pddl::Predicate &predicate) const override;

  /**
   * @brief Applies an effect to the knowledge base.
   * @param exp The effect to apply.
   */
  void apply_effect(const easy_plan::pddl::Effect &exp) override;

private:
  /**
   * @brief Callback for knowledge update messages.
   * @param msg The knowledge update message.
   */
  void knowledge_update_callback(
      const easy_plan_msgs::msg::KnowledgeUpdate::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;

  // Service clients
  rclcpp::Client<easy_plan_msgs::srv::GetTypes>::SharedPtr get_types_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetObjects>::SharedPtr
      get_objects_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetPredicates>::SharedPtr
      get_predicates_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetFacts>::SharedPtr get_facts_client_;
  rclcpp::Client<easy_plan_msgs::srv::GetGoals>::SharedPtr get_goals_client_;
  rclcpp::Client<easy_plan_msgs::srv::AddFact>::SharedPtr add_fact_client_;
  rclcpp::Client<easy_plan_msgs::srv::RemoveFact>::SharedPtr
      remove_fact_client_;

  // Subscription
  rclcpp::Subscription<easy_plan_msgs::msg::KnowledgeUpdate>::SharedPtr
      knowledge_update_sub_;

  // Executor and thread
  rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
  std::thread executor_thread_;

  // Synchronization for goals
  mutable std::mutex goal_mutex_;
  mutable std::condition_variable goal_cv_;
};

} // namespace easy_plan_knowledge_base

#endif // EASY_PLAN_KNOWLEDGE_BASE__KB_PDDL_MANAGER_HPP_
