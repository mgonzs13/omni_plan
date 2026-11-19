// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef OMNI_PLAN__PDDL__PLAN_HPP_
#define OMNI_PLAN__PDDL__PLAN_HPP_

#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan_msgs/msg/plan.hpp"

namespace omni_plan {
namespace pddl {

struct PlanningGraph;

/**
 * @class Plan
 * @brief Represents a sequence of actions that solves a planning problem.
 * @details This class stores a sequence of actions with their parameters that,
 * when executed in order, achieve the goals of a planning problem. It also
 * tracks whether a valid solution was found.
 */
class Plan {
public:
  /**
   * @brief Constructs a Plan with an optional solution status.
   */
  Plan();

  /**
   * @brief Sets whether this plan represents a valid solution.
   * @param has_solution True if the plan has a solution, false otherwise.
   */
  void set_has_solution(bool has_solution);

  /**
   * @brief Checks if this plan represents a valid solution.
   * @return True if the plan has a solution, false otherwise.
   */
  bool has_solution() const;

  /**
   * @brief Sets the original planner raw output string.
   * @param raw_output The raw output from the planner as a string.
   */
  void set_raw_output(const std::string &raw_output);

  /**
   * @brief Gets the original planner raw output string.
   * @return The raw output from the planner as a string.
   */
  std::string get_raw_output() const;

  /**
   * @brief Adds an action with its parameters to the plan.
   * @details Appends an action and its execution parameters to the end of the
   * plan.
   * @param action A shared pointer to the action to add.
   * @param params The parameters for executing the action (default: empty
   * vector).
   * @param start_time The start time for the action (default: 0.0f).
   */
  void add_action(const std::shared_ptr<pddl::Action> &action,
                  const std::vector<std::string> &params = {},
                  float start_time = 0.0f);

  /**
   * @brief Gets the number of actions in the plan.
   * @return The size of the plan (number of actions).
   */
  size_t size() const;

  /**
   * @brief Gets the action at a specific index in the plan.
   * @param index The index of the action to retrieve.
   * @return A shared pointer to the action at the specified index.
   */
  std::shared_ptr<pddl::Action> get_action(size_t index) const;

  /**
   * @brief Gets the parameters for the action at a specific index.
   * @param index The index of the action whose parameters to retrieve.
   * @return A vector of parameter strings for the action.
   */
  std::vector<std::string> get_action_params(size_t index) const;

  /**
   * @brief Gets both the action and its parameters at a specific index.
   * @param index The index of the action to retrieve.
   * @return A pair containing the action and its parameters.
   */
  std::pair<std::shared_ptr<pddl::Action>, std::vector<std::string>>
  get_action_with_params(size_t index) const;

  /**
   * @brief Gets the start time of the action at a specific index.
   * @param index The index of the action.
   * @return The start time as a float.
   */
  float get_action_start_time(size_t index) const;

  /**
   * @brief Converts the plan to its PDDL representation.
   * @details Generates a string that represents the entire plan in valid PDDL
   * syntax, listing all actions with their parameters in order.
   * @return A string representing the plan in PDDL format.
   */
  std::string to_pddl() const;

  /**
   * @brief Converts the plan to a ROS message.
   * @return An omni_plan_msgs::msg::Plan message representing the plan.
   */
  omni_plan_msgs::msg::Plan to_msg() const;

  /**
   * @brief Attaches a pre-built planning graph to this plan.
   * @details Planners that already build an execution graph (for example, the
   * MRTA planner) can attach it here. Dispatch states use the attached graph
   * instead of rebuilding it from the plan.
   * @param graph The graph describing the plan's action dependencies.
   */
  void set_planning_graph(std::shared_ptr<PlanningGraph> graph);

  /**
   * @brief Gets the planning graph attached to this plan, if any.
   * @return The attached graph, or nullptr when no graph was attached.
   */
  std::shared_ptr<PlanningGraph> get_planning_graph() const;

private:
  /// @brief The original planner raw output string (for reference/debugging).
  std::string raw_output;
  /// @brief Whether this plan represents a valid solution.
  bool has_solution_ = false;
  /// @brief The sequence of actions in the plan.
  std::vector<std::shared_ptr<pddl::Action>> actions_;
  /// @brief The parameters for each action in the plan.
  std::vector<std::vector<std::string>> params_;
  /// @brief The start time for each action in the plan.
  std::vector<float> start_times_;
  /// @brief Optional execution graph built by the planner.
  std::shared_ptr<PlanningGraph> planning_graph_;
};

} // namespace pddl
} // namespace omni_plan
#endif // OMNI_PLAN__PDDL__PLAN_HPP_