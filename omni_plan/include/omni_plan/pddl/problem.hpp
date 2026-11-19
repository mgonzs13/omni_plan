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

#ifndef OMNI_PLAN__PDDL__PROBLEM_HPP_
#define OMNI_PLAN__PDDL__PROBLEM_HPP_

#include <memory>
#include <set>
#include <string>

#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan_msgs/msg/problem.hpp"

namespace omni_plan {
namespace pddl {

/**
 * @class Problem
 * @brief Represents a PDDL problem with its objects, goals, and facts.
 * @details A PDDL problem defines a specific instance of a planning domain,
 * including the objects that exist, the initial state (facts), and the goals
 * that need to be achieved. This class provides methods to build and manipulate
 * problem components and generate PDDL output.
 */
class Problem {
public:
  /**
   * @brief Constructs an empty Problem.
   */
  Problem() = default;

  /**
   * @brief Adds an object to the problem.
   * @details Objects are the entities that exist in this specific problem
   * instance.
   * @param obj The object to add.
   */
  void add_object(const Object &obj);

  /**
   * @brief Gets the objects in the problem.
   * @return A const reference to the set of objects.
   */
  const std::set<Object> &get_objects() const { return this->objects_; }

  /**
   * @brief Adds a goal to the problem.
   * @details Goals define the desired final state that the planner must
   * achieve.
   * @param goal The goal predicate to add.
   */
  void add_goal(const Predicate &goal);

  /**
   * @brief Gets the goals of the problem.
   * @return A const reference to the set of goal predicates.
   */
  const std::set<Predicate> &get_goals() const { return this->goals_; }

  /**
   * @brief Adds a fact to the problem.
   * @details Facts describe the initial state of the world at the
   * beginning of planning.
   * @param fact The fact predicate to add.
   */
  void add_fact(const Predicate &fact);

  /**
   * @brief Gets the initial facts of the problem.
   * @return A const reference to the set of fact predicates.
   */
  const std::set<Predicate> &get_facts() const { return this->facts_; }

  /**
   * @brief Converts the problem to its PDDL representation.
   * @details Generates a complete PDDL problem file content as a string,
   * including all objects, initial facts, and goals.
   * @return A string representing the problem in PDDL format.
   */
  std::string to_pddl() const;

  /**
   * @brief Converts the problem to a ROS message.
   * @return An omni_plan_msgs::msg::Problem message representing the problem.
   */
  omni_plan_msgs::msg::Problem to_msg() const;

private:
  /// @brief The objects in the problem.
  std::set<Object> objects_;
  /// @brief The goals of the problem.
  std::set<Predicate> goals_;
  /// @brief The facts of the problem.
  std::set<Predicate> facts_;
};

} // namespace pddl
} // namespace omni_plan

#endif // OMNI_PLAN__PDDL__PROBLEM_HPP_
