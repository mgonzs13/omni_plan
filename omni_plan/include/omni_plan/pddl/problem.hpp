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
   * @brief Adds a goal to the problem.
   * @details Goals define the desired final state that the planner must
   * achieve.
   * @param goal The goal predicate to add.
   */
  void add_goal(const Predicate &goal);

  /**
   * @brief Adds a fact to the problem.
   * @details Facts describe the initial state of the world at the beginning of
   * planning.
   * @param fact The fact predicate to add.
   */
  void add_fact(const Predicate &fact);

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
  /// The objects in the problem.
  std::set<Object> objects_;
  /// The goals of the problem.
  std::set<Predicate> goals_;
  /// The facts of the problem.
  std::set<Predicate> facts_;
};

} // namespace pddl
} // namespace omni_plan

#endif // OMNI_PLAN__PDDL__PROBLEM_HPP_
