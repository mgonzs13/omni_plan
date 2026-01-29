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

#ifndef OMNI_PLAN__PDDL__TIMING_PREDICATE_HPP_
#define OMNI_PLAN__PDDL__TIMING_PREDICATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan_msgs/msg/predicate.hpp"

namespace omni_plan {
namespace pddl {

/**
 * @enum Type
 * @brief Represents the timing type of the predicate.
 * @details Timing types specify when a condition or effect occurs relative
 * to the action's duration: at the start, throughout execution, or at the end.
 */
enum Type { START, OVER_ALL, END };

/**
 * @class TimingPredicate
 * @brief Represents a timing predicate in a PDDL domain.
 * @details Extends Predicate to include timing information for durative
 * actions. Timing predicates specify when conditions must hold or effects occur
 * during action execution: at the start, over the entire duration, or at the
 * end.
 */
class TimingPredicate : public Predicate {

public:
  /**
   * @brief Constructs a TimingPredicate with a given type, name, arguments, and
   * negation status.
   * @param type The timing type (START, OVER_ALL, END).
   * @param name The name of the predicate.
   * @param args The arguments of the predicate.
   * @param negated Whether the predicate is negated.
   */
  TimingPredicate(Type type, const std::string &name,
                  const std::vector<std::string> &args = {},
                  bool negated = false);

  /**
   * @brief Gets the timing type of the predicate.
   * @return The timing type of the predicate.
   */
  Type get_type() const;

  /**
   * @brief Converts the timing predicate to its PDDL representation.
   * @details Generates PDDL syntax including timing keywords like "at start",
   * "over all", or "at end" depending on the predicate's type.
   * @param as_fact If true, formats the predicate as a fact.
   * @return A string representing the timing predicate in PDDL format.
   */
  std::string to_pddl(bool as_fact = false) const;

  /**
   * @brief Converts the predicate to a ROS message.
   * @return An omni_plan_msgs::msg::Predicate message representing the
   * predicate.
   */
  omni_plan_msgs::msg::Predicate to_msg() const override;

private:
  /// @brief The timing type of the predicate.
  Type type_;
};

/// Type alias for conditions in actions.
using Condition = TimingPredicate;

/// Type alias for effects in actions.
using Effect = TimingPredicate;

} // namespace pddl
} // namespace omni_plan

#endif // OMNI_PLAN__PDDL__TIMING_PREDICATE_HPP_
