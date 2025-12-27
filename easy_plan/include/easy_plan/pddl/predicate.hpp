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

#ifndef EASY_PLAN__PDDL__PREDICATE_HPP_
#define EASY_PLAN__PDDL__PREDICATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "easy_plan_msgs/msg/predicate.hpp"

namespace easy_plan {
namespace pddl {

/**
 * @class Predicate
 * @brief Represents a predicate in a PDDL domain.
 * @details Predicates are atomic formulas that describe properties of objects
 * or relationships between them. They form the basis of state representation
 * in PDDL, appearing in initial states, goals, preconditions, and effects.
 */
class Predicate {
public:
  /**
   * @brief Constructs a Predicate with a given name, arguments, and negation
   * status.
   * @param name The name of the predicate.
   * @param args The arguments of the predicate (default is an empty vector).
   * @param negated Whether the predicate is negated (default is false).
   */
  Predicate(const std::string &name, const std::vector<std::string> &args = {},
            bool negated = false);

  /**
   * @brief Gets the name of the predicate.
   * @return The name of the predicate as a string.
   */
  std::string get_name() const;

  /**
   * @brief Gets the arguments of the predicate.
   * @return A vector of strings representing the arguments.
   */
  std::vector<std::string> get_args() const;

  /**
   * @brief Checks if the predicate is negated.
   * @return True if the predicate is negated, false otherwise.
   */
  bool is_negated() const;

  /**
   * @brief Sets the negation status of the predicate.
   * @param negated The new negation status.
   */
  void set_negation(bool negated);

  /**
   * @brief Converts the predicate to its PDDL representation.
   * @details Generates a string in PDDL syntax. If as_fact is true, formats
   * it as an initial fact; otherwise, as a general predicate expression.
   * @param as_fact If true, formats the predicate as a fact (default is false).
   * @return A string representing the predicate in PDDL format.
   */
  std::string to_pddl(bool as_fact = false) const;

  /**
   * @brief Converts the predicate to a ROS message.
   * @return An easy_plan_msgs::msg::Predicate message representing the
   * predicate.
   */
  virtual easy_plan_msgs::msg::Predicate to_msg() const;

  /**
   * @brief Comparison operator for ordering predicates.
   * @details Predicates are ordered by name, then by arguments, then by
   * negation status.
   * @param other The other predicate to compare with.
   * @return True if this predicate is less than the other, false otherwise.
   */
  bool operator<(const Predicate &other) const {
    if (name_ != other.name_) {
      return name_ < other.name_;
    }
    if (args_ != other.args_) {
      return args_ < other.args_;
    }
    return negated_ < other.negated_;
  }

private:
  /// The name of the predicate.
  std::string name_;
  /// The arguments of the predicate.
  std::vector<std::string> args_;
  /// Whether the predicate is negated.
  bool negated_;
};

} // namespace pddl
} // namespace easy_plan

#endif // EASY_PLAN__PDDL__PREDICATE_HPP_
