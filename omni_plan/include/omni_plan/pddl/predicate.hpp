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

#ifndef OMNI_PLAN__PDDL__PREDICATE_HPP_
#define OMNI_PLAN__PDDL__PREDICATE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "omni_plan_msgs/msg/predicate.hpp"

namespace omni_plan {
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
   * @return An omni_plan_msgs::msg::Predicate message representing the
   * predicate.
   */
  virtual omni_plan_msgs::msg::Predicate to_msg() const;

  /**
   * @brief Comparison operator for ordering predicates.
   * @details Predicates are ordered by name, then by arguments, then by
   * negation status.
   * @param other The other predicate to compare with.
   * @return True if this predicate is less than the other, false otherwise.
   */
  bool operator<(const Predicate &other) const {
    if (this->name_ != other.name_) {
      return this->name_ < other.name_;
    }
    if (this->args_ != other.args_) {
      return this->args_ < other.args_;
    }
    return this->negated_ < other.negated_;
  }

  /**
   * @brief Equality comparison operator for predicates.
   * @details Two predicates are considered equal if they have the same name,
   * arguments, and negation status.
   * @param other The other predicate to compare with.
   * @return True if the predicates are equal, false otherwise.
   */
  bool operator==(const Predicate &other) const {
    return this->name_ == other.name_ && this->args_ == other.args_ &&
           this->negated_ == other.negated_;
  }

private:
  /// @brief The name of the predicate.
  std::string name_;
  /// @brief The arguments of the predicate.
  std::vector<std::string> args_;
  /// @brief Whether the predicate is negated.
  bool negated_;
};

} // namespace pddl
} // namespace omni_plan

#endif // OMNI_PLAN__PDDL__PREDICATE_HPP_
