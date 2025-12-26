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

#ifndef EASY_PLAN__ACTION_HPP_
#define EASY_PLAN__ACTION_HPP_

#include <memory>
#include <string>
#include <vector>

#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/timing_predicate.hpp"
#include "easy_plan/utils/parameter_loader.hpp"
#include "easy_plan_msgs/msg/action.hpp"

namespace easy_plan {
namespace pddl {

/**
 * @enum ActionStatus
 * @brief Represents the possible statuses of an action after execution.
 */
enum ActionStatus { SUCCEED, CANCEL, ABORT };

/**
 * @class Action
 * @brief Represents an action in a PDDL domain.
 * @details This class encapsulates the properties and behaviors of a PDDL
 * action, including its name, parameters, preconditions (conditions), and
 * effects. Actions are the core components that define how the state of the
 * world can change in a planning domain. This class supports both durative and
 * non-durative actions through timing predicates for conditions and effects.
 */
class Action : public utils::ParameterLoader {
public:
  /**
   * @brief Constructs an Action with a given name and optional parameters.
   * @param name The name of the action.
   * @param params The parameters of the action (default is an empty vector).
   */
  Action(const std::string &name,
         const std::vector<std::pair<std::string, std::string>> &params = {});

  /**
   * @brief Virtual destructor for the Action class.
   */
  virtual ~Action() = default;

  /**
   * @brief Gets the name of the action.
   * @return The name of the action as a string.
   */
  std::string get_name() const;

  /**
   * @brief Gets the parameters of the action.
   * @return A vector of parameters associated with the action.
   */
  std::vector<Parameter> get_parameters() const;

  /**
   * @brief Gets the type of a parameter by its name.
   * @param param_name The name of the parameter.
   * @return The type of the parameter as a string.
   */
  std::string get_parameter_type(const std::string &param_name) const;

  /**
   * @brief Gets the index of a parameter by its name.
   * @param param_name The name of the parameter.
   * @return The index of the parameter as an integer.
   */
  int get_parameter_index(const std::string &param_name) const;

  /**
   * @brief Adds a condition to the action.
   * @details Conditions define the preconditions that must be met before the
   * action can be executed. They can be timed (at start, over all, at end) and
   * can be negated.
   * @param type The timing type of the condition (START, OVER_ALL, END).
   * @param name The name of the condition predicate.
   * @param args The arguments of the condition predicate.
   * @param negated Whether the condition is negated (default is false).
   */
  void add_condition(Type type, std::string name,
                     const std::vector<std::string> &args = {},
                     bool negated = false);

  /**
   * @brief Adds an effect to the action.
   * @details Effects describe how the action changes the state of the world.
   * They can be timed and can represent adding or deleting predicates.
   * @param type The timing type of the effect (START, OVER_ALL, END).
   * @param name The name of the effect predicate.
   * @param args The arguments of the effect predicate.
   * @param negated Whether the effect is negated (default is false).
   */
  void add_effect(Type type, std::string name,
                  const std::vector<std::string> &args = {},
                  bool negated = false);

  /**
   * @brief Gets all conditions of the action.
   * @return A vector of all conditions associated with the action.
   */
  std::vector<Condition> get_conditions() const;

  /**
   * @brief Gets conditions that must hold at the start of the action.
   * @return A vector of conditions that apply at the start timing.
   */
  std::vector<Condition> get_on_start_conditions() const;

  /**
   * @brief Gets conditions that must hold at the end of the action.
   * @return A vector of conditions that apply at the end timing.
   */
  std::vector<Condition> get_on_end_conditions() const;

  /**
   * @brief Gets conditions that must hold throughout the action execution.
   * @return A vector of conditions that apply over all timing.
   */
  std::vector<Condition> get_over_all_conditions() const;

  /**
   * @brief Gets all effects of the action.
   * @return A vector of all effects associated with the action.
   */
  std::vector<Effect> get_effects() const;

  /**
   * @brief Gets effects that occur at the start of the action.
   * @return A vector of effects that apply at the start timing.
   */
  std::vector<Effect> get_on_start_effects() const;

  /**
   * @brief Gets effects that occur at the end of the action.
   * @return A vector of effects that apply at the end timing.
   */
  std::vector<Effect> get_on_end_effects() const;

  /**
   * @brief Gets effects that occur throughout the action execution.
   * @return A vector of effects that apply over all timing.
   */
  std::vector<Effect> get_over_all_effects() const;

  /**
   * @brief Converts the action to its PDDL representation.
   * @details Generates a string that represents the action in valid PDDL
   * syntax, including parameters, conditions, and effects.
   * @return A string representing the action in PDDL format.
   */
  std::string to_pddl() const;

  /**
   * @brief Converts the action to its ROS message representation.
   * @return A ROS message of type easy_plan_msgs::msg::Action representing the
   * action.
   */
  easy_plan_msgs::msg::Action to_msg() const;

  /**
   * @brief Executes the action with the given parameters.
   * @details This is a pure virtual method that must be implemented by derived
   * classes to define the actual execution logic of the action.
   * @param params The parameters required for action execution.
   * @return The status of the action after execution (SUCCEED, CANCEL, or
   * ABORT).
   */
  virtual ActionStatus run(const std::vector<std::string> &params) = 0;

  /**
   * @brief Cancels the action execution.
   * @details This is a pure virtual method that must be implemented by derived
   * classes to handle cancellation of the action during execution.
   */
  virtual void cancel() = 0;

private:
  /**
   * @brief Builds a PDDL section for timing predicates.
   * @details Helper method to generate PDDL strings for conditions or effects
   * grouped by their timing (at start, over all, at end).
   * @param section The name of the section (e.g., "conditions", "effects").
   * @param items The timing predicates to include in the section.
   * @return A string representing the PDDL section.
   */
  std::string
  build_timing_section(const std::string &section,
                       const std::vector<TimingPredicate> &items) const;

private:
  /// The name of the action.
  std::string name_;
  /// The parameters of the action.
  std::vector<Parameter> parameters_;
  /// The conditions of the action.
  std::vector<Condition> conditions_;
  /// The effects of the action.
  std::vector<Effect> effects_;
};

} // namespace pddl
} // namespace easy_plan
#endif // EASY_PLAN__ACTION_HPP_