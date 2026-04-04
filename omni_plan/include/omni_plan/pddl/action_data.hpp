// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef OMNI_PLAN__PDDL__ACTION_DATA_HPP_
#define OMNI_PLAN__PDDL__ACTION_DATA_HPP_

#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"

namespace omni_plan {
namespace pddl {

/**
 * @class ActionData
 * @brief A lightweight, data-only Action that holds PDDL metadata without
 *        runtime resources (e.g. ROS action clients, behavior trees).
 * @details Used to store action definitions in the blackboard for PDDL
 *          generation and planning. Plugin instances are created only
 *          during plan execution.
 */
class ActionData : public Action {
public:
  /**
   * @brief Constructs an ActionData by copying metadata from an existing
   * Action.
   * @param source The action instance to copy metadata from.
   */
  explicit ActionData(const Action &source)
      : Action(source.get_name(), source.get_duration(),
               extract_params(source)) {
    for (const auto &cond : source.get_conditions()) {
      this->add_condition(cond.get_type(), cond.get_name(), cond.get_args(),
                          cond.is_negated());
    }
    for (const auto &eff : source.get_effects()) {
      this->add_effect(eff.get_type(), eff.get_name(), eff.get_args(),
                       eff.is_negated());
    }
  }

  /**
   * @brief No-op execution — ActionData is not meant to be run.
   * @return ActionStatus::ABORT always, as this data-only type cannot execute.
   */
  ActionStatus run(const std::vector<std::string> &) override {
    return ActionStatus::ABORT;
  }

  /**
   * @brief No-op cancellation — ActionData has nothing to cancel.
   */
  void cancel() override {}

private:
  /**
   * @brief Extracts (name, type) parameter pairs from an Action.
   * @param source The action to extract parameters from.
   * @return A vector of (parameter_name, parameter_type) pairs.
   */
  static std::vector<std::pair<std::string, std::string>>
  extract_params(const Action &source) {
    std::vector<std::pair<std::string, std::string>> params;
    for (const auto &p : source.get_parameters()) {
      params.emplace_back(p.get_name(), p.get_type());
    }
    return params;
  }
};

} // namespace pddl
} // namespace omni_plan

#endif // OMNI_PLAN__PDDL__ACTION_DATA_HPP_
