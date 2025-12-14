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

#include <memory>
#include <string>

#include <yasmin/state.hpp>

#include "easy_plan/pddl/action.hpp"
#include "easy_plan/pddl_manager.hpp"
#include "easy_plan/plan.hpp"
#include "easy_plan/states/outcomes.hpp"

class ExecutePlanState : public yasmin::State {

public:
  ExecutePlanState()
      : yasmin::State({
            easy_plan::states::outcomes::SUCCEED,
            easy_plan::states::outcomes::FAILED,
            easy_plan::states::outcomes::CANCELED,
        }) {}

  std::vector<easy_plan::pddl::Effect>
  instantiate_effects(std::vector<easy_plan::pddl::Effect> effects,
                      const std::vector<std::string> &params) {

    std::vector<easy_plan::pddl::Effect> instantiated_effects;
    for (const auto &eff : effects) {
      auto args = eff.expression->get_args();
      std::vector<std::string> instantiated_args;
      for (size_t j = 0; j < args.size(); j++) {
        instantiated_args.push_back(
            params[this->current_action_->get_parameter_index(args[j])]);
      }
      instantiated_effects.push_back(easy_plan::pddl::Effect{
          eff.type, std::make_shared<easy_plan::pddl::Predicate>(
                        eff.expression->get_name(), instantiated_args,
                        eff.expression->is_negated())});
    }
    return instantiated_effects;
  }

  void apply_effects_with_params(
      std::vector<easy_plan::pddl::Effect> effects,
      const std::vector<std::string> &params,
      std::shared_ptr<easy_plan::PddlManager> pddl_manager) {

    std::vector<easy_plan::pddl::Effect> effects_to_apply =
        this->instantiate_effects(effects, params);

    // Apply action effects before running the action
    pddl_manager->apply_effects(effects_to_apply);
  }

  void undo_effects_with_params(
      std::vector<easy_plan::pddl::Effect> effects,
      const std::vector<std::string> &params,
      std::shared_ptr<easy_plan::PddlManager> pddl_manager) {

    std::vector<easy_plan::pddl::Effect> effects_to_apply =
        this->instantiate_effects(effects, params);

    // Undone action effects before running the action
    pddl_manager->undo_effects(effects_to_apply);
  }

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {

    auto pddl_manager =
        blackboard->get<std::shared_ptr<easy_plan::PddlManager>>(
            "pddl_manager");
    easy_plan::Plan plan = blackboard->get<easy_plan::Plan>("plan");

    for (size_t i = 0; i < plan.size(); ++i) {
      auto [action, params] = plan.get_action_with_params(i);
      this->current_action_ = action;

      YASMIN_LOG_INFO("Executing action: %s",
                      this->current_action_->get_name().c_str());

      // Apply action effects before running the action
      this->apply_effects_with_params(
          this->current_action_->get_on_start_effects(), params, pddl_manager);
      this->apply_effects_with_params(
          this->current_action_->get_over_all_effects(), params, pddl_manager);

      // Run the action
      auto status = this->current_action_->run(params);

      // Apply action effects after running the action
      this->undo_effects_with_params(
          this->current_action_->get_over_all_effects(), params, pddl_manager);
      this->apply_effects_with_params(
          this->current_action_->get_on_end_effects(), params, pddl_manager);

      if (this->is_canceled() ||
          status == easy_plan::pddl::ActionStatus::CANCELED) {
        YASMIN_LOG_INFO("Plan execution canceled");
        return easy_plan::states::outcomes::CANCELED;

      } else if (status == easy_plan::pddl::ActionStatus::FAILED) {
        YASMIN_LOG_ERROR("Action '%s' failed",
                         this->current_action_->get_name().c_str());
        return easy_plan::states::outcomes::FAILED;
      }
    }

    return easy_plan::states::outcomes::SUCCEED;
  }

  void cancel_state() override {
    if (this->current_action_) {
      this->current_action_->cancel();
    }
    yasmin::State::cancel_state();
  }

private:
  std::shared_ptr<easy_plan::pddl::Action> current_action_ = nullptr;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ExecutePlanState, yasmin::State)