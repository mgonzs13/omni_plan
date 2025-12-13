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
            easy_plan::states::outcomes::CANCELED,
        }) {}

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {

    auto pddl_manager =
        blackboard->get<std::shared_ptr<easy_plan::PddlManager>>(
            "pddl_manager");
    easy_plan::Plan plan = blackboard->get<easy_plan::Plan>("plan");
    std::vector<std::string> params;

    while (plan.get_next_action(this->current_action_, params)) {
      YASMIN_LOG_INFO("Executing action: %s",
                      this->current_action_->get_name().c_str());

      // Apply action effects before running the action
      pddl_manager->apply_effects(
          this->current_action_->get_on_start_effects());
      pddl_manager->apply_effects(
          this->current_action_->get_over_all_effects());

      // Run the action
      this->current_action_->run(params);

      // Apply action effects after running the action
      pddl_manager->undo_effects(this->current_action_->get_over_all_effects());
      pddl_manager->apply_effects(this->current_action_->get_on_end_effects());

      if (this->is_canceled()) {
        YASMIN_LOG_INFO("Plan execution canceled");
        return easy_plan::states::outcomes::CANCELED;
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