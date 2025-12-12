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

#include <pluginlib/class_loader.hpp>
#include <yasmin/state.hpp>

#include "easy_plan/planner.hpp"
#include "easy_plan/states/outcomes.hpp"

class PlanState : public yasmin::State {

public:
  PlanState()
      : yasmin::State({
            easy_plan::states::outcomes::SUCCEED,
            easy_plan::states::outcomes::FAILED,
        }),
        state_loader_(
            std::make_unique<pluginlib::ClassLoader<easy_plan::Planner>>(
                "easy_plan", "Planner")) {}

  std::string execute(std::shared_ptr<yasmin::Blackboard> blackboard) {
    if (!this->planner_) {
      std::string planner_plugin =
          blackboard->get<std::string>("planner_plugin");
      this->planner_ =
          this->state_loader_->createUniqueInstance(planner_plugin);
    }

    blackboard->set<std::string>("plan", this->planner_->get_plan());
    return easy_plan::states::outcomes::SUCCEED;
  }

private:
  std::unique_ptr<pluginlib::ClassLoader<easy_plan::Planner>> state_loader_;
  pluginlib::UniquePtr<easy_plan::Planner> planner_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PlanState, yasmin::State)