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

#include "yasmin/blackboard.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#include "easy_plan_yasmin/yasmin_action.hpp"

using namespace easy_plan_yasmin;

YasminAction::YasminAction(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params)
    : easy_plan::pddl::Action(name, params),
      yasmin::StateMachine(
          std::set<std::string>{yasmin_ros::basic_outcomes::SUCCEED,
                                yasmin_ros::basic_outcomes::CANCEL,
                                yasmin_ros::basic_outcomes::FAIL}) {

  // Enable Yasmin Viewer publisher
  this->viewer_pub_ = std::make_unique<yasmin_viewer::YasminViewerPub>(
      std::shared_ptr<yasmin::StateMachine>(this));
}

easy_plan::pddl::ActionStatus
YasminAction::run(const std::vector<std::string> &params) {

  yasmin::Blackboard::SharedPtr bb = std::make_shared<yasmin::Blackboard>();

  // Populate blackboard with parameters
  for (size_t i = 0; i < params.size() && i < this->get_parameters().size();
       ++i) {
    const auto &param_name = this->get_parameters()[i].get_name();
    bb->set<std::string>(param_name, params[i]);
  }

  std::string outcome = this->execute(bb);

  if (outcome == yasmin_ros::basic_outcomes::SUCCEED) {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  } else if (outcome == yasmin_ros::basic_outcomes::CANCEL) {
    return easy_plan::pddl::ActionStatus::CANCEL;
  } else {
    return easy_plan::pddl::ActionStatus::ABORT;
  }
}

void YasminAction::cancel() { this->cancel_state(); }
