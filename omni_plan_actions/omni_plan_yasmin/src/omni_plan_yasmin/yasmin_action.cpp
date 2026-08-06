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

#include "omni_plan_yasmin/yasmin_action.hpp"

using namespace omni_plan_yasmin;

YasminAction::YasminAction(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params)
    : omni_plan::pddl::Action(name, params),
      yasmin::StateMachine(std::set<std::string>{
          yasmin_ros::basic_outcomes::SUCCEED,
          yasmin_ros::basic_outcomes::CANCEL,
          yasmin_ros::basic_outcomes::ABORT,
      }),
      enable_viewer_pub_(true), viewer_pub_(nullptr) {

  this->set_outcome_description(yasmin_ros::basic_outcomes::SUCCEED,
                                "Action succeeded");
  this->set_outcome_description(yasmin_ros::basic_outcomes::CANCEL,
                                "Action cancelled");
  this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                "Action aborted");

  // Add parameters
  this->add_ros_parameters({
      {"enable_viewer_pub", true, this->enable_viewer_pub_},
  });

  // Create state machine name in uppercase
  std::string sm_name;
  for (const auto &c : name) {
    sm_name += std::toupper(c);
  }
  sm_name += "_ACTION_SM";
  this->set_name(sm_name);
}

omni_plan::pddl::ActionStatus
YasminAction::run(const std::vector<std::string> &params) {

  if (this->enable_viewer_pub_ && this->viewer_pub_ == nullptr) {
    // Enable Yasmin Viewer publisher
    auto non_owning = std::shared_ptr<yasmin::StateMachine>(
        static_cast<yasmin::StateMachine *>(this), [](void *) {});
    this->viewer_pub_ =
        std::make_unique<yasmin_viewer::YasminViewerPub>(non_owning);
  }

  yasmin::Blackboard::SharedPtr bb = std::make_shared<yasmin::Blackboard>();

  // Populate blackboard with parameters
  for (size_t i = 0; i < params.size() &&
                     i < omni_plan::pddl::Action::get_parameters().size();
       ++i) {
    const auto &param_name =
        omni_plan::pddl::Action::get_parameters()[i].get_name();
    bb->set<std::string>(param_name, params[i]);
  }

  std::string outcome = (*this)(bb);

  if (outcome == yasmin_ros::basic_outcomes::SUCCEED) {
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  } else if (outcome == yasmin_ros::basic_outcomes::CANCEL) {
    return omni_plan::pddl::ActionStatus::CANCELED;
  } else if (outcome == yasmin_ros::basic_outcomes::ABORT) {
    return omni_plan::pddl::ActionStatus::ABORTED;
  } else {
    return omni_plan::pddl::ActionStatus::ABORTED;
  }
}

void YasminAction::cancel() { this->cancel_state(); }
