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

YasminAction::~YasminAction() { this->viewer_pub_.reset(); }

omni_plan::pddl::ActionStatus
YasminAction::run(const std::vector<std::string> &params) {

  if (this->enable_viewer_pub_ && this->viewer_pub_ == nullptr) {
    // YasminViewerPub stores a shared_ptr to the state machine it visualizes.
    // Passing the action's own shared_from_this() would create a reference
    // cycle (action -> viewer -> action) that leaks the action and every
    // clone. Use an aliasing shared_ptr that points at this instance without
    // taking ownership: the action owns the viewer, so the viewer cannot
    // outlive the state machine.
    yasmin::StateMachine::SharedPtr sm_ptr(std::shared_ptr<void>(), this);
    this->viewer_pub_ =
        std::make_unique<yasmin_viewer::YasminViewerPub>(sm_ptr);
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
