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
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#include "easy_plan_yasmin/yasmin_factory_action.hpp"

using namespace easy_plan_yasmin;

YasminFactoryAction::YasminFactoryAction(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params)
    : easy_plan::pddl::Action(name, params), state_machine_(nullptr) {

  // Create Yasmin Factory
  this->factory_ = std::make_unique<yasmin_factory::YasminFactory>();

  // Add state_machine_xml to parameters
  this->add_parameters({{"state_machine_xml", std::string(""), this->state_machine_xml_}});

  // Enable Yasmin Viewer publisher
  this->viewer_pub_ =
      std::make_unique<yasmin_viewer::YasminViewerPub>(this->state_machine_);
}

easy_plan::pddl::ActionStatus
YasminFactoryAction::run(const std::vector<std::string> &params) {

  if (this->state_machine_ == nullptr) {

    if (this->state_machine_xml_.empty()) {
      return easy_plan::pddl::ActionStatus::ABORT;
    }

    this->state_machine_ =
        this->factory_->create_sm_from_file(this->state_machine_xml_);
  }

  yasmin::Blackboard::SharedPtr bb = std::make_shared<yasmin::Blackboard>();

  // Populate blackboard with parameters
  for (size_t i = 0; i < params.size() && i < this->get_parameters().size();
       ++i) {
    const auto &param_name = this->get_parameters()[i].get_name();
    bb->set<std::string>(param_name, params[i]);
  }

  std::string outcome = (*this->state_machine_)(bb);

  if (outcome == yasmin_ros::basic_outcomes::SUCCEED) {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  } else if (outcome == yasmin_ros::basic_outcomes::CANCEL) {
    return easy_plan::pddl::ActionStatus::CANCEL;
  } else {
    return easy_plan::pddl::ActionStatus::ABORT;
  }
}

void YasminFactoryAction::cancel() { this->state_machine_->cancel_state(); }
