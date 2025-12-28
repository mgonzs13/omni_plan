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

#include <filesystem>
#include <memory>

#include "yasmin/blackboard.hpp"
#include "yasmin_factory/yasmin_factory.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_viewer/yasmin_viewer_pub.hpp"

#include "easy_plan_yasmin/yasmin_factory_action.hpp"

using namespace easy_plan_yasmin;

YasminFactoryAction::YasminFactoryAction(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params,
    const std::string &default_state_machine_xml)
    : easy_plan::pddl::Action(name, params), state_machine_(nullptr),
      default_state_machine_xml_(default_state_machine_xml) {

  // Create Yasmin Factory
  this->factory_ = std::make_unique<yasmin_factory::YasminFactory>();

  // Add parameters
  this->add_ros_parameters({
      {"state_machine_xml", this->default_state_machine_xml_,
       this->state_machine_xml_},
      {"enable_viewer_pub", true, this->enable_viewer_pub_},
      {"succeed_outcome", std::string(yasmin_ros::basic_outcomes::SUCCEED),
       this->succeed_outcome_},
      {"cancel_outcome", std::string(yasmin_ros::basic_outcomes::CANCEL),
       this->cancel_outcome_},
      {"abort_outcome", std::string(yasmin_ros::basic_outcomes::ABORT),
       this->abort_outcome_},
  });
}

easy_plan::pddl::ActionStatus
YasminFactoryAction::run(const std::vector<std::string> &params) {

  if (this->state_machine_ == nullptr) {

    std::string state_machine_xml = this->state_machine_xml_;

    // Check if state_machine_xml_ file exists
    if (state_machine_xml.empty() ||
        !std::filesystem::exists(state_machine_xml) ||
        std::filesystem::is_directory(state_machine_xml)) {
      return easy_plan::pddl::ActionStatus::ABORT;
    }

    // Check if bt_file_path_ is an absolute path, if not, make it absolute
    if (state_machine_xml.empty() ||
        !std::filesystem::path(state_machine_xml).is_absolute()) {
      state_machine_xml =
          (std::filesystem::current_path() / state_machine_xml).string();
    }

    // Create state machine from XML
    this->state_machine_ =
        this->factory_->create_sm_from_file(state_machine_xml);

    if (this->enable_viewer_pub_) {
      // Enable Yasmin Viewer publisher
      this->viewer_pub_ = std::make_unique<yasmin_viewer::YasminViewerPub>(
          this->state_machine_);
    }
  }

  // Create blackboard
  yasmin::Blackboard::SharedPtr bb = std::make_shared<yasmin::Blackboard>();

  // Populate blackboard with parameters
  for (size_t i = 0; i < params.size() && i < this->get_parameters().size();
       ++i) {
    const auto &param_name = this->get_parameters()[i].get_name();
    bb->set<std::string>(param_name, params[i]);
  }

  std::string outcome = (*this->state_machine_)(bb);

  if (outcome == this->succeed_outcome_) {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  } else if (outcome == this->cancel_outcome_) {
    return easy_plan::pddl::ActionStatus::CANCEL;
  } else if (outcome == this->abort_outcome_) {
    return easy_plan::pddl::ActionStatus::ABORT;
  } else {
    return easy_plan::pddl::ActionStatus::ABORT;
  }
}

void YasminFactoryAction::cancel() { this->state_machine_->cancel_state(); }
