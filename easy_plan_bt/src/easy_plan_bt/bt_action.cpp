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
#include <vector>

#include "easy_plan_bt/bt_action.hpp"

using namespace easy_plan_bt;

BtAction::BtAction(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params)
    : easy_plan::pddl::Action(name, params), tree_(nullptr) {

  this->add_parameters({
      {"bt_file_path", std::string("tree.xml"), this->bt_file_path_},
      {"plugins", std::vector<std::string>(), this->plugins_},
      {"tick_rate", 10, this->tick_rate_},
      {"enable_groot_monitoring", false, this->enable_groot_monitoring_},
      {"max_msg_per_second", 10, this->max_msg_per_second_},
      {"publisher_port", 1666, this->publisher_port_},
      {"server_port", 1667, this->server_port_},
  });
}

easy_plan::pddl::ActionStatus
BtAction::run(const std::vector<std::string> &params) {

  if (this->tree_ == nullptr) {
    // Load plugins
    for (const auto &p : this->plugins_) {
      this->bt_factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
    }

    // Load tree
    this->blackboard_ = BT::Blackboard::create();
    this->tree_ =
        std::make_shared<BT::Tree>(this->bt_factory_.createTreeFromFile(
            this->bt_file_path_, this->blackboard_));

    // Enable Groot monitoring if required
    if (this->enable_groot_monitoring_) {
      this->groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
          *this->tree_, this->max_msg_per_second_, this->publisher_port_,
          this->server_port_);
    }
  }

  // Run the tree
  BT::NodeStatus result = BT::NodeStatus::RUNNING;

  // Populate blackboard with parameters
  for (size_t i = 0; i < params.size() && i < this->get_parameters().size();
       ++i) {
    const auto &param_name = this->get_parameters()[i].get_name();
    this->blackboard_->set(param_name, params[i]);
  }

  rclcpp::Rate loop_rate(this->tick_rate_);

  while (result == BT::NodeStatus::RUNNING) {
    result = this->tree_->rootNode()->executeTick();
    loop_rate.sleep();
  }

  if (result == BT::NodeStatus::SUCCESS) {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  } else if (result == BT::NodeStatus::FAILURE) {
    return easy_plan::pddl::ActionStatus::ABORT;
  }

  return easy_plan::pddl::ActionStatus::CANCEL;
}

void BtAction::cancel() { this->tree_->haltTree(); }
