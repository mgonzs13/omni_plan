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
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "easy_plan_bt/bt_action.hpp"

using namespace easy_plan_bt;

BtAction::BtAction(
    const std::string &name,
    const std::vector<std::pair<std::string, std::string>> &params,
    const std::string &default_bt_file_path)
    : easy_plan::pddl::Action(name, params), tree_(nullptr),
      default_bt_file_path_(default_bt_file_path) {

  this->add_ros_parameters({
      {"bt_file_path", this->default_bt_file_path_, this->bt_file_path_},
      {"plugins", std::vector<std::string>(), this->plugins_},
      {"tick_rate", 10, this->tick_rate_},
      {"enable_groot_monitoring", false, this->enable_groot_monitoring_},
      {"max_msg_per_second", 10, this->max_msg_per_second_},
      {"publisher_port", 1666, this->publisher_port_},
      {"server_port", 1667, this->server_port_},
  });

  this->add_load_ros_parameters_callback(std::bind(&BtAction::load_tree, this));
}

easy_plan::pddl::ActionStatus
BtAction::run(const std::vector<std::string> &params) {

  // Check tree is loaded
  if (this->tree_ == nullptr) {
    return easy_plan::pddl::ActionStatus::ABORT;
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
  this->is_canceled_.store(false);

  while (result == BT::NodeStatus::RUNNING) {
    result = this->tree_->rootNode()->executeTick();
    loop_rate.sleep();
  }

  if (result == BT::NodeStatus::SUCCESS) {
    return easy_plan::pddl::ActionStatus::SUCCEED;
  } else if (result == BT::NodeStatus::FAILURE) {
    return easy_plan::pddl::ActionStatus::ABORT;
  } else if (this->is_canceled_.load()) {
    return easy_plan::pddl::ActionStatus::CANCEL;
  }

  return easy_plan::pddl::ActionStatus::ABORT;
}

void BtAction::cancel() {
  this->tree_->haltTree();
  this->is_canceled_.store(true);
}

void BtAction::load_tree() {
  // Load plugins
  for (const auto &p : this->plugins_) {
    this->bt_factory_.registerFromPlugin(BT::SharedLibrary::getOSName(p));
  }

  // Load tree
  this->blackboard_ = BT::Blackboard::create();

  std::string bt_xml_path = this->bt_file_path_;

  // Check if bt_file_path file exists
  if (bt_xml_path.empty() || !std::filesystem::exists(bt_xml_path) ||
      std::filesystem::is_directory(bt_xml_path)) {
    throw std::runtime_error("BT XML file does not exist: " + bt_xml_path);
  }

  // Check if bt_file_path is an absolute path, if not, make it absolute
  if (bt_xml_path.empty() ||
      !std::filesystem::path(bt_xml_path).is_absolute()) {
    bt_xml_path = (std::filesystem::current_path() / bt_xml_path).string();
  }

  // Create tree from file
  this->tree_ = std::make_shared<BT::Tree>(
      this->bt_factory_.createTreeFromFile(bt_xml_path, this->blackboard_));

  // Enable Groot monitoring if required
  if (this->enable_groot_monitoring_) {
    this->groot_monitor_ = std::make_unique<BT::PublisherZMQ>(
        *this->tree_, this->max_msg_per_second_, this->publisher_port_,
        this->server_port_);
  }
}