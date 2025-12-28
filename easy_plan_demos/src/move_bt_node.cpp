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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "behaviortree_cpp_v3/action_node.h"
#include "behaviortree_cpp_v3/bt_factory.h"

/**
 * @brief MoveBTNode class that implements a BehaviorTree action node
 * for moving a robot between rooms.
 */
class MoveBTNode : public BT::SyncActionNode {
public:
  MoveBTNode(const std::string &name, const BT::NodeConfiguration &config)
      : BT::SyncActionNode(name, config), progress_(0.0f), increment_(0.05f) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::string>("robot"),
            BT::InputPort<std::string>("r1"), BT::InputPort<std::string>("r2")};
  }

  BT::NodeStatus tick() override {
    // Get parameters from blackboard
    std::string robot, r1, r2;
    if (!getInput<std::string>("robot", robot)) {
      std::cerr << "MoveBTNode: missing required input [robot]" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput<std::string>("r1", r1)) {
      std::cerr << "MoveBTNode: missing required input [r1]" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
    if (!getInput<std::string>("r2", r2)) {
      std::cerr << "MoveBTNode: missing required input [r2]" << std::endl;
      return BT::NodeStatus::FAILURE;
    }

    std::cout << "Moving " << robot << " from " << r1 << " to " << r2
              << " using BtAction" << std::endl;

    // Simulate progressive movement
    while (this->progress_ < 1.0) {
      this->progress_ += this->increment_;
      std::cout << "Moving robot ... ["
                << std::min(100.0f, this->progress_ * 100.0f) << "%]"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    this->progress_ = 0.0f;

    return BT::NodeStatus::SUCCESS;
  }

private:
  float progress_;
  float increment_;
};

// Register the node with BehaviorTree.CPP
BT_REGISTER_NODES(factory) {
  factory.registerNodeType<MoveBTNode>("MoveBTNode");
}
