// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include "omni_plan_dispatcher/sequential_plan_dispatcher.hpp"
#include "omni_plan_msgs/msg/plan_action_status.hpp"
#include "omni_plan_msgs/msg/plan_execution_status.hpp"

#include <pluginlib/class_list_macros.hpp>

using namespace omni_plan_dispatcher;
using namespace omni_plan;

SequentialPlanDispatcher::SequentialPlanDispatcher()
    : omni_plan::PlanDispatcher() {}

pddl::ActionStatus SequentialPlanDispatcher::dispatch_actions(
    const std::vector<pddl::GraphNode::Ptr> &all_nodes) {

  // Sort nodes by (level_num, node_num) so execution follows the topological
  // order established by the planning graph builder.
  std::vector<pddl::GraphNode::Ptr> ordered = all_nodes;
  std::sort(ordered.begin(), ordered.end(),
            [](const pddl::GraphNode::Ptr &a, const pddl::GraphNode::Ptr &b) {
              if (a->level_num != b->level_num) {
                return a->level_num < b->level_num;
              }
              return a->node_num < b->node_num;
            });

  for (const auto &node : ordered) {
    const int idx = node->node_num;

    if (this->is_canceled()) {
      this->set_node_status(idx,
                            omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
      this->publish_exec_status(
          omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
      continue;
    }

    auto action = node->action.action;

    if (!action) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "[SequentialPlanDispatcher] Action at node %d has no plugin",
                   idx);
      this->set_node_status(idx, omni_plan_msgs::msg::PlanActionStatus::FAILED);
      this->publish_exec_status(
          omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

      return pddl::ActionStatus::ABORTED;
    }

    this->set_node_status(idx, omni_plan_msgs::msg::PlanActionStatus::RUNNING);
    this->publish_exec_status(
        omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

    this->push_current_action(action);
    pddl::ActionStatus result = this->run_node_action(node, action);
    this->remove_current_action(action);

    if (result == pddl::ActionStatus::SUCCEEDED) {
      this->set_node_status(idx,
                            omni_plan_msgs::msg::PlanActionStatus::SUCCEEDED);
    } else if (result == pddl::ActionStatus::CANCELED) {
      this->set_node_status(idx,
                            omni_plan_msgs::msg::PlanActionStatus::CANCELLED);
    } else {
      this->set_node_status(idx, omni_plan_msgs::msg::PlanActionStatus::FAILED);
    }

    this->publish_exec_status(
        omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

    if (result == pddl::ActionStatus::CANCELED) {
      return pddl::ActionStatus::CANCELED;
    }

    if (result == pddl::ActionStatus::ABORTED) {
      return pddl::ActionStatus::ABORTED;
    }
  }

  this->clear_current_actions();
  return this->is_canceled() ? pddl::ActionStatus::CANCELED
                             : pddl::ActionStatus::SUCCEEDED;
}

PLUGINLIB_EXPORT_CLASS(omni_plan_dispatcher::SequentialPlanDispatcher,
                       omni_plan::PlanDispatcher)
