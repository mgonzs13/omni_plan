// Copyright (C) 2026 Miguel Ángel González Santamarta
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

  // Marks every node that has not been processed yet as SKIPPED and publishes
  // a final status update, so consumers never see downstream nodes stuck in
  // PENDING after an abort or cancellation.
  auto mark_remaining_skipped = [&](size_t first_remaining) {
    for (size_t i = first_remaining; i < ordered.size(); ++i) {
      this->set_node_status(ordered[i]->node_num,
                            omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
    }
    this->publish_exec_status(
        omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);
  };

  for (size_t i = 0; i < ordered.size(); ++i) {
    const auto &node = ordered[i];
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
      mark_remaining_skipped(i + 1);
      this->clear_current_actions();
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
    } else if (result == pddl::ActionStatus::SKIPPED) {
      this->set_node_status(idx,
                            omni_plan_msgs::msg::PlanActionStatus::SKIPPED);
    } else {
      this->set_node_status(idx, omni_plan_msgs::msg::PlanActionStatus::FAILED);
    }

    this->publish_exec_status(
        omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

    if (result == pddl::ActionStatus::CANCELED) {
      mark_remaining_skipped(i + 1);
      this->clear_current_actions();
      return pddl::ActionStatus::CANCELED;
    }

    if (result == pddl::ActionStatus::ABORTED) {
      mark_remaining_skipped(i + 1);
      this->clear_current_actions();
      return pddl::ActionStatus::ABORTED;
    }
  }

  this->clear_current_actions();
  return this->is_canceled() ? pddl::ActionStatus::CANCELED
                             : pddl::ActionStatus::SUCCEEDED;
}

PLUGINLIB_EXPORT_CLASS(SequentialPlanDispatcher, omni_plan::PlanDispatcher)
