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

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include <pluginlib/class_loader.hpp>

#include "poirot/poirot.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/plan_dispatcher.hpp"

using namespace omni_plan;

class DispatchPlanState : public yasmin::State {

public:
  DispatchPlanState()
      : yasmin::State({
            yasmin_ros::basic_outcomes::SUCCEED,
            yasmin_ros::basic_outcomes::ABORT,
            yasmin_ros::basic_outcomes::CANCEL,
        }) {
    this->set_description(
        "State responsible for executing a given plan, which is expected to be "
        "on the blackboard. The state will execute each action in the plan as "
        "soon as all its dependencies have succeeded, maximizing parallelism "
        "across branches. The state monitors for cancellation and aborts "
        "execution as soon as possible if requested.");
    this->set_outcome_description(yasmin_ros::basic_outcomes::SUCCEED,
                                  "Plan executed successfully");
    this->set_outcome_description(yasmin_ros::basic_outcomes::CANCEL,
                                  "Plan execution was cancelled");
    this->set_outcome_description(yasmin_ros::basic_outcomes::ABORT,
                                  "Plan execution failed due to an error");
    this->add_input_key("plan_dispatcher", "The PlanDispatcher to use");
    this->add_input_key(
        "plan", "The plan to execute, expected to be on the blackboard");
    this->add_input_key(
        "problem",
        "The PDDL problem, expected to be on the blackboard (used for initial "
        "state predicates)");
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    PROFILE_FUNCTION();

    try {

      if (!this->dispatcher_) {
        this->dispatcher_ =
            blackboard->get<std::shared_ptr<omni_plan::PlanDispatcher>>(
                "plan_dispatcher");
      }

      if (!this->dispatcher_) {
        YASMIN_LOG_ERROR("plan_dispatcher is null");
        return yasmin_ros::basic_outcomes::ABORT;
      }

      auto plan = blackboard->get<omni_plan::pddl::Plan>("plan");
      auto problem = blackboard->get<omni_plan::pddl::Problem>("problem");

      const std::set<pddl::Predicate> &initial_predicates = problem.get_facts();

      // Planners such as MrtaPlanner attach an ordering graph to the plan.
      // Only fall back to building one here for planners that do not.
      auto graph = plan.get_planning_graph();
      if (graph) {
        YASMIN_LOG_INFO("Using planning graph attached by the planner");
      } else {
        pddl::PlanningGraphBuilder builder(initial_predicates);
        graph = builder.build_graph(plan);
      }

      // Collect all graph nodes
      auto all_nodes = this->collect_nodes(graph);

      // Renumber sequentially so every node_num is a valid index into
      // [0, total)
      for (size_t i = 0; i < all_nodes.size(); ++i) {
        all_nodes[i]->node_num = static_cast<int>(i);
      }

      int total = static_cast<int>(all_nodes.size());

      if (total == 0) {
        return yasmin_ros::basic_outcomes::SUCCEED;
      }

      YASMIN_LOG_INFO("Planning graph built with %d nodes for branch execution",
                      total);

      auto result = this->dispatcher_->dispatch_plan(all_nodes);

      if (result == pddl::ActionStatus::SUCCEEDED) {
        return yasmin_ros::basic_outcomes::SUCCEED;
      } else if (result == pddl::ActionStatus::CANCELED) {
        return yasmin_ros::basic_outcomes::CANCEL;
      } else {
        return yasmin_ros::basic_outcomes::ABORT;
      }

    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to dispatch plan: %s", e.what());
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void cancel_state() override {
    if (this->dispatcher_) {
      this->dispatcher_->cancel_plan();
    }
    yasmin::State::cancel_state();
  }

private:
  std::shared_ptr<omni_plan::PlanDispatcher> dispatcher_;

  std::vector<pddl::GraphNode::Ptr>
  collect_nodes(const pddl::PlanningGraph::Ptr &graph) {
    std::vector<pddl::GraphNode::Ptr> nodes;
    std::set<pddl::GraphNode::Ptr> visited;

    std::function<void(const pddl::GraphNode::Ptr &)> traverse;
    traverse = [&](const pddl::GraphNode::Ptr &n) {
      if (!visited.insert(n).second) {
        return;
      }
      nodes.push_back(n);
      for (const auto &child : n->out_arcs) {
        traverse(child);
      }
    };

    for (const auto &root : graph->roots) {
      traverse(root);
    }

    return nodes;
  }
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(DispatchPlanState, yasmin::State)
