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

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>

#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"
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
    this->add_input_key(
        "plan", "The plan to execute, expected to be on the blackboard");
    this->add_input_key("actions", "Map of action name to Action plugin, "
                                   "expected to be on the blackboard");
    this->add_input_key(
        "problem",
        "The PDDL problem, expected to be on the blackboard (used for initial "
        "state predicates)");
    this->add_input_key("pddl_manager",
                        "The PDDL manager, expected to be on the blackboard");
    this->add_input_key(
        "actions_plugins",
        "The map of action name to Action plugin, expected to be on "
        "the blackboard (used for cloning actions for parallel "
        "branches)");
  }

  void configure() override {
    auto node = yasmin_ros::YasminNode::get_instance();
    this->dispatcher_ = std::make_unique<omni_plan::PlanDispatcher>(node);
    this->dispatcher_->load_ros_parameters(node);
  }

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    auto pddl_manager =
        blackboard->get<std::shared_ptr<omni_plan::PddlManager>>(
            "pddl_manager");
    auto plan = blackboard->get<omni_plan::pddl::Plan>("plan");
    auto actions_map = blackboard->get<std::unordered_map<
        std::string, std::shared_ptr<omni_plan::pddl::Action>>>("actions");
    auto actions_plugins_map =
        blackboard->get<std::unordered_map<std::string, std::string>>(
            "actions_plugins");
    auto problem = blackboard->get<omni_plan::pddl::Problem>("problem");

    const std::set<pddl::Predicate> &initial_predicates = problem.get_facts();

    // Build the planning graph
    pddl::PlanningGraphBuilder builder(initial_predicates);
    auto graph = builder.build_graph(plan);

    // Collect all graph nodes
    auto all_nodes = this->collect_nodes(graph);

    // Renumber sequentially so every node_num is a valid index into [0, total)
    for (size_t i = 0; i < all_nodes.size(); ++i) {
      all_nodes[i]->node_num = static_cast<int>(i);
    }

    int total = static_cast<int>(all_nodes.size());

    if (total == 0) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    }

    YASMIN_LOG_INFO("Planning graph built with %d nodes for branch execution",
                    total);

    auto result = this->dispatcher_->dispatch_plan(
        all_nodes, actions_map, actions_plugins_map, pddl_manager);

    if (result == pddl::ActionStatus::SUCCEED) {
      return yasmin_ros::basic_outcomes::SUCCEED;
    } else if (result == pddl::ActionStatus::CANCEL) {
      return yasmin_ros::basic_outcomes::CANCEL;
    } else {
      return yasmin_ros::basic_outcomes::ABORT;
    }
  }

  void cancel_state() override {
    this->dispatcher_->cancel_plan();
    yasmin::State::cancel_state();
  }

private:
  std::unique_ptr<omni_plan::PlanDispatcher> dispatcher_;

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
