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
#include <list>
#include <map>
#include <memory>
#include <queue>
#include <set>
#include <string>
#include <vector>

#include "omni_plan/pddl/planning_graph.hpp"

using namespace omni_plan::pddl;

PlanningGraphBuilder::PlanningGraphBuilder(
    const std::set<Predicate> &initial_predicates)
    : initial_predicates_(initial_predicates) {}

std::vector<ActionStamped>
PlanningGraphBuilder::get_plan_actions(const Plan &plan) const {
  std::vector<ActionStamped> actions;
  for (size_t i = 0; i < plan.size(); ++i) {
    ActionStamped stamped;
    stamped.time = plan.get_action_start_time(i);
    stamped.duration = plan.get_action_duration(i);
    stamped.action = plan.get_action(i);
    stamped.params = plan.get_action_params(i);
    actions.push_back(stamped);
  }
  return actions;
}

Predicate PlanningGraphBuilder::instantiate_predicate(
    const Predicate &pred, const std::shared_ptr<Action> &action,
    const std::vector<std::string> &params) const {

  auto args = pred.get_args();
  std::vector<std::string> instantiated_args;
  for (const auto &arg : args) {
    int idx = action->get_parameter_index(arg);
    if (idx >= 0 && idx < static_cast<int>(params.size())) {
      instantiated_args.push_back(params[idx]);
    } else {
      instantiated_args.push_back(arg);
    }
  }
  return Predicate(pred.get_name(), instantiated_args, pred.is_negated());
}

std::vector<Predicate>
PlanningGraphBuilder::get_start_conditions(const ActionStamped &action) const {
  std::vector<Predicate> result;
  for (const auto &cond : action.action->get_on_start_conditions()) {
    result.push_back(instantiate_predicate(cond, action.action, action.params));
  }
  return result;
}

std::vector<Predicate> PlanningGraphBuilder::get_overall_conditions(
    const ActionStamped &action) const {
  std::vector<Predicate> result;
  for (const auto &cond : action.action->get_over_all_conditions()) {
    result.push_back(instantiate_predicate(cond, action.action, action.params));
  }
  return result;
}

std::vector<Predicate>
PlanningGraphBuilder::get_end_conditions(const ActionStamped &action) const {
  std::vector<Predicate> result;
  for (const auto &cond : action.action->get_on_end_conditions()) {
    result.push_back(instantiate_predicate(cond, action.action, action.params));
  }
  return result;
}

std::vector<Predicate>
PlanningGraphBuilder::get_all_conditions(const ActionStamped &action) const {
  std::vector<Predicate> result;
  auto start = get_start_conditions(action);
  auto overall = get_overall_conditions(action);
  auto end = get_end_conditions(action);
  result.insert(result.end(), start.begin(), start.end());
  result.insert(result.end(), overall.begin(), overall.end());
  result.insert(result.end(), end.begin(), end.end());
  return result;
}

std::vector<Predicate>
PlanningGraphBuilder::get_start_effects(const ActionStamped &action) const {
  std::vector<Predicate> result;
  for (const auto &eff : action.action->get_on_start_effects()) {
    result.push_back(instantiate_predicate(eff, action.action, action.params));
  }
  return result;
}

std::vector<Predicate>
PlanningGraphBuilder::get_end_effects(const ActionStamped &action) const {
  std::vector<Predicate> result;
  for (const auto &eff : action.action->get_on_end_effects()) {
    result.push_back(instantiate_predicate(eff, action.action, action.params));
  }
  return result;
}

void PlanningGraphBuilder::apply_effects(
    const std::vector<Predicate> &effects,
    std::set<Predicate> &predicates) const {

  for (const auto &eff : effects) {
    if (eff.is_negated()) {
      // Remove the positive version
      Predicate positive(eff.get_name(), eff.get_args(), false);
      predicates.erase(positive);
    } else {
      // Remove negated version if exists, add positive
      Predicate negated(eff.get_name(), eff.get_args(), true);
      predicates.erase(negated);
      predicates.insert(eff);
    }
  }
}

bool PlanningGraphBuilder::is_condition_satisfied(
    const Predicate &condition, const std::set<Predicate> &predicates) const {

  if (condition.is_negated()) {
    // Negated condition is satisfied if the positive predicate is NOT present
    Predicate positive(condition.get_name(), condition.get_args(), false);
    return predicates.find(positive) == predicates.end();
  } else {
    // Positive condition is satisfied if the predicate IS present
    return predicates.find(condition) != predicates.end();
  }
}

bool PlanningGraphBuilder::is_action_executable(
    const ActionStamped &action, const std::set<Predicate> &predicates) const {

  auto start_conds = get_start_conditions(action);
  auto overall_conds = get_overall_conditions(action);

  for (const auto &cond : start_conds) {
    if (!is_condition_satisfied(cond, predicates)) {
      return false;
    }
  }
  for (const auto &cond : overall_conds) {
    if (!is_condition_satisfied(cond, predicates)) {
      return false;
    }
  }
  return true;
}

bool PlanningGraphBuilder::is_parallelizable(
    const ActionStamped &action, const std::set<Predicate> &predicates,
    const std::list<GraphNode::Ptr> &existing_nodes) const {

  // Check: applying this action's at-start effects doesn't break existing
  // actions' requirements
  auto new_preds = predicates;
  this->apply_effects(this->get_start_effects(action), new_preds);

  for (const auto &node : existing_nodes) {
    if (!this->is_action_executable(node->action, new_preds)) {
      return false;
    }
  }

  // Check: applying each existing action's at-start effects doesn't break
  // this action's requirements
  for (const auto &node : existing_nodes) {
    auto temp_preds = predicates;
    this->apply_effects(this->get_start_effects(node->action), temp_preds);
    if (!this->is_action_executable(action, temp_preds)) {
      return false;
    }
  }

  return true;
}

GraphNode::Ptr PlanningGraphBuilder::find_node_satisfying(
    const Predicate &condition, const std::vector<GraphNode::Ptr> &nodes,
    const GraphNode::Ptr &current) const {

  // Scan backwards: the most-recently-added node that first produces
  // `condition` (was absent before its effects, present after) is the causal
  // predecessor we want.  This is O(N×E) with no predicate-set copies.
  for (auto it = nodes.rbegin(); it != nodes.rend(); ++it) {
    const auto &node = *it;
    if (node == current) {
      continue;
    }

    // If condition was already satisfied before this node, it didn't produce it
    if (this->is_condition_satisfied(condition, node->predicates)) {
      continue;
    }

    // Check whether this node's start or end effects produce condition
    for (const auto &eff : this->get_start_effects(node->action)) {
      if (!eff.is_negated() && eff == condition) {
        return node;
      }
    }
    for (const auto &eff : this->get_end_effects(node->action)) {
      if (!eff.is_negated() && eff == condition) {
        return node;
      }
    }
  }

  return nullptr;
}

std::list<GraphNode::Ptr> PlanningGraphBuilder::find_contradicting_nodes(
    const std::vector<GraphNode::Ptr> &nodes,
    const GraphNode::Ptr &current) const {

  std::list<GraphNode::Ptr> contradictions;

  // current's at-start deletions (negated effects) may break existing nodes'
  // at-start or over-all conditions.  Check each processed node once.
  // O(N×E×C) with no predicate-set copies.
  const auto current_start_effs = this->get_start_effects(current->action);

  for (const auto &node : nodes) {
    if (node == current) {
      continue;
    }

    bool contradicts = false;
    for (const auto &eff : current_start_effs) {
      if (!eff.is_negated()) {
        continue; // only deletions can break conditions
      }

      // The positive predicate that would be deleted
      Predicate deleted(eff.get_name(), eff.get_args(), false);

      // Deleted condition must currently be true in node's state
      if (!is_condition_satisfied(deleted, node->predicates)) {
        continue;
      }

      // Check if node needs this predicate as an at-start or over-all condition
      for (const auto &cond : this->get_start_conditions(node->action)) {
        if (cond == deleted) {
          contradicts = true;
          break;
        }
      }
      if (!contradicts) {
        for (const auto &cond : this->get_overall_conditions(node->action)) {
          if (cond == deleted) {
            contradicts = true;
            break;
          }
        }
      }
      if (contradicts) {
        break;
      }
    }

    if (contradicts) {
      contradictions.push_back(node);
    }
  }

  return contradictions;
}

std::list<GraphNode::Ptr>
PlanningGraphBuilder::get_roots(std::vector<ActionStamped> &action_sequence,
                                std::set<Predicate> &predicates,
                                int &node_counter) const {

  std::list<GraphNode::Ptr> roots;

  auto it = action_sequence.begin();
  while (it != action_sequence.end()) {
    if (this->is_action_executable(*it, predicates) &&
        this->is_parallelizable(*it, predicates, roots)) {

      auto new_root = GraphNode::make_shared();
      new_root->action = *it;
      new_root->node_num = node_counter++;
      new_root->level_num = 0;
      new_root->predicates = predicates;

      roots.push_back(new_root);
      it = action_sequence.erase(it);
    } else {
      break;
    }
  }

  return roots;
}

PlanningGraph::Ptr PlanningGraphBuilder::build_graph(const Plan &plan) const {

  int node_counter = 0;
  int level_counter = 0;
  auto graph = PlanningGraph::make_shared();

  auto action_sequence = this->get_plan_actions(plan);
  auto predicates = initial_predicates_;

  // Get root actions that can be run in parallel from initial state
  graph->roots = const_cast<PlanningGraphBuilder *>(this)->get_roots(
      action_sequence, predicates, node_counter);

  // Flat list of ALL nodes in processing order (roots first, then the rest).
  // Used by find_node_satisfying and find_contradicting_nodes to avoid O(N²)
  // DFS graph traversals — a linear scan is sufficient.
  std::vector<GraphNode::Ptr> flat_nodes;
  flat_nodes.reserve(action_sequence.size() + graph->roots.size());
  for (const auto &root : graph->roots) {
    flat_nodes.push_back(root);
  }

  // Advance the incremental world state past all root effects so that the
  // first non-root node's predicates reflect the post-root world.
  for (const auto &root : graph->roots) {
    this->apply_effects(this->get_start_effects(root->action), predicates);
    this->apply_effects(this->get_end_effects(root->action), predicates);
  }

  // Build the rest of the graph
  while (!action_sequence.empty()) {
    auto new_node = GraphNode::make_shared();
    new_node->action = action_sequence.front();
    new_node->node_num = node_counter++;
    float time = new_node->action.time;

    // Add to time-based level
    auto level = graph->levels.find(time);
    if (level == graph->levels.end()) {
      level_counter++;
      std::list<GraphNode::Ptr> new_level;
      new_level.push_back(new_node);
      graph->levels.insert({time, new_level});
    } else {
      level->second.push_back(new_node);
    }

    new_node->level_num = level_counter;

    // Assign the current incremental world state to this node (state before
    // this action executes).  Replaces the old compute_state_at_node() which
    // retraversed all ancestors from scratch for every node (O(N²)).
    new_node->predicates = predicates;

    // Find all conditions of this action
    auto conditions = this->get_all_conditions(new_node->action);

    // Find satisfying nodes (causal links) using flat list
    for (const auto &condition : conditions) {
      auto parent = this->find_node_satisfying(condition, flat_nodes, new_node);
      if (parent != nullptr) {
        if (std::find(new_node->in_arcs.begin(), new_node->in_arcs.end(),
                      parent) == new_node->in_arcs.end()) {
          new_node->in_arcs.push_back(parent);
        }
        if (std::find(parent->out_arcs.begin(), parent->out_arcs.end(),
                      new_node) == parent->out_arcs.end()) {
          parent->out_arcs.push_back(new_node);
        }
      }
    }

    // Find contradicting nodes (mutex links) using flat list
    auto contradictions = this->find_contradicting_nodes(flat_nodes, new_node);
    for (const auto &parent : contradictions) {
      if (std::find(new_node->in_arcs.begin(), new_node->in_arcs.end(),
                    parent) == new_node->in_arcs.end()) {
        new_node->in_arcs.push_back(parent);
      }
      if (std::find(parent->out_arcs.begin(), parent->out_arcs.end(),
                    new_node) == parent->out_arcs.end()) {
        parent->out_arcs.push_back(new_node);
      }
    }

    // Advance incremental state past this action's effects before moving on
    this->apply_effects(this->get_start_effects(new_node->action), predicates);
    this->apply_effects(this->get_end_effects(new_node->action), predicates);

    flat_nodes.push_back(new_node);
    action_sequence.erase(action_sequence.begin());
  }

  return graph;
}

std::vector<std::vector<GraphNode::Ptr>>
PlanningGraphBuilder::get_execution_levels(const PlanningGraph::Ptr &graph) {

  std::vector<std::vector<GraphNode::Ptr>> levels;

  // Compute in-degree for each node using BFS from roots
  std::map<GraphNode::Ptr, int> in_degree;
  std::set<GraphNode::Ptr> all_nodes;

  // Collect all nodes
  std::function<void(const GraphNode::Ptr &)> collect;
  collect = [&](const GraphNode::Ptr &node) {
    if (all_nodes.find(node) != all_nodes.end()) {
      return;
    }
    all_nodes.insert(node);
    in_degree[node] = static_cast<int>(node->in_arcs.size());
    for (const auto &child : node->out_arcs) {
      collect(child);
    }
  };

  for (const auto &root : graph->roots) {
    collect(root);
  }

  // Topological sort by levels (Kahn's algorithm)
  std::queue<GraphNode::Ptr> ready;
  for (const auto &node : all_nodes) {
    if (in_degree[node] == 0) {
      ready.push(node);
    }
  }

  while (!ready.empty()) {
    std::vector<GraphNode::Ptr> current_level;

    size_t level_size = ready.size();
    for (size_t i = 0; i < level_size; ++i) {
      auto node = ready.front();
      ready.pop();
      current_level.push_back(node);
    }

    // Decrease in-degree of successors
    for (const auto &node : current_level) {
      for (const auto &child : node->out_arcs) {
        in_degree[child]--;
        if (in_degree[child] == 0) {
          ready.push(child);
        }
      }
    }

    levels.push_back(current_level);
  }

  return levels;
}
