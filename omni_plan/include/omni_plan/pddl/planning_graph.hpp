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

#ifndef OMNI_PLAN__PDDL__PLANNING_GRAPH_HPP_
#define OMNI_PLAN__PDDL__PLANNING_GRAPH_HPP_

#include <list>
#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/timing_predicate.hpp"

namespace omni_plan {
namespace pddl {

/**
 * @struct ActionStamped
 * @brief An action with its temporal information from the planner output.
 */
struct ActionStamped {
  float time;
  float duration;
  std::shared_ptr<Action> action;
  std::vector<std::string> params;
};

/**
 * @struct GraphNode
 * @brief A node in the planning execution graph representing an action.
 * @details Each node contains an action with timing info, its dependencies
 * (in_arcs) and dependents (out_arcs), and the state of predicates at the
 * point before this action executes.
 */
struct GraphNode {
  using Ptr = std::shared_ptr<GraphNode>;
  static Ptr make_shared() { return std::make_shared<GraphNode>(); }

  ActionStamped action;
  int node_num = 0;
  int level_num = 0;

  /// Predicates that are true in the state before this action executes.
  std::set<Predicate> predicates;

  /// Nodes that must complete before this one can start.
  std::list<GraphNode::Ptr> in_arcs;
  /// Nodes that depend on this one.
  std::list<GraphNode::Ptr> out_arcs;
};

/**
 * @struct PlanningGraph
 * @brief A directed acyclic graph of plan actions for parallel execution.
 * @details The graph groups actions into levels based on their start times.
 * Root actions can be executed concurrently. Edges represent causal
 * dependencies: a child node can only start after all its parent nodes
 * complete. This follows the approach presented in "Optimized Execution of
 * PDDL Plans using Behavior Trees" (Martín et al., 2021).
 */
struct PlanningGraph {
  using Ptr = std::shared_ptr<PlanningGraph>;
  static Ptr make_shared() { return std::make_shared<PlanningGraph>(); }

  /// Root actions that can be started immediately (no dependencies).
  std::list<GraphNode::Ptr> roots;
  /// Actions grouped by their start time level.
  std::map<float, std::list<GraphNode::Ptr>> levels;
};

/**
 * @class PlanningGraphBuilder
 * @brief Builds a PlanningGraph from a Plan.
 * @details Analyzes the plan actions' start times, conditions, and effects
 * to construct a DAG that enables parallel execution of independent actions.
 * The algorithm:
 * 1. Extract actions with their start times from the plan.
 * 2. Identify root actions (those executable from initial state and
 *    parallelizable with each other).
 * 3. For each remaining action, find causal dependencies (which action's
 *    effects satisfy this action's conditions) and contradictions (which
 *    action's effects would break this action's requirements).
 * 4. Build a DAG with edges representing these dependencies.
 */
class PlanningGraphBuilder {
public:
  /**
   * @brief Constructs a PlanningGraphBuilder with the initial state.
   * @param initial_predicates The predicates true in the initial state.
   */
  explicit PlanningGraphBuilder(
      const std::set<Predicate> &initial_predicates = {});

  /**
   * @brief Builds a planning graph from a plan.
   * @param plan The plan to build the graph from.
   * @return A shared pointer to the constructed PlanningGraph.
   */
  PlanningGraph::Ptr build_graph(const Plan &plan) const;

  /**
   * @brief Gets the execution levels from the planning graph.
   * @details Returns groups of graph nodes ordered by execution level.
   * Nodes within the same level can be executed in parallel.
   * @param graph The planning graph to extract levels from.
   * @return A vector of vectors, where each inner vector contains nodes
   *         to be executed in parallel.
   */
  static std::vector<std::vector<GraphNode::Ptr>>
  get_execution_levels(const PlanningGraph::Ptr &graph);

private:
  /// The initial state predicates.
  std::set<Predicate> initial_predicates_;

  /**
   * @brief Extracts ActionStamped items from a plan.
   */
  std::vector<ActionStamped> get_plan_actions(const Plan &plan) const;

  /**
   * @brief Instantiates predicate arguments with concrete parameter values.
   */
  Predicate instantiate_predicate(const Predicate &pred,
                                  const std::shared_ptr<Action> &action,
                                  const std::vector<std::string> &params) const;

  /**
   * @brief Gets the instantiated at-start conditions for an action.
   */
  std::vector<Predicate>
  get_start_conditions(const ActionStamped &action) const;

  /**
   * @brief Gets the instantiated over-all conditions for an action.
   */
  std::vector<Predicate>
  get_overall_conditions(const ActionStamped &action) const;

  /**
   * @brief Gets the instantiated at-end conditions for an action.
   */
  std::vector<Predicate> get_end_conditions(const ActionStamped &action) const;

  /**
   * @brief Gets all instantiated conditions for an action.
   */
  std::vector<Predicate> get_all_conditions(const ActionStamped &action) const;

  /**
   * @brief Gets the instantiated at-start effects for an action.
   */
  std::vector<Predicate> get_start_effects(const ActionStamped &action) const;

  /**
   * @brief Gets the instantiated at-end effects for an action.
   */
  std::vector<Predicate> get_end_effects(const ActionStamped &action) const;

  /**
   * @brief Applies effects to a set of predicates.
   */
  void apply_effects(const std::vector<Predicate> &effects,
                     std::set<Predicate> &predicates) const;

  /**
   * @brief Checks if an action is executable given the current predicates.
   */
  bool is_action_executable(const ActionStamped &action,
                            const std::set<Predicate> &predicates) const;

  /**
   * @brief Checks if a condition is satisfied by the given predicates.
   */
  bool is_condition_satisfied(const Predicate &condition,
                              const std::set<Predicate> &predicates) const;

  /**
   * @brief Checks if an action can run in parallel with a set of existing
   * nodes.
   * @details Verifies that the action's at-start effects don't conflict with
   * the requirements of existing nodes, and vice versa.
   */
  bool is_parallelizable(const ActionStamped &action,
                         const std::set<Predicate> &predicates,
                         const std::list<GraphNode::Ptr> &existing_nodes) const;

  /**
   * @brief Finds which existing node satisfies a given condition.
   * @details Searches the graph for a node whose effects produce a predicate
   * that satisfies the given condition.
   */
  GraphNode::Ptr find_node_satisfying(const Predicate &condition,
                                      const PlanningGraph::Ptr &graph,
                                      const GraphNode::Ptr &current) const;

  /**
   * @brief Finds nodes that contradict the requirements of a given node.
   */
  std::list<GraphNode::Ptr>
  find_contradicting_nodes(const PlanningGraph::Ptr &graph,
                           const GraphNode::Ptr &current) const;

  /**
   * @brief Gets root actions from the action sequence.
   */
  std::list<GraphNode::Ptr>
  get_roots(std::vector<ActionStamped> &action_sequence,
            std::set<Predicate> &predicates, int &node_counter) const;

  /**
   * @brief Computes the state at a given node by traversing its ancestors.
   */
  std::set<Predicate> compute_state_at_node(const GraphNode::Ptr &node) const;
};

} // namespace pddl
} // namespace omni_plan
#endif // OMNI_PLAN__PDDL__PLANNING_GRAPH_HPP_
