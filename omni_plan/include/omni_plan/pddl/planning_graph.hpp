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
  /// Position of the action in the source plan. Used for stable ordering.
  int plan_index = 0;
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

  /// Breaks the in_arcs/out_arcs reference cycles between nodes so the graph
  /// (and every node it owns) is actually freed.
  ~PlanningGraph();

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
   * @brief Repairs and reorders a plan according to its conditions/effects.
   * @details First attempts a state-aware repair: actions are scheduled in
   * waves, only when their preconditions hold, and conflicting actions are
   * serialized. This preserves safe parallelism while resolving shared
   * resource contention. If that fails, a bounded backtracking search tries
   * to find a valid serial order. Start times are rewritten for the resulting
   * order and the planning graph is attached to the returned plan.
   * @param plan The plan to reorder.
   * @param goals Goals that must hold after the last action. Used to reject a
   * repaired order that satisfies every precondition but destroys a goal.
   * @return The reordered plan. If neither strategy can represent every
   * action, the original plan is returned unchanged.
   */
  Plan reorder_plan(const Plan &plan,
                    const std::set<Predicate> &goals = {}) const;

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
  /// @brief The initial state predicates.
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
   * @details Verifies that neither action's effects conflict with the other's
   * conditions or opposite-polarity effects (classical mutex test).
   */
  bool is_parallelizable(const ActionStamped &action,
                         const std::set<Predicate> &predicates,
                         const std::list<GraphNode::Ptr> &existing_nodes) const;

  /**
   * @brief Classical mutex test between two actions.
   * @details Two actions conflict when an effect of one interferes with a
   * condition of the other (adding a predicate required to be absent or
   * deleting a predicate required to be present), or when both change the
   * same predicate in opposite directions.
   */
  bool actions_conflict(const ActionStamped &a, const ActionStamped &b) const;

  /**
   * @brief Finds which processed node most recently produces a given condition.
   * @details Scans the flat list of already-processed nodes backwards and
   * returns the most recent one whose effects produce the condition (i.e., the
   * condition was absent from the state just before that node and present
   * after). O(N×E) with no predicate-set copies.
   */
  GraphNode::Ptr find_node_satisfying(const Predicate &condition,
                                      const std::vector<GraphNode::Ptr> &nodes,
                                      const GraphNode::Ptr &current) const;

  /**
   * @brief Finds processed nodes whose conditions would be broken by the
   * at-start effects of the given node.
   * @details Iterates the flat processed-node list once. O(N×E×C) with no
   * predicate-set copies.
   */
  std::list<GraphNode::Ptr>
  find_contradicting_nodes(const std::vector<GraphNode::Ptr> &nodes,
                           const GraphNode::Ptr &current) const;

  /**
   * @brief Gets root actions from the action sequence.
   */
  std::list<GraphNode::Ptr>
  get_roots(std::vector<ActionStamped> &action_sequence,
            std::set<Predicate> &predicates, int &node_counter) const;
};

} // namespace pddl
} // namespace omni_plan
#endif // OMNI_PLAN__PDDL__PLANNING_GRAPH_HPP_
