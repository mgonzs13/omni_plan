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

#ifndef EASY_PLAN_KNOWLEDGE_GRAPH__KG_PDDL_MANAGER_HPP_
#define EASY_PLAN_KNOWLEDGE_GRAPH__KG_PDDL_MANAGER_HPP_

#include <atomic>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "knowledge_graph/graph/edge.hpp"
#include "knowledge_graph/graph/node.hpp"
#include "knowledge_graph/knowledge_graph.hpp"

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan/pddl_manager.hpp"

namespace easy_plan_knowledge_graph {

/**
 * @class KgPddlManager
 * @brief PDDL manager implementation using a knowledge graph for state
 * representation.
 * @details This class extends the base PddlManager to provide knowledge
 * graph-based state management. It maintains the current world state in a
 * knowledge graph and provides methods to query and update this state. The
 * manager can optionally register callbacks to automatically update the state
 * when the knowledge graph changes.
 */
class KgPddlManager : public easy_plan::PddlManager {
public:
  /**
   * @brief Constructs a KgPddlManager with optional callback registration.
   * @details Initializes the knowledge graph connection and optionally
   * registers a callback to automatically update the PDDL state when the
   * knowledge graph changes.
   * @param add_callback Whether to register a callback for automatic state
   * updates (default: true).
   */
  KgPddlManager(bool add_callback = true);

  /**
   * @brief Generates PDDL domain and problem from the current knowledge graph
   * state.
   * @details Creates PDDL representations based on the current state stored in
   * the knowledge graph, including all known facts and goals.
   * @return A pair containing the generated Domain and Problem objects.
   */
  std::pair<easy_plan::pddl::Domain, easy_plan::pddl::Problem>
  get_pddl() const override;

  /**
   * @brief Checks if there are any goals in the knowledge graph.
   * @details Queries the knowledge graph to determine if any goal predicates
   * are currently defined and need to be achieved.
   * @return True if goals exist, false otherwise.
   */
  bool has_goals() const override;

  /**
   * @brief Checks if a predicate exists in the current knowledge graph state.
   * @details Queries the knowledge graph to verify whether the given predicate
   * is currently true in the world state.
   * @param predicate The predicate to check for existence.
   * @return True if the predicate exists in the current state, false otherwise.
   */
  bool
  predicate_exists(const easy_plan::pddl::Predicate &predicate) const override;

  /**
   * @brief Checks if a predicate represents a goal in the knowledge graph.
   * @details Determines whether the given predicate is marked as a goal
   * that needs to be achieved in the knowledge graph.
   * @param predicate The predicate to check.
   * @return True if the predicate is a goal, false otherwise.
   */
  bool
  predicate_is_goal(const easy_plan::pddl::Predicate &predicate) const override;

  /**
   * @brief Applies an effect to update the knowledge graph state.
   * @details Updates the knowledge graph by applying the specified effect,
   * which may add or remove predicates from the current state representation.
   * @param exp The effect to apply to the knowledge graph.
   */
  void apply_effect(const easy_plan::pddl::Effect &exp) override;

private:
  /**
   * @brief Callback function for knowledge graph updates.
   * @details This method is called when the knowledge graph is modified.
   * It processes the changes and updates the internal PDDL state accordingly.
   * @param operation The type of operation performed (add/remove/update).
   * @param element_type The type of elements affected (nodes/edges).
   * @param elements The elements that were modified in the knowledge graph.
   */
  void graph_callback(
      const std::string &operation, const std::string &element_type,
      const std::vector<std::variant<knowledge_graph::graph::Node,
                                     knowledge_graph::graph::Edge>> &elements);

  /// Shared pointer to the knowledge graph instance.
  std::shared_ptr<knowledge_graph::KnowledgeGraph> kg_;
  /// Mutex for thread-safe access to goal-related operations.
  mutable std::mutex goal_mutex_;
  /// Condition variable for goal state synchronization.
  mutable std::condition_variable goal_cv_;
};

} // namespace easy_plan_knowledge_graph
#endif // EASY_PLAN_KNOWLEDGE_GRAPH__KG_PDDL_MANAGER_HPP_