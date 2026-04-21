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

#ifndef OMNI_PLAN_DISPATCHER__SEQUENTIAL_PLAN_DISPATCHER_HPP_
#define OMNI_PLAN_DISPATCHER__SEQUENTIAL_PLAN_DISPATCHER_HPP_

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/planning_graph.hpp"
#include "omni_plan/pddl_manager.hpp"
#include "omni_plan/plan_dispatcher.hpp"

namespace omni_plan_dispatcher {

/**
 * @class SequentialPlanDispatcher
 * @brief Executes plan actions one at a time in topological order.
 * @details Actions are processed level by level (earliest preconditions first).
 * Within each level actions are executed sequentially in node_num order.
 * If any action fails the dispatcher stops immediately and returns ABORTED.
 * This is the simplest dispatcher and is useful for debugging or for
 * environments where parallel execution would cause resource contention.
 */
class SequentialPlanDispatcher : public omni_plan::PlanDispatcher {
public:
  /**
   * @brief Constructor. Registers no additional parameters beyond the base
   * class defaults.
   */
  SequentialPlanDispatcher();

  /**
   * @brief Default destructor.
   */
  virtual ~SequentialPlanDispatcher() = default;

protected:
  /**
   * @brief Executes all actions sequentially in topological (level) order.
   * @param all_nodes           Graph nodes, indexed by node_num.
   * @return SUCCEEDED if every action completed successfully, CANCELED if
   *   the dispatcher was cancelled, ABORTED otherwise.
   */
  omni_plan::pddl::ActionStatus dispatch_actions(
      const std::vector<omni_plan::pddl::GraphNode::Ptr> &all_nodes) override;
};

} // namespace omni_plan_dispatcher

#endif // omni_plan_dispatcher__SEQUENTIAL_PLAN_DISPATCHER_HPP_
