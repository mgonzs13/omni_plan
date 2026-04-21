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

#ifndef OMNI_PLAN_DISPATCHER__PARALLEL_PLAN_DISPATCHER_HPP_
#define OMNI_PLAN_DISPATCHER__PARALLEL_PLAN_DISPATCHER_HPP_

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
 * @class ParallelPlanDispatcher
 * @brief Executes plan actions in parallel using a bounded thread pool.
 * @details Actions are dispatched to a fixed-size worker pool as soon as all
 * their causal dependencies have succeeded.  The pool size is controlled by
 * the `plan_dispatcher.execution_threads` ROS parameter (default:
 * `hardware_concurrency`).  Parallel branches never share a single Action
 * object: the dispatcher maintains a per-action instance cache from which
 * extra instances are acquired on demand.
 */
class ParallelPlanDispatcher : public omni_plan::PlanDispatcher {
public:
  /**
   * @brief Constructor. Registers the `execution_threads` ROS parameter.
   */
  ParallelPlanDispatcher();

  /**
   * @brief Default destructor.
   */
  virtual ~ParallelPlanDispatcher() = default;

protected:
  /**
   * @brief Executes all nodes using a bounded thread pool.
   * @details Nodes are submitted to the pool as soon as their dependency
   * counter reaches zero. Workers run `run_node_action()` for each node,
   * never blocking on future results (promises are resolved before dependent
   * nodes are enqueued).
   * @param all_nodes           Graph nodes, indexed by node_num.
   * @return SUCCEEDED if all nodes succeeded, CANCELED if cancelled,
   *   ABORTED if any node aborted.
   */
  omni_plan::pddl::ActionStatus dispatch_actions(
      const std::vector<omni_plan::pddl::GraphNode::Ptr> &all_nodes) override;

private:
  /// @brief Number of worker threads (0 = hardware_concurrency).
  int execution_threads_ = 0;
};

} // namespace omni_plan_dispatcher

#endif // omni_plan_dispatcher__PARALLEL_PLAN_DISPATCHER_HPP_
