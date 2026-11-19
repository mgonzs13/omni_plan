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
