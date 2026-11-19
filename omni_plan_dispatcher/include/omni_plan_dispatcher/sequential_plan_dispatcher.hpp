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
