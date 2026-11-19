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

#ifndef OMNI_PLAN_MRTA__ALLOCATORS__CBBA_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__ALLOCATORS__CBBA_ALLOCATOR_HPP_

#include "omni_plan_mrta/task_allocator.hpp"

namespace omni_plan_mrta {

/**
 * @class CbbaAllocator
 * @brief Consensus-Based Bundle Algorithm (CBBA) with h_add / h_max bidding.
 *
 * @details Two alternating phases iterate until convergence:
 *
 * 1. **Bundle phase** – each robot greedily extends its bundle by adding the
 *    goal it can outbid all others for. Bid:
 *    @code
 *      c(i, j)      = -(h_cost(i,j) * dist_scale + bfs_capped(i,j))
 *      bid(i, j, k) = c(i, j) - alpha * k
 *    @endcode
 *    where @c h_cost is the delete-relaxed heuristic (h_add or h_max),
 *    @c dist_scale = max_finite_bfs + 1 ensures h-cost differences always
 *    dominate BFS differences, and @c bfs_capped(i,j) is the BFS hop-count
 *    from robot @c i to the nearest argument of goal @c j in the object
 *    co-occurrence graph of initial facts — capped at @c dist_scale for
 *    spatially unreachable robots (so they always lose ties to a reachable
 *    robot, even when h-costs are equal). @c alpha is a load-balancing penalty.
 *    Bid arithmetic is computed in @c long @c long and saturated to a safe
 *    integer range, so finite h-costs near @c INT_MAX/2 cannot overflow.
 *
 * 2. **Consensus phase** – for each goal, the robot with the globally highest
 *    bid wins. Losing robots release the goal from their bundle.
 *
 * With h_add (default): costs combine additively across all goals in the
 * bundle. With h_max (@c use_h_max = true): bundle cost reflects the single
 * hardest goal, which naturally clusters similarly-difficult goals on one
 * robot.
 *
 * Goals unassigned after convergence are distributed to the least-loaded robot.
 * A post-convergence rebalancing pass (M ≥ N) then repeatedly steals the
 * minimum-cost-delta goal from the most-loaded robot to the least-loaded one
 * until the spread is ≤ 1 (i.e. every robot holds ⌊M/N⌋ or ⌈M/N⌉ goals).
 * Only h-cost-reachable steals are performed.
 *
 * Reference: Choi, H.-L., Brunet, L., & How, J. P. (2009). Consensus-based
 * decentralized auctions for robust task allocation. IEEE Transactions on
 * Robotics, 25(4), 912–926.
 *
 * ROS parameter: @c allocator.use_h_max (bool, default false).
 */
class CbbaAllocator : public TaskAllocator {
public:
  /**
   * @brief Constructs the allocator.
   * @param use_h_max If true use h_max aggregation; otherwise h_add.
   *        Can also be set via the ROS 2 parameter @c allocator.use_h_max.
   */
  explicit CbbaAllocator(bool use_h_max = false);

  std::vector<TeamAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;

private:
  /// @brief If true use h_max aggregation; otherwise h_add.
  bool use_h_max_;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__CBBA_ALLOCATOR_HPP_
