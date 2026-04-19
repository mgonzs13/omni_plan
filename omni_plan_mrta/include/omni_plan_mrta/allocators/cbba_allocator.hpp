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
 *      bid(i, j, k) = -h_cost(i, j) - alpha * k
 *    @endcode
 *    where @c h_cost is the delete-relaxed heuristic (h_add or h_max) and
 *    @c alpha = max_cost / (M + 1) is a load-balancing penalty.
 *
 * 2. **Consensus phase** – for each goal, the robot with the globally highest
 *    bid wins. Losing robots release the goal from their bundle.
 *
 * With h_add (default): costs combine additively — no inter-goal synergy.
 * With h_max (@c use_h_max = true): CBBA naturally clusters goals of similar
 * difficulty on the same robot.
 *
 * Goals unassigned after convergence are distributed to the least-loaded robot.
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
           const std::unordered_map<std::string,
                                    std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;

private:
  bool use_h_max_;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__CBBA_ALLOCATOR_HPP_
