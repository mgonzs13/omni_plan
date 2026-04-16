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

#ifndef OMNI_PLAN_MRTA__ALLOCATORS__GREEDY_AUCTION_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__ALLOCATORS__GREEDY_AUCTION_ALLOCATOR_HPP_

#include "omni_plan_mrta/task_allocator.hpp"

namespace omni_plan_mrta {

/**
 * @class GreedyAuctionAllocator
 * @brief Sequential Single-Item Auction (SSI) with BFS-distance bidding.
 *
 * @details Two objects are adjacent in the co-occurrence graph when they
 * co-appear as arguments of any single ground fact in the initial state.
 * D[i][j] is the BFS hop-count from robot_i to the nearest non-robot argument
 * of goal_j in this graph.
 *
 * @code
 *   score(i, j) = −D[i][j] − load_coeff × load[i]
 * @endcode
 *
 * @c load_coeff = max_finite_distance + 1 ensures load-balancing resolves
 * ties within a distance tier without overriding genuine proximity advantages.
 * When all distances are equal, the result is a perfectly balanced assignment.
 *
 * Reference: Gerkey, B. P. & Matarić, M. J. (2004). A formal analysis and
 * taxonomy of task allocation in multi-robot systems. The International
 * Journal of Robotics Research, 23(9), 939–954.
 * https://doi.org/10.1177/0278364904045564
 * BFS-distance bidding heuristic is a custom extension.
 *
 * Complexity: O(N × M²) time, O(N × M) space.
 */
class GreedyAuctionAllocator : public TaskAllocator {
public:
  GreedyAuctionAllocator();

  std::unordered_map<std::string, RobotAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::unordered_map<std::string,
                                    std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__GREEDY_AUCTION_ALLOCATOR_HPP_
