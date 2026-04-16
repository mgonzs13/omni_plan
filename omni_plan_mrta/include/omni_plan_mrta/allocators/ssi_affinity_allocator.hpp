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

#ifndef OMNI_PLAN_MRTA__ALLOCATORS__SSI_AFFINITY_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__ALLOCATORS__SSI_AFFINITY_ALLOCATOR_HPP_

#include "omni_plan_mrta/task_allocator.hpp"

namespace omni_plan_mrta {

/**
 * @class SsiAffinityAllocator
 * @brief Sequential Single-Item Auction (SSI) with 1-hop affinity bidding.
 *
 * @details The bid of robot i for goal j is the count of initial-state facts
 * that directly co-mention robot_i and at least one non-robot argument of g_j:
 * @code
 *   affinity(i, j) = |{f ∈ S₀ | robot_i ∈ args(f) ∧ args(g_j) ∩ args(f) ≠ ∅}|
 *   score(i, j)    = affinity(i, j) − load(i)
 * @endcode
 *
 * Limitation: captures only direct (1-hop) co-occurrence. Prefer
 * GreedyAuctionAllocator when indirect proximity matters.
 *
 * Complexity: O(N × M²) time, O(N × M) space.
 */
class SsiAffinityAllocator : public TaskAllocator {
public:
  SsiAffinityAllocator();

  std::unordered_map<std::string, RobotAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::unordered_map<std::string,
                                    std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__SSI_AFFINITY_ALLOCATOR_HPP_
