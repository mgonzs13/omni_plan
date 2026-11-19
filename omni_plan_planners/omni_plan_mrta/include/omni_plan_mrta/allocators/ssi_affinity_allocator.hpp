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

  std::vector<TeamAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__SSI_AFFINITY_ALLOCATOR_HPP_
