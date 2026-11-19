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

#ifndef OMNI_PLAN_MRTA__ALLOCATORS__ROUND_ROBIN_ALLOCATOR_HPP_
#define OMNI_PLAN_MRTA__ALLOCATORS__ROUND_ROBIN_ALLOCATOR_HPP_

#include "omni_plan_mrta/task_allocator.hpp"

namespace omni_plan_mrta {

/**
 * @class RoundRobinAllocator
 * @brief Assigns goals to robots in a cyclic round-robin order.
 *
 * @details Goal g_j is assigned to robot r_{j mod N}. This is a purely
 * structural allocator with no knowledge of the problem domain. It guarantees
 * perfect load balance but ignores proximity or cost information.
 *
 * Complexity: O(M).
 */
class RoundRobinAllocator : public TaskAllocator {
public:
  RoundRobinAllocator();

  std::vector<TeamAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__ROUND_ROBIN_ALLOCATOR_HPP_
