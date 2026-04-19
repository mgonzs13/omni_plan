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
           const std::unordered_map<std::string,
                                    std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override;
};

} // namespace omni_plan_mrta

#endif // OMNI_PLAN_MRTA__ALLOCATORS__ROUND_ROBIN_ALLOCATOR_HPP_
