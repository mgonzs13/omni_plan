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

#include <algorithm>

#include "pluginlib/class_list_macros.hpp"

#include "omni_plan_mrta/allocators/round_robin_allocator.hpp"

namespace omni_plan_mrta {

RoundRobinAllocator::RoundRobinAllocator() : TaskAllocator() {}

std::vector<TeamAllocation> RoundRobinAllocator::allocate(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem & /*problem*/,
    const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        & /*actions*/) const {
  const int N = static_cast<int>(robots.size());
  const int M = static_cast<int>(goals.size());

  std::vector<TeamAllocation> result;
  if (N == 0 || M == 0) {
    return result;
  }

  // One entry per robot, initialised empty.
  result.reserve(static_cast<size_t>(N));
  for (const auto &r : robots) {
    result.push_back(TeamAllocation{{r}, {}});
  }

  for (int j = 0; j < M; ++j) {
    result[static_cast<size_t>(j % N)].goal_indices.push_back(j);
  }

  // Drop entries with no goals.
  result.erase(std::remove_if(result.begin(), result.end(),
                              [](const TeamAllocation &t) {
                                return t.goal_indices.empty();
                              }),
               result.end());

  return result;
}

} // namespace omni_plan_mrta

PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::RoundRobinAllocator,
                       omni_plan_mrta::TaskAllocator)
