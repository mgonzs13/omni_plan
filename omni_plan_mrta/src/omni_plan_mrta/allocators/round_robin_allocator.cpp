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

#include "pluginlib/class_list_macros.hpp"

#include "omni_plan_mrta/allocators/round_robin_allocator.hpp"

namespace omni_plan_mrta {

RoundRobinAllocator::RoundRobinAllocator() : TaskAllocator() {}

std::unordered_map<std::string, RobotAllocation> RoundRobinAllocator::allocate(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem & /*problem*/,
    const std::unordered_map<std::string,
                             std::shared_ptr<omni_plan::pddl::Action>>
        & /*actions*/) const {
  const int N = static_cast<int>(robots.size());

  std::unordered_map<std::string, RobotAllocation> result;
  for (const auto &r : robots) {
    result[r] = RobotAllocation{};
  }

  if (N == 0) {
    return result;
  }

  for (int j = 0; j < static_cast<int>(goals.size()); ++j) {
    result[robots[static_cast<size_t>(j % N)]].goal_indices.push_back(j);
  }

  return result;
}

} // namespace omni_plan_mrta

PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::RoundRobinAllocator,
                       omni_plan_mrta::TaskAllocator)
