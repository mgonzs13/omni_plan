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

#include "omni_plan_cache/detail/plan_adapter.hpp"

namespace omni_plan_cache {
namespace detail {

std::unordered_map<std::string, std::string> PlanAdapter::build_name_mapping(
    const std::unordered_map<std::string, std::string>
        &old_placeholder_to_original,
    const std::vector<ObjectsByType> &new_objects_by_type) {

  std::unordered_map<std::string, std::string> mapping;
  for (const auto &group : new_objects_by_type) {
    for (size_t i = 0; i < group.names.size(); ++i) {
      const std::string placeholder =
          "__obj_" + group.type + "_" + std::to_string(i) + "__";
      auto it = old_placeholder_to_original.find(placeholder);
      if (it != old_placeholder_to_original.end()) {
        mapping[it->second] = group.names[i];
      }
    }
  }
  return mapping;
}

omni_plan::pddl::Plan PlanAdapter::adapt(
    const CachedPlanData &cached,
    const std::unordered_map<std::string, std::string> &old_to_new) {

  omni_plan::pddl::Plan adapted;
  for (size_t i = 0; i < cached.plan.size(); ++i) {
    auto [action, params] = cached.plan.get_action_with_params(i);
    for (auto &param : params) {
      auto it = old_to_new.find(param);
      if (it != old_to_new.end()) {
        param = it->second;
      }
    }
    adapted.add_action(action, params, cached.plan.get_action_start_time(i));
  }
  adapted.set_has_solution(cached.plan.has_solution());
  adapted.set_raw_output(adapted.to_pddl());
  return adapted;
}

} // namespace detail
} // namespace omni_plan_cache
