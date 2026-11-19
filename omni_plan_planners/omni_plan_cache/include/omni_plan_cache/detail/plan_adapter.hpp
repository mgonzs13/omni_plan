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

/**
 * @file plan_adapter.hpp
 * @brief Object-name mapping and structural adaptation of cached plans.
 */

#ifndef OMNI_PLAN_CACHE__DETAIL__PLAN_ADAPTER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__PLAN_ADAPTER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan_cache/types.hpp"

namespace omni_plan_cache {
namespace detail {

/**
 * @class PlanAdapter
 * @brief Placeholder mapping and structural plan adaptation.
 * @details On a structural cache hit the cached plan refers to the object
 * names of the problem it was created for. This class aligns the cached
 * placeholder positions with the new problem and rebuilds the plan with the
 * new names, without any parsing or raw-text manipulation.
 */
class PlanAdapter {
public:
  /**
   * @brief Builds a mapping from old object names to new object names.
   * @details Matches placeholder positions "__obj_<type>_<i>__" between the
   * cached problem and the new, role-sorted object groups. Only placeholders
   * present in both are mapped.
   * @param old_placeholder_to_original Placeholder map stored in the cached
   * plan entry.
   * @param new_objects_by_type Role-sorted object groups of the new problem.
   * @return Map from cached object name to new object name.
   */
  static std::unordered_map<std::string, std::string>
  build_name_mapping(const std::unordered_map<std::string, std::string>
                         &old_placeholder_to_original,
                     const std::vector<ObjectsByType> &new_objects_by_type);

  /**
   * @brief Rebuilds a cached plan with new object names.
   * @details Copies every action of the cached plan, substitutes the names in
   * its parameters via @p old_to_new, preserves start times and solution
   * status, and regenerates raw_output from the adapted plan.
   * @param cached The cached plan entry to adapt.
   * @param old_to_new Mapping from cached names to new names (from
   * build_name_mapping()).
   * @return The adapted plan, ready to be validated and returned.
   */
  static omni_plan::pddl::Plan
  adapt(const CachedPlanData &cached,
        const std::unordered_map<std::string, std::string> &old_to_new);
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__PLAN_ADAPTER_HPP_
