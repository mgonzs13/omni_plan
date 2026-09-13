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
  static std::unordered_map<std::string, std::string> build_name_mapping(
      const std::unordered_map<std::string, std::string>
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
