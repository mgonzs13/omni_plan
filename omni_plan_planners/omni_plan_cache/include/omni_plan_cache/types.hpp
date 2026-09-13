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
 * @file types.hpp
 * @brief Value types shared by the cache planner and its internal helpers.
 * @details Defines the object grouping used for structural normalization, the
 * cached-plan container shared by both cache levels, and the performance
 * counter snapshot exposed by CachePlanner::get_cache_stats().
 */

#ifndef OMNI_PLAN_CACHE__TYPES_HPP_
#define OMNI_PLAN_CACHE__TYPES_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/plan.hpp"

namespace omni_plan_cache {

/**
 * @struct ObjectsByType
 * @brief Groups object names by their PDDL type.
 * @details Produced by StructuralKeyer::group_objects_by_type() and consumed
 * by the structural keying and name-adaptation helpers. Groups are ordered by
 * type name; the order of the names inside a group is the order supplied by
 * the producer (role-sorted before cache-key computation).
 */
struct ObjectsByType {
  /// @brief PDDL type shared by every name in this group (e.g. "robot").
  std::string type;
  /// @brief Object names belonging to this type.
  std::vector<std::string> names;
};

/**
 * @struct CachedPlanData
 * @brief A cached plan plus the placeholder map needed to adapt object names.
 * @details Entries are stored as
 * std::shared_ptr<const CachedPlanData> so a single instance can be referenced
 * by both the exact cache and the structural cache. The placeholder map
 * records which concrete object each "__obj_<type>_<i>__" position referred
 * to, allowing structural hits to rename the cached plan for a problem with
 * different object names.
 */
struct CachedPlanData {
  /// @brief Parsed plan as produced by the planner (or adapted on reuse).
  omni_plan::pddl::Plan plan;
  /// @brief Maps "__obj_<type>_<i>__" placeholders to the concrete object
  /// names of the problem this entry was created for.
  std::unordered_map<std::string, std::string> placeholder_to_original;
};

/// @brief Backwards-compatible alias for the pre-refactor struct name.
using CachedPlan = CachedPlanData;

/**
 * @struct CacheStats
 * @brief Snapshot of the PlanCache performance counters.
 * @details Counters are cumulative since the cache was created and are meant
 * for diagnostics and benchmarking; the two size fields are instantaneous.
 */
struct CacheStats {
  /// @brief Exact-cache lookups that found an entry.
  uint64_t exact_hits = 0;
  /// @brief Exact-cache lookups that found no entry.
  uint64_t exact_misses = 0;
  /// @brief Structural-cache lookups that found an entry (recorded before
  /// the hit is adapted/validated).
  uint64_t structural_hits = 0;
  /// @brief Requests without a usable cache hit that had to be planned by
  /// the sub-planner (includes structural hits rejected by the validator).
  uint64_t full_misses = 0;
  /// @brief Structural hits re-validated before being returned.
  uint64_t validations = 0;
  /// @brief Structural hits whose object names had to be adapted.
  uint64_t adaptations = 0;
  /// @brief Component-composed plans accepted by the validator and cached.
  uint64_t compositions = 0;
  /// @brief Composed plans rejected by the validator (full solve used).
  uint64_t composition_fallbacks = 0;
  /// @brief Exact-cache entries evicted after a bound was configured.
  uint64_t exact_evictions = 0;
  /// @brief Structural-cache entries evicted after a bound was configured.
  uint64_t structural_evictions = 0;
  /// @brief Current number of exact-cache entries.
  size_t exact_entries = 0;
  /// @brief Current number of structural-cache entries.
  size_t structural_entries = 0;
};

} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__TYPES_HPP_
