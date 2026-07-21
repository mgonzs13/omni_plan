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

#ifndef OMNI_PLAN_CACHE__CACHE_PLANNER_HPP_
#define OMNI_PLAN_CACHE__CACHE_PLANNER_HPP_

#include <memory>
#include <set>
#include <shared_mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/planner.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace omni_plan_cache {

/**
 * @struct CachedPlan
 * @brief Stores a cached plan with its object-name mapping for structural
 * reuse.
 * @details Holds the raw planner output and the mapping from type-indexed
 * placeholders to concrete object names, enabling name substitution when a
 * structurally isomorphic problem is encountered.
 */
struct CachedPlan {
  /// @brief The raw planner output string.
  std::string raw_output;
  /// @brief Whether the cached plan represents a valid solution.
  bool has_solution;
  /// @brief Maps placeholders (e.g., "__obj_robot_0__") to original object
  /// names.
  std::unordered_map<std::string, std::string> placeholder_to_original;
};

/**
 * @struct ObjectsByType
 * @brief Groups object names by their PDDL type for structural normalization.
 */
struct ObjectsByType {
  /// @brief The PDDL type name (e.g., "robot", "location").
  std::string type;
  /// @brief The object names belonging to this type.
  std::vector<std::string> names;
};

/**
 * @class CachePlanner
 * @brief Planner implementation that caches plans using exact and structural
 * hashing to avoid redundant planner invocations.
 * @details This class wraps another planner plugin and provides a two-level
 * caching mechanism. On the first level, it caches plans by the SHA-256 hash
 * of the full domain and problem PDDL. On the second level, it normalizes
 * object names into type-indexed placeholders to identify structurally
 * isomorphic problems. When a structural match is found, object names are
 * substituted into the cached plan, avoiding a call to the underlying planner.
 */
class CachePlanner : public omni_plan::Planner {
public:
  /**
   * @brief Default constructor for CachePlanner.
   * @details Initializes the pluginlib class loader for loading the wrapped
   * planner and declares the planner_plugin ROS parameter.
   */
  CachePlanner();

  /**
   * @brief Default destructor.
   */
  ~CachePlanner() override = default;

  /**
   * @brief Generates a plan with two-level caching.
   * @details Computes an exact hash of the domain and problem PDDL. If a match
   * is found in the exact cache, returns the cached plan directly. Otherwise,
   * normalizes object names to type-indexed placeholders and checks the
   * structural cache. On a structural hit, substitutes new object names into
   * the cached raw output. On a complete miss, delegates to the wrapped
   * planner and populates both caches.
   * @param domain The PDDL domain definition.
   * @param problem The PDDL problem definition.
   * @return A Plan object containing the solution or indicating no solution
   * found.
   */
  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem &problem) const override;

  using Planner::generate_plan;

  /**
   * @brief Parses the raw planner output into a Plan object.
   * @details Extracts actions and their parameters from the planner's output
   * and populates a Plan object. This method is used for both exact and
   * structural cache hits.
   * @param domain The PDDL domain containing the action definitions.
   * @param raw_output The raw output string from the planner.
   * @return A Plan object containing the parsed actions.
   */
  omni_plan::pddl::Plan parse_plan(const omni_plan::pddl::Domain &domain,
                                   const std::string &str_plan) const override;

  /**
   * @brief Computes the SHA-256 hash of a string.
   * @param input The input string to hash.
   * @return A 64-character hexadecimal string representing the hash.
   */
  static std::string sha256(const std::string &input);

  /**
   * @brief Groups objects by their PDDL type.
   * @details Iterates over a set of PDDL objects and collects them into
   * groups keyed by type, preserving type insertion order.
   * @param objects The set of objects to group.
   * @return A vector of ObjectsByType entries, one per unique type.
   */
  static std::vector<ObjectsByType>
  group_objects_by_type(const std::set<omni_plan::pddl::Object> &objects);

  /**
   * @brief Normalizes a PDDL problem string by replacing object names with
   * type-indexed placeholders.
   * @details Each object name is replaced with a placeholder of the form
   * "__obj_{type}_{index}__". Replacements use word-boundary-aware matching
   * to avoid corrupting substrings. Names are processed in descending length
   * order to prevent partial-name issues.
   * @param pddl_str The PDDL problem string to normalize.
   * @param objects_by_type The object groupings produced by
   * group_objects_by_type.
   * @param out_placeholder_map Output map from placeholder strings to original
   * object names.
   * @return The normalized PDDL string with placeholders.
   */
  static std::string normalize_pddl(
      const std::string &pddl_str,
      const std::vector<ObjectsByType> &objects_by_type,
      std::unordered_map<std::string, std::string> &out_placeholder_map);

  /**
   * @brief Builds a mapping from old object names to new object names for
   * structural cache adaptation.
   * @details Aligns placeholders by their type and index across the cached
   * and new problem. For each placeholder that exists in both, maps the old
   * object name to the new object name.
   * @param old_placeholder_to_original The placeholder-to-name map from the
   * cached problem.
   * @param new_objects_by_type The object groupings from the current problem.
   * @return A map from old object names to new object names.
   */
  static std::unordered_map<std::string, std::string>
  build_name_mapping(const std::unordered_map<std::string, std::string>
                         &old_placeholder_to_original,
                     const std::vector<ObjectsByType> &new_objects_by_type);

private:
  /// @brief Node used for logging and parameter loading.
  mutable std::shared_ptr<rclcpp::Node> node_;
  /// @brief Pluginlib class loader for instantiating the wrapped planner.
  mutable std::unique_ptr<pluginlib::ClassLoader<omni_plan::Planner>>
      planner_loader_;
  /// @brief The wrapped planner instance, loaded eagerly after parameters.
  mutable std::shared_ptr<omni_plan::Planner> wrapped_planner_;
  /// @brief The pluginlib class name of the wrapped planner plugin.
  std::string wrapped_planner_name_;

  /// @brief Cache mapping exact hashes to fully parsed Plan objects.
  mutable std::unordered_map<std::string, omni_plan::pddl::Plan> exact_cache_;
  /// @brief Cache mapping structural hashes to CachedPlan entries.
  mutable std::unordered_map<std::string, CachedPlan> structural_cache_;
  /// @brief Mutex protecting concurrent access to both caches.
  mutable std::shared_mutex cache_mutex_;
};

} // namespace omni_plan_cache
#endif // OMNI_PLAN_CACHE__CACHE_PLANNER_HPP_
