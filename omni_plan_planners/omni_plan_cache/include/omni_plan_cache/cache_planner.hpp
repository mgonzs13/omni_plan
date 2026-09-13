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
 * @file cache_planner.hpp
 * @brief Public CachePlanner plugin: a two-level plan cache wrapping another
 * planner plugin.
 */

#ifndef OMNI_PLAN_CACHE__CACHE_PLANNER_HPP_
#define OMNI_PLAN_CACHE__CACHE_PLANNER_HPP_

#include <memory>
#include <optional>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/plan_validator.hpp"
#include "omni_plan/planner.hpp"
#include "omni_plan_cache/detail/plan_cache.hpp"
#include "omni_plan_cache/detail/relevance_analyzer.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"
#include "omni_plan_cache/types.hpp"
#include "yasmin_ros/yasmin_node.hpp"

namespace omni_plan_cache {

/**
 * @class CachePlanner
 * @brief Planner implementation that caches plans using exact and structural
 * hashing to avoid redundant planner invocations.
 * @details Wraps another planner plugin loaded via the `planner_plugin`
 * parameter and provides two cache levels:
 *  - an exact cache keyed by a SHA-256 digest of every field of the domain
 *    and problem, and
 *  - a structural cache keyed by a role-normalized abstraction that ignores
 *    concrete object names, allowing isomorphic problems to reuse a plan
 *    after renaming its action parameters.
 *
 * On a structural hit the cached parsed plan is adapted (no re-parsing or raw
 * text manipulation), optionally re-validated by the `validator_plugin`, and
 * returned. When the goals decompose into independent components and a
 * validator is available, sub-plans are solved (and cached) per component and
 * stitched together, with the composed plan validated against the full
 * problem. Concurrent misses on the same structure are deduplicated through
 * single-flight coordination.
 */
class CachePlanner : public omni_plan::Planner {
public:
  /**
   * @brief Default constructor for CachePlanner.
   * @details Initializes the pluginlib class loader for the wrapped planner
   * and declares the ROS parameters (`planner_plugin`, `validator_plugin`,
   * `validate_on_hit`, `robot_type`, `component_goal_limit`,
   * `component_priority_predicate`, `max_exact_cache_entries`,
   * `max_structural_cache_entries`). The wrapped planner and validator are
   * instantiated by the loaded-parameters callback.
   */
  CachePlanner();

  /**
   * @brief Default destructor.
   */
  ~CachePlanner() override = default;

  /**
   * @brief Generates a plan with two-level caching.
   * @details The lookup order is:
   *  1. Exact cache: a canonical digest of the whole domain and problem.
   *  2. Structural cache: role-normalized key; on a hit the cached plan is
   *     adapted to the new object names and validated when required.
   *  3. Component composition: if a validator is loaded and the goals split
   *     into small independent components, each component is solved through
   *     the cache and the sub-plans are stitched and validated.
   *  4. Cache miss: `delegate_plan()` is called under single-flight
   *     coordination. Composed and delegated results are cached only when
   *     `should_cache_result()` returns true; the single-flight leader
   *     re-checks both cache levels after taking ownership to close the
   *     check-then-act gap, and re-entrant callers never publish over the
   *     flight owned by their caller.
   * @param domain The PDDL domain definition.
   * @param problem The PDDL problem definition.
   * @return A Plan object containing the solution or indicating no solution
   * found.
   */
  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem &problem) const override;

  /**
   * @brief Brings the base class overloads into scope.
   * @details `Planner` declares a protected path-based `generate_plan`; this
   * using-declaration keeps it visible alongside the override above, as
   * required by the base class contract.
   */
  using Planner::generate_plan;

  /**
   * @brief Computes the SHA-256 hash of a string.
   * @details Provided for convenience and tests; cache keys are computed with
   * the streaming detail::StructuralKeyer helpers.
   * @param input The input string to hash.
   * @return A 64-character hexadecimal string representing the hash.
   */
  static std::string sha256(const std::string &input);

  /**
   * @brief Groups objects by their PDDL type.
   * @details Iterates over a set of PDDL objects and collects them into
   * groups keyed by type. Groups are ordered by type name; names within a
   * group keep the order of the input set.
   * @param objects The set of objects to group.
   * @return A vector of ObjectsByType entries, one per unique type.
   */
  static std::vector<ObjectsByType>
  group_objects_by_type(const std::set<omni_plan::pddl::Object> &objects);

  /**
   * @brief Computes a role signature for each object based on its usage in
   * predicates.
   * @details Objects that appear in the same predicates at the same argument
   * positions (fact vs goal) get the same signature, enabling structural
   * cache matching across problems with different object names but the same
   * structure.
   * @param objects_by_type The grouped objects to compute signatures for.
   * @param facts The initial-state predicates from the problem.
   * @param goals The goal predicates from the problem.
   * @param name_to_alias Optional alias map used to embed co-occurring
   * object identities into concrete keys; nullptr yields structural keys.
   * @param abstract_keys When true, ignore @p name_to_alias and emit keys
   * that only encode predicate usage.
   * @return A map from object name to role-key string.
   */
  static std::unordered_map<std::string, std::string> compute_role_keys(
      const std::vector<ObjectsByType> &objects_by_type,
      const std::set<omni_plan::pddl::Predicate> &facts,
      const std::set<omni_plan::pddl::Predicate> &goals,
      const std::unordered_map<std::string, std::string> *name_to_alias =
          nullptr,
      bool abstract_keys = false);

  /**
   * @brief Computes a role-aware structural hash for cache matching.
   * @details Hashes the domain plus a canonical representation built from:
   *   1. Object-type counts
   *   2. Sorted list of type-abstracted init predicates
   *   3. Sorted list of type-abstracted goal predicates
   *   4. Sorted list of role-key signatures (ignoring which concrete
   *      object owns each signature, so swapping two objects with the
   *      same connectivity preserves the key).
   *
   * Two problems produce the same key iff they have the same types, same
   * object counts per type, same type-abstracted predicates, AND the same
   * multiset of role signatures - meaning objects can only swap roles if
   * they have identical connectivity (like kitchen and bedroom in the demo
   * graph, both connected to 2 neighbors).
   * @param domain_pddl The domain PDDL string.
   * @param problem The PDDL problem.
   * @param objects_by_type The object groupings (role-sorted for
   * deterministic placeholder indices on cache hit).
   * @param role_keys Map from object name to role-key string.
   * @param filtered_facts Relevant facts to abstract; when nullptr, all
   * problem facts are used.
   * @return A SHA-256 hash serving as the structural cache key.
   */
  static std::string compute_structural_key(
      const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
      const std::vector<ObjectsByType> &objects_by_type,
      const std::unordered_map<std::string, std::string> &role_keys,
      const std::set<omni_plan::pddl::Predicate> *filtered_facts = nullptr);

  /**
   * @brief Computes the set of predicate names relevant to achieving the goals.
   * @details Uses backward chaining from the goal predicates through action
   * effects and preconditions. Any predicate that can influence goal
   * achievement (by appearing in a precondition chain) is considered relevant.
   * Predicates that never appear are irrelevant and can be safely excluded from
   * structural caching.
   * @param domain The PDDL domain with action definitions.
   * @param problem The PDDL problem with initial facts and goals.
   * @return A set of predicate names that are relevant to the plan.
   */
  static std::set<std::string>
  compute_relevant_predicates(const omni_plan::pddl::Domain &domain,
                              const omni_plan::pddl::Problem &problem);

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

  /**
   * @brief Returns a snapshot of the cache performance counters.
   * @details See CacheStats for the meaning of each counter. Counter reads
   * are atomic; entry sizes are read under the cache lock.
   * @return A CacheStats snapshot valid at the time of the call.
   */
  omni_plan_cache::CacheStats get_cache_stats() const;

protected:
  /// @brief The wrapped planner instance, loaded eagerly after parameters.
  mutable std::shared_ptr<omni_plan::Planner> wrapped_planner_;

  /// @brief Node used for logging and parameter loading.
  mutable std::shared_ptr<rclcpp::Node> node_;

  /**
   * @brief Invoked on a cache miss to perform the actual planning.
   *
   * The default implementation delegates to the wrapped planner loaded via
   * the planner_plugin parameter.  Subclasses (e.g. HomeostaticPlanner) may
   * override this to select among multiple planners instead.
   *
   * @param domain          The PDDL domain.
   * @param problem         The PDDL problem.
   * @param structural_key  The role-aware structural hash pre-computed by
   *                        generate_plan (available for subclasses that need
   *                        a deterministic problem identity).
   * @return The plan produced by the underlying planner.
   */
  virtual omni_plan::pddl::Plan
  delegate_plan(const omni_plan::pddl::Domain &domain,
                const omni_plan::pddl::Problem &problem,
                const std::string &structural_key) const;

  /**
   * @brief Determines whether a plan should be stored in the cache.
   *
   * The default returns plan.has_solution(), i.e. only plans that contain a
   * solution are cached.  Subclasses may override to apply additional
   * filtering.
   *
   * @param plan The plan just produced by delegate_plan.
   * @return true if the plan should be cached, false otherwise.
   */
  virtual bool should_cache_result(const omni_plan::pddl::Plan &plan) const;

  /// @brief Pluginlib class loader for instantiating planner plugins.
  mutable std::unique_ptr<pluginlib::ClassLoader<omni_plan::Planner>>
      planner_loader_;

  /// @brief The loaded plan validator for validating adapted plans (optional).
  mutable std::shared_ptr<omni_plan::PlanValidator> validator_;
  /// @brief Pluginlib class loader for instantiating validator plugins.
  mutable std::unique_ptr<pluginlib::ClassLoader<omni_plan::PlanValidator>>
      validator_loader_;
  /// @brief Whether structural cache hits are re-validated by the validator
  /// before being returned (default true; disable to make hits cheap).
  mutable bool validate_on_hit_;

private:
  /// @brief The pluginlib class name of the wrapped planner plugin.
  std::string wrapped_planner_name_;
  /// @brief The pluginlib class name of the validator plugin (optional).
  std::string validator_plugin_name_;

  /// @brief Robot object type used by component composition and relevance.
  std::string robot_type_;
  /// @brief Goal predicate whose component is ordered first ("battery_ok" by
  /// default, matching the historical hardcoded priority; "" disables it).
  std::string component_priority_predicate_ = "battery_ok";
  /// @brief Maximum goals per component eligible for composition.
  int component_goal_limit_ = 4;
  /// @brief Maximum exact cache entries (0 = unbounded).
  int max_exact_cache_entries_ = 0;
  /// @brief Maximum structural cache entries (0 = unbounded).
  int max_structural_cache_entries_ = 0;

  /// @brief Cache store with bounds, metrics and single-flight.
  mutable detail::PlanCache plan_cache_;

  /**
   * @brief Returns the node logger, or a fallback logger before parameters
   * are loaded.
   * @return Logger used by all CachePlanner messages.
   */
  rclcpp::Logger log() const;

  /**
   * @brief Serves a structural cache entry if one exists for @p structural_key.
   * @details Copies the entry out of the cache, adapts object names when the
   * placeholder maps differ, and validates the result when the validator and
   * validate_on_hit settings require it. No cache lock is held while adapting
   * or validating.
   * @param domain The PDDL domain.
   * @param problem The PDDL problem.
   * @param structural_key The pre-computed structural key.
   * @param prepared The role-normalized view of the current problem.
   * @param abstract_keys Whether abstract (fully role-based) keys are in use.
   * @return The served plan, or std::nullopt when there is no entry or the
   * adapted plan is rejected by the validator.
   */
  std::optional<omni_plan::pddl::Plan>
  try_structural_hit(const omni_plan::pddl::Domain &domain,
                     const omni_plan::pddl::Problem &problem,
                     const std::string &structural_key,
                     const detail::PreparedStructure &prepared,
                     bool abstract_keys) const;

  /**
   * @brief Adapts, records and validates one cached entry for a new problem.
   * @param entry The cached entry (originals and placeholders of its problem).
   * @param domain The PDDL domain.
   * @param problem The PDDL problem the plan is being served for.
   * @param prepared The role-normalized view of the current problem.
   * @param abstract_keys Whether abstract keys are in use (forces validation
   * even when no rename is required).
   * @return The adapted plan, or std::nullopt if validation rejects it.
   */
  std::optional<omni_plan::pddl::Plan> serve_cached_entry(
      const CachedPlanData &entry, const omni_plan::pddl::Domain &domain,
      const omni_plan::pddl::Problem &problem,
      const detail::PreparedStructure &prepared, bool abstract_keys) const;

  /**
   * @brief Produces a plan on a cache miss and stores it.
   * @details Tries component composition first (when a validator is loaded),
   * validates the result against the full problem, and falls back to
   * delegate_plan() on failure. Both successful paths store the plan in the
   * exact and structural caches when should_cache_result() accepts it. When
   * @p publish is true the result (or a null plan when the policy refuses to
   * cache it) is published to any single-flight followers; a re-entrant
   * caller that does not own the in-flight entry passes false and leaves the
   * outer flight untouched.
   * @param domain The PDDL domain.
   * @param problem The PDDL problem.
   * @param exact_key Pre-computed exact cache key.
   * @param structural_key Pre-computed structural cache key.
   * @param relevance Relevance sets for the problem.
   * @param prepared Role-normalized view of the problem.
   * @param publish Whether this caller owns the in-flight entry and should
   * publish the result to followers.
   * @return The generated (and possibly cached) plan.
   */
  omni_plan::pddl::Plan
  compute_miss_plan(const omni_plan::pddl::Domain &domain,
                    const omni_plan::pddl::Problem &problem,
                    const std::string &exact_key,
                    const std::string &structural_key,
                    const detail::RelevanceResult &relevance,
                    const detail::PreparedStructure &prepared,
                    bool publish) const;
};

} // namespace omni_plan_cache
#endif // OMNI_PLAN_CACHE__CACHE_PLANNER_HPP_
