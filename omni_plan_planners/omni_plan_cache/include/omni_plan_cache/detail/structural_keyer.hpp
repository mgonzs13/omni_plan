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
 * @file structural_keyer.hpp
 * @brief Structural normalization of PDDL problems: object grouping, role
 * keys and canonical cache-key digests.
 */

#ifndef OMNI_PLAN_CACHE__DETAIL__STRUCTURAL_KEYER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__STRUCTURAL_KEYER_HPP_

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/types.hpp"

namespace omni_plan_cache {
namespace detail {

/**
 * @struct PreparedStructure
 * @brief Result of preparing the role-normalized structural view of a problem.
 * @details Groups the filtered/sorted objects, their deterministic aliases,
 * their role keys and the placeholder map used to adapt cached plans. The
 * three maps are keyed by concrete object name (except
 * placeholder_to_original, which is keyed by placeholder).
 */
struct PreparedStructure {
  /// @brief Filtered, role-sorted object view used for cache keys and
  /// placeholder indices. Objects that never appear in the relevant
  /// predicates are omitted.
  std::vector<ObjectsByType> objects_by_type;
  /// @brief Maps each filtered object name to its deterministic alias of the
  /// form "<type>_<index>" (index within its role-sorted type group).
  std::unordered_map<std::string, std::string> name_to_alias;
  /// @brief Role key for each filtered object name. Abstract keys ignore
  /// concrete co-occurring objects; concrete keys embed their aliases.
  std::unordered_map<std::string, std::string> role_keys;
  /// @brief Maps "__obj_<type>_<i>__" placeholders to the concrete object
  /// names they stand for, enabling plan adaptation on structural hits.
  std::unordered_map<std::string, std::string> placeholder_to_original;
};

/**
 * @class StructuralKeyer
 * @brief Pure functions for object grouping, role keys and structural
 * digests.
 * @details All methods are stateless. A "role key" summarizes how an object
 * participates in the relevant facts and goals (predicate name, argument
 * position and fact-vs-goal role). Role keys make two problems with different
 * object names but the same structure produce the same structural cache key.
 */
class StructuralKeyer {
public:
  /**
   * @brief Groups objects by their PDDL type.
   * @param objects Objects to group (the problem's object set).
   * @return One ObjectsByType per unique type, ordered by type name; names
   * within a group keep the input set order (sorted by name).
   */
  static std::vector<ObjectsByType>
  group_objects_by_type(const std::set<omni_plan::pddl::Object> &objects);

  /**
   * @brief Computes a role-key signature for each object.
   * @details For every predicate occurrence, each argument position
   * contributes "[N:]<predicate>_<arg_index>_<goal?1:0>", where the "N:"
   * prefix marks a negated predicate so that p(x) and not-p(x) get different
   * keys. When @p name_to_alias is provided and @p abstract_keys is false,
   * the aliases (or concrete names) of the co-occurring arguments are
   * appended so that attribute distributions are distinguished.
   * Contributions per object are sorted and concatenated with '|'.
   * @param objects_by_type Objects to compute keys for, grouped by type.
   * @param facts Relevant initial-state predicates.
   * @param goals Goal predicates.
   * @param name_to_alias Optional object-name to alias map used to build
   * concrete keys. Pass nullptr (default) for purely structural keys.
   * @param abstract_keys When true, ignore @p name_to_alias and emit keys
   * that only encode predicate usage (default false).
   * @return Map from object name to role-key string. Objects with no
   * occurrences get an empty key.
   */
  static std::unordered_map<std::string, std::string> compute_role_keys(
      const std::vector<ObjectsByType> &objects_by_type,
      const std::set<omni_plan::pddl::Predicate> &facts,
      const std::set<omni_plan::pddl::Predicate> &goals,
      const std::unordered_map<std::string, std::string> *name_to_alias =
          nullptr,
      bool abstract_keys = false);

  /**
   * @brief Builds the complete role-normalized view of a problem.
   * @details Computes abstract role keys, drops objects with empty keys,
   * sorts each type group by role key then name, assigns deterministic
   * aliases/placeholders, and finally computes the concrete role keys (or
   * abstract ones when @p abstract_keys is true). This is the single input
   * required by compute_key().
   * @param objects All objects of the problem.
   * @param facts Relevant initial-state predicates.
   * @param goals Goal predicates.
   * @param abstract_keys When true, role keys ignore concrete co-occurrence
   * (safe with validated hits); when false, concrete aliases are embedded.
   * @return The prepared structure (filtered groups, aliases, keys and
   * placeholders).
   */
  static PreparedStructure
  prepare(const std::set<omni_plan::pddl::Object> &objects,
          const std::set<omni_plan::pddl::Predicate> &facts,
          const std::set<omni_plan::pddl::Predicate> &goals,
          bool abstract_keys);

  /**
   * @brief Computes the exact cache key for a domain/problem pair.
   * @details Streams every field of the domain (requirements, types,
   * predicates, action names/durations/parameters/conditions/effects) and of
   * the problem (objects, facts, goals, including negations) into SHA-256.
   * Two problems share a key iff they serialize to the same canonical
   * representation.
   * @param domain The PDDL domain.
   * @param problem The PDDL problem.
   * @return Lowercase hexadecimal SHA-256 digest.
   */
  static std::string exact_key(const omni_plan::pddl::Domain &domain,
                               const omni_plan::pddl::Problem &problem);

  /**
   * @brief Computes the structural cache key from a Domain object.
   * @details Delegates to the string overload using domain.to_pddl(), so the
   * runtime key and CachePlanner::compute_structural_key(domain_pddl, ...)
   * produce the same digest for the same serialized domain. The abstraction
   * built by update_abstraction() covers object-type counts, type-abstracted
   * facts, type-abstracted goals and the sorted multiset of role keys; both
   * fact and goal signatures include predicate polarity.
   * @param domain The PDDL domain.
   * @param problem The PDDL problem.
   * @param objects_by_type Filtered, role-sorted objects.
   * @param role_keys Role key per object name (concrete or abstract).
   * @param filtered_facts Relevant facts to abstract; when nullptr, all
   * problem facts are used.
   * @return Lowercase hexadecimal SHA-256 digest.
   */
  static std::string
  compute_key(const omni_plan::pddl::Domain &domain,
              const omni_plan::pddl::Problem &problem,
              const std::vector<ObjectsByType> &objects_by_type,
              const std::unordered_map<std::string, std::string> &role_keys,
              const std::set<omni_plan::pddl::Predicate> *filtered_facts);

  /**
   * @brief Computes the structural cache key from a pre-serialized domain.
   * @details This is the canonical implementation: it hashes the given
   * @p domain_pddl text and the abstraction built by update_abstraction().
   * The Domain overload delegates here with domain.to_pddl(), so both
   * overloads agree for a serialized domain. The test suite asserts this
   * equivalence.
   * @param domain_pddl The domain PDDL text.
   * @param problem The PDDL problem.
   * @param objects_by_type Filtered, role-sorted objects.
   * @param role_keys Role key per object name (concrete or abstract).
   * @param filtered_facts Relevant facts to abstract; when nullptr, all
   * problem facts are used.
   * @return Lowercase hexadecimal SHA-256 digest.
   */
  static std::string
  compute_key(const std::string &domain_pddl,
              const omni_plan::pddl::Problem &problem,
              const std::vector<ObjectsByType> &objects_by_type,
              const std::unordered_map<std::string, std::string> &role_keys,
              const std::set<omni_plan::pddl::Predicate> *filtered_facts);
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__STRUCTURAL_KEYER_HPP_
