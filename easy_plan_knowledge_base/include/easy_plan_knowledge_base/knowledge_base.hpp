// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#ifndef EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_HPP_
#define EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_HPP_

#include <mutex>
#include <set>
#include <string>

#include "easy_plan/pddl/object.hpp"
#include "easy_plan/pddl/predicate.hpp"
#include "easy_plan_knowledge_base/knowledge_base_exceptions.hpp"

namespace easy_plan_knowledge_base {

/**
 * @class KnowledgeBase
 * @brief A thread-safe storage for PDDL knowledge elements.
 * @details This class provides storage and management for PDDL types, objects,
 * predicates, facts, and goals. All operations are thread-safe through mutex
 * protection.
 */
class KnowledgeBase {
public:
  /**
   * @brief Default constructor.
   */
  KnowledgeBase();

  /**
   * @brief Default destructor.
   */
  ~KnowledgeBase() = default;

  // ==================== Type Management ====================
  /**
   * @brief Adds a type to the knowledge base.
   * @param type The type name to add.
   * @return True if the type was added, false if it already exists.
   */
  bool add_type(const std::string &type);

  /**
   * @brief Removes a type from the knowledge base.
   * @details Cascades removal: removes all objects of this type, all predicates
   * with this type in their arguments, and all facts/goals that reference those
   * objects or predicates.
   * @param type The type name to remove.
   * @return True if the type was removed, false if it didn't exist.
   */
  bool remove_type(const std::string &type);

  /**
   * @brief Checks if a type exists in the knowledge base.
   * @param type The type name to check.
   * @return True if the type exists, false otherwise.
   */
  bool has_type(const std::string &type) const;

  /**
   * @brief Gets all types in the knowledge base.
   * @return A set of all type names.
   */
  const std::set<std::string> &get_types() const;

  // ==================== Object Management ====================
  /**
   * @brief Adds an object to the knowledge base.
   * @param object The object to add.
   * @return True if the object was added, false if it already exists.
   * @throws TypeNotFoundException if the object's type does not exist.
   */
  bool add_object(const easy_plan::pddl::Object &object);

  /**
   * @brief Removes an object from the knowledge base.
   * @details Also removes all facts and goals that reference this object.
   * @param object The object to remove.
   * @return True if the object was removed, false if it didn't exist.
   */
  bool remove_object(const easy_plan::pddl::Object &object);

  /**
   * @brief Checks if an object exists in the knowledge base.
   * @param object The object to check.
   * @return True if the object exists, false otherwise.
   */
  bool has_object(const easy_plan::pddl::Object &object) const;

  /**
   * @brief Gets all objects in the knowledge base.
   * @return A set of all objects.
   */
  const std::set<easy_plan::pddl::Object> &get_objects() const;

  /**
   * @brief Gets objects of a specific type.
   * @param type The type to filter by.
   * @return A set of objects of the specified type.
   */
  std::set<easy_plan::pddl::Object>
  get_objects_by_type(const std::string &type) const;

  // ==================== Predicate Management ====================
  /**
   * @brief Adds a predicate definition to the knowledge base.
   * @param predicate The predicate to add.
   * @return True if the predicate was added, false if it already exists.
   * @throws TypeNotFoundException if any of the predicate's argument types do
   * not exist.
   */
  bool add_predicate(const easy_plan::pddl::Predicate &predicate);

  /**
   * @brief Removes a predicate definition from the knowledge base.
   * @details Also removes all facts and goals that use this predicate.
   * @param predicate The predicate to remove.
   * @return True if the predicate was removed, false if it didn't exist.
   */
  bool remove_predicate(const easy_plan::pddl::Predicate &predicate);

  /**
   * @brief Checks if a predicate definition exists in the knowledge base.
   * @param predicate The predicate to check.
   * @return True if the predicate exists, false otherwise.
   */
  bool has_predicate(const easy_plan::pddl::Predicate &predicate) const;

  /**
   * @brief Gets all predicate definitions in the knowledge base.
   * @return A set of all predicates.
   */
  const std::set<easy_plan::pddl::Predicate> &get_predicates() const;

  // ==================== Fact Management ====================
  /**
   * @brief Adds a fact to the knowledge base.
   * @param fact The fact to add.
   * @return True if the fact was added, false if it already exists.
   * @throws PredicateNotFoundException if the predicate does not exist.
   * @throws ObjectNotFoundException if any referenced object does not exist.
   */
  bool add_fact(const easy_plan::pddl::Predicate &fact);

  /**
   * @brief Removes a fact from the knowledge base.
   * @param fact The fact to remove.
   * @return True if the fact was removed, false if it didn't exist.
   */
  bool remove_fact(const easy_plan::pddl::Predicate &fact);

  /**
   * @brief Checks if a fact exists in the knowledge base.
   * @param fact The fact to check.
   * @return True if the fact exists, false otherwise.
   */
  bool has_fact(const easy_plan::pddl::Predicate &fact) const;

  /**
   * @brief Gets all facts in the knowledge base.
   * @return A set of all facts.
   */
  const std::set<easy_plan::pddl::Predicate> &get_facts() const;

  /**
   * @brief Gets facts with a specific predicate name.
   * @param name The predicate name to filter by.
   * @return A set of facts with the specified predicate name.
   */
  std::set<easy_plan::pddl::Predicate>
  get_facts_by_name(const std::string &name) const;

  // ==================== Goal Management ====================
  /**
   * @brief Adds a goal to the knowledge base.
   * @param goal The goal to add.
   * @return True if the goal was added, false if it already exists.
   * @throws PredicateNotFoundException if the predicate does not exist.
   * @throws ObjectNotFoundException if any referenced object does not exist.
   */
  bool add_goal(const easy_plan::pddl::Predicate &goal);

  /**
   * @brief Removes a goal from the knowledge base.
   * @param goal The goal to remove.
   * @return True if the goal was removed, false if it didn't exist.
   */
  bool remove_goal(const easy_plan::pddl::Predicate &goal);

  /**
   * @brief Checks if a goal exists in the knowledge base.
   * @param goal The goal to check.
   * @return True if the goal exists, false otherwise.
   */
  bool has_goal(const easy_plan::pddl::Predicate &goal) const;

  /**
   * @brief Gets all goals in the knowledge base.
   * @return A set of all goals.
   */
  const std::set<easy_plan::pddl::Predicate> &get_goals() const;

  /**
   * @brief Checks if there are any goals in the knowledge base.
   * @return True if there are goals, false otherwise.
   */
  bool has_goals() const;

  // ==================== Utility Methods ====================
  /**
   * @brief Clears all data from the knowledge base.
   */
  void clear();

private:
  /// Mutex for thread-safe access.
  mutable std::recursive_mutex mutex_;

  /// Set of types.
  std::set<std::string> types_;

  /// Set of objects.
  std::set<easy_plan::pddl::Object> objects_;

  /// Set of predicate definitions.
  std::set<easy_plan::pddl::Predicate> predicates_;

  /// Set of facts (ground predicates that are true).
  std::set<easy_plan::pddl::Predicate> facts_;

  /// Set of goals (predicates to be achieved).
  std::set<easy_plan::pddl::Predicate> goals_;
};

} // namespace easy_plan_knowledge_base

#endif // EASY_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_HPP_
