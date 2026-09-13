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

#include <algorithm>

#include "omni_plan_knowledge_base/knowledge_base.hpp"

using namespace omni_plan_knowledge_base;

namespace {

const omni_plan::pddl::Predicate *find_predicate_definition(
    const std::set<omni_plan::pddl::Predicate> &predicates,
    const omni_plan::pddl::Predicate &ground) {
  const omni_plan::pddl::Predicate *fallback = nullptr;

  for (const auto &pred : predicates) {
    if (pred.get_name() != ground.get_name()) {
      continue;
    }
    if (pred.get_args().size() == ground.get_args().size()) {
      return &pred;
    }
    fallback = &pred;
  }

  return fallback;
}

void validate_ground_predicate(
    const std::set<omni_plan::pddl::Predicate> &predicates,
    const std::set<omni_plan::pddl::Object> &objects,
    const omni_plan::pddl::Predicate &ground) {

  const auto *definition = find_predicate_definition(predicates, ground);

  if (definition == nullptr) {
    throw PredicateNotFoundException(ground.get_name());
  }

  if (definition->get_args().size() != ground.get_args().size()) {
    throw InvalidPredicateException(
        "Predicate '" + ground.get_name() + "' expects " +
        std::to_string(definition->get_args().size()) + " arguments but got " +
        std::to_string(ground.get_args().size()));
  }

  const auto &declared_types = definition->get_args();
  const auto &args = ground.get_args();

  for (size_t i = 0; i < args.size(); ++i) {
    const omni_plan::pddl::Object *object = nullptr;

    for (const auto &obj : objects) {
      if (obj.get_name() == args[i]) {
        object = &obj;
        break;
      }
    }

    if (object == nullptr) {
      throw ObjectNotFoundException(args[i]);
    }

    const std::string &declared_type = declared_types[i];
    if (!declared_type.empty() && declared_type.front() != '?' &&
        declared_type != object->get_type()) {
      throw InvalidPredicateException(
          "Argument '" + args[i] + "' of predicate '" + ground.get_name() +
          "' has type '" + object->get_type() + "' but '" + declared_type +
          "' was expected");
    }
  }
}

} // namespace

KnowledgeBase::KnowledgeBase() {}

// ==================== Type Management ====================
bool KnowledgeBase::add_type(const std::string &type) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  auto result = this->types_.insert(type);
  return result.second;
}

bool KnowledgeBase::remove_type(const std::string &type) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  // First, remove all objects of this type (and their associated facts/goals)
  auto objects = this->get_objects_by_type(type);
  for (const auto &obj : objects) {
    this->remove_object(obj);
  }

  // Second, remove all predicates with this type in their arguments
  auto predicates = this->get_predicates();
  for (const auto &pred : predicates) {
    const auto &arg_types = pred.get_args();
    if (std::find(arg_types.begin(), arg_types.end(), type) !=
        arg_types.end()) {
      this->remove_predicate(pred);
    }
  }

  // Finally, remove the type itself
  return this->types_.erase(type) > 0;
}

bool KnowledgeBase::has_type(const std::string &type) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->types_.find(type) != this->types_.end();
}

std::set<std::string> KnowledgeBase::get_types() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->types_;
}

// ==================== Object Management ====================
bool KnowledgeBase::add_object(const omni_plan::pddl::Object &object) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  // Check if the object's type exists
  if (this->types_.find(object.get_type()) == this->types_.end()) {
    throw TypeNotFoundException(object.get_type());
  }

  auto result = this->objects_.insert(object);
  return result.second;
}

bool KnowledgeBase::remove_object(const omni_plan::pddl::Object &object) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  const std::string obj_name = object.get_name();

  // Remove facts referencing this object
  auto facts = this->get_facts();
  for (const auto &fact : facts) {
    const auto &args = fact.get_args();
    if (std::find(args.begin(), args.end(), obj_name) != args.end()) {
      this->remove_fact(fact);
    }
  }

  // Remove goals referencing this object
  auto goals = this->get_goals();
  for (const auto &goal : goals) {
    const auto &args = goal.get_args();
    if (std::find(args.begin(), args.end(), obj_name) != args.end()) {
      this->remove_goal(goal);
    }
  }

  // Remove the object itself
  return this->objects_.erase(object) > 0;
}

bool KnowledgeBase::has_object(const omni_plan::pddl::Object &object) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->objects_.find(object) != this->objects_.end();
}

std::set<omni_plan::pddl::Object> KnowledgeBase::get_objects() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->objects_;
}

std::set<omni_plan::pddl::Object>
KnowledgeBase::get_objects_by_type(const std::string &type) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  std::set<omni_plan::pddl::Object> result;
  for (const auto &obj : this->get_objects()) {
    if (obj.get_type() == type) {
      result.insert(obj);
    }
  }
  return result;
}

// ==================== Predicate Management ====================
bool KnowledgeBase::add_predicate(const omni_plan::pddl::Predicate &predicate) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  // Check if all argument types exist
  for (const auto &arg : predicate.get_args()) {
    if (this->types_.find(arg) == this->types_.end()) {
      throw TypeNotFoundException(arg);
    }
  }

  auto result = this->predicates_.insert(predicate);
  return result.second;
}

bool KnowledgeBase::remove_predicate(
    const omni_plan::pddl::Predicate &predicate) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  const std::string pred_name = predicate.get_name();

  // Remove facts with this predicate name
  auto facts = this->get_facts();
  for (const auto &fact : facts) {
    if (fact.get_name() == pred_name) {
      this->remove_fact(fact);
    }
  }

  // Remove goals with this predicate name
  auto goals = this->get_goals();
  for (const auto &goal : goals) {
    if (goal.get_name() == pred_name) {
      this->remove_goal(goal);
    }
  }

  // Remove the predicate itself
  return this->predicates_.erase(predicate) > 0;
}

bool KnowledgeBase::has_predicate(
    const omni_plan::pddl::Predicate &predicate) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->predicates_.find(predicate) != this->predicates_.end();
}

std::set<omni_plan::pddl::Predicate> KnowledgeBase::get_predicates() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->predicates_;
}

// ==================== Fact Management ====================
bool KnowledgeBase::add_fact(const omni_plan::pddl::Predicate &fact) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  validate_ground_predicate(this->predicates_, this->objects_, fact);

  auto result = this->facts_.insert(fact);

  // Only erase the matching goal when the fact was actually inserted
  if (result.second) {
    this->goals_.erase(fact);
  }

  return result.second;
}

bool KnowledgeBase::remove_fact(const omni_plan::pddl::Predicate &fact) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->facts_.erase(fact) > 0;
}

bool KnowledgeBase::has_fact(const omni_plan::pddl::Predicate &fact) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->facts_.find(fact) != this->facts_.end();
}

std::set<omni_plan::pddl::Predicate> KnowledgeBase::get_facts() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->facts_;
}

std::set<omni_plan::pddl::Predicate>
KnowledgeBase::get_facts_by_name(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  std::set<omni_plan::pddl::Predicate> result;
  for (const auto &fact : this->get_facts()) {
    if (fact.get_name() == name) {
      result.insert(fact);
    }
  }
  return result;
}

// ==================== Goal Management ====================
bool KnowledgeBase::add_goal(const omni_plan::pddl::Predicate &goal) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  validate_ground_predicate(this->predicates_, this->objects_, goal);

  // Reject goals that are already satisfied by a fact
  if (this->facts_.find(goal) != this->facts_.end()) {
    return false;
  }

  auto result = this->goals_.insert(goal);
  return result.second;
}

bool KnowledgeBase::remove_goal(const omni_plan::pddl::Predicate &goal) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->goals_.erase(goal) > 0;
}

bool KnowledgeBase::has_goal(const omni_plan::pddl::Predicate &goal) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->goals_.find(goal) != this->goals_.end();
}

std::set<omni_plan::pddl::Predicate> KnowledgeBase::get_goals() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->goals_;
}

bool KnowledgeBase::has_goals() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return !this->goals_.empty();
}

// ==================== Utility Methods ====================
void KnowledgeBase::clear() {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  this->types_.clear();
  this->objects_.clear();
  this->predicates_.clear();
  this->facts_.clear();
  this->goals_.clear();
}
