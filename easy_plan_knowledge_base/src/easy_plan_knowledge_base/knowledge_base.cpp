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

#include "easy_plan_knowledge_base/knowledge_base.hpp"

using namespace easy_plan_knowledge_base;

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

const std::set<std::string> &KnowledgeBase::get_types() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->types_;
}

// ==================== Object Management ====================
bool KnowledgeBase::add_object(const easy_plan::pddl::Object &object) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  // Check if the object's type exists
  if (this->types_.find(object.get_type()) == this->types_.end()) {
    throw TypeNotFoundException(object.get_type());
  }

  auto result = this->objects_.insert(object);
  return result.second;
}

bool KnowledgeBase::remove_object(const easy_plan::pddl::Object &object) {
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

bool KnowledgeBase::has_object(const easy_plan::pddl::Object &object) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->objects_.find(object) != this->objects_.end();
}

const std::set<easy_plan::pddl::Object> &KnowledgeBase::get_objects() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->objects_;
}

std::set<easy_plan::pddl::Object>
KnowledgeBase::get_objects_by_type(const std::string &type) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  std::set<easy_plan::pddl::Object> result;
  for (const auto &obj : this->get_objects()) {
    if (obj.get_type() == type) {
      result.insert(obj);
    }
  }
  return result;
}

// ==================== Predicate Management ====================
bool KnowledgeBase::add_predicate(const easy_plan::pddl::Predicate &predicate) {
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
    const easy_plan::pddl::Predicate &predicate) {
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
    const easy_plan::pddl::Predicate &predicate) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->predicates_.find(predicate) != this->predicates_.end();
}

const std::set<easy_plan::pddl::Predicate> &
KnowledgeBase::get_predicates() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->predicates_;
}

// ==================== Fact Management ====================
bool KnowledgeBase::add_fact(const easy_plan::pddl::Predicate &fact) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  // Check if the predicate definition exists
  bool predicate_found = false;
  for (const auto &pred : this->predicates_) {
    if (pred.get_name() == fact.get_name()) {
      predicate_found = true;
      break;
    }
  }
  if (!predicate_found) {
    throw PredicateNotFoundException(fact.get_name());
  }

  // Check if all referenced objects exist
  for (const auto &arg : fact.get_args()) {
    bool object_found = false;
    for (const auto &obj : this->objects_) {
      if (obj.get_name() == arg) {
        object_found = true;
        break;
      }
    }
    if (!object_found) {
      throw ObjectNotFoundException(arg);
    }
  }

  // Remove from goals if it exists there (goal achieved)
  this->goals_.erase(fact);

  auto result = this->facts_.insert(fact);
  return result.second;
}

bool KnowledgeBase::remove_fact(const easy_plan::pddl::Predicate &fact) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->facts_.erase(fact) > 0;
}

bool KnowledgeBase::has_fact(const easy_plan::pddl::Predicate &fact) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->facts_.find(fact) != this->facts_.end();
}

const std::set<easy_plan::pddl::Predicate> &KnowledgeBase::get_facts() const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->facts_;
}

std::set<easy_plan::pddl::Predicate>
KnowledgeBase::get_facts_by_name(const std::string &name) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  std::set<easy_plan::pddl::Predicate> result;
  for (const auto &fact : this->get_facts()) {
    if (fact.get_name() == name) {
      result.insert(fact);
    }
  }
  return result;
}

// ==================== Goal Management ====================
bool KnowledgeBase::add_goal(const easy_plan::pddl::Predicate &goal) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);

  // Check if the predicate definition exists
  bool predicate_found = false;
  for (const auto &pred : this->predicates_) {
    if (pred.get_name() == goal.get_name()) {
      predicate_found = true;
      break;
    }
  }
  if (!predicate_found) {
    throw PredicateNotFoundException(goal.get_name());
  }

  // Check if all referenced objects exist
  for (const auto &arg : goal.get_args()) {
    bool object_found = false;
    for (const auto &obj : this->objects_) {
      if (obj.get_name() == arg) {
        object_found = true;
        break;
      }
    }
    if (!object_found) {
      throw ObjectNotFoundException(arg);
    }
  }

  auto result = this->goals_.insert(goal);
  return result.second;
}

bool KnowledgeBase::remove_goal(const easy_plan::pddl::Predicate &goal) {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->goals_.erase(goal) > 0;
}

bool KnowledgeBase::has_goal(const easy_plan::pddl::Predicate &goal) const {
  std::lock_guard<std::recursive_mutex> lock(this->mutex_);
  return this->goals_.find(goal) != this->goals_.end();
}

const std::set<easy_plan::pddl::Predicate> &KnowledgeBase::get_goals() const {
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
