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

#ifndef OMNI_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_EXCEPTIONS_HPP_
#define OMNI_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_EXCEPTIONS_HPP_

#include <stdexcept>
#include <string>

namespace omni_plan_knowledge_base {

/**
 * @class KnowledgeBaseException
 * @brief Base exception class for knowledge base errors.
 */
class KnowledgeBaseException : public std::runtime_error {
public:
  explicit KnowledgeBaseException(const std::string &message)
      : std::runtime_error(message) {}
};

/**
 * @class TypeNotFoundException
 * @brief Exception thrown when a referenced type does not exist.
 */
class TypeNotFoundException : public KnowledgeBaseException {
public:
  explicit TypeNotFoundException(const std::string &type)
      : KnowledgeBaseException("Type '" + type + "' does not exist") {}
};

/**
 * @class ObjectNotFoundException
 * @brief Exception thrown when a referenced object does not exist.
 */
class ObjectNotFoundException : public KnowledgeBaseException {
public:
  explicit ObjectNotFoundException(const std::string &object)
      : KnowledgeBaseException("Object '" + object + "' does not exist") {}
};

/**
 * @class PredicateNotFoundException
 * @brief Exception thrown when a referenced predicate does not exist.
 */
class PredicateNotFoundException : public KnowledgeBaseException {
public:
  explicit PredicateNotFoundException(const std::string &predicate)
      : KnowledgeBaseException("Predicate '" + predicate + "' does not exist") {
  }
};

} // namespace omni_plan_knowledge_base

#endif // OMNI_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_EXCEPTIONS_HPP_
