// Copyright (C) 2025 Miguel Ángel González Santamarta
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

/**
 * @class InvalidPredicateException
 * @brief Exception thrown when a ground predicate does not match its
 * predicate definition (arity or argument types).
 */
class InvalidPredicateException : public KnowledgeBaseException {
public:
  explicit InvalidPredicateException(const std::string &message)
      : KnowledgeBaseException(message) {}
};

} // namespace omni_plan_knowledge_base

#endif // OMNI_PLAN_KNOWLEDGE_BASE__KNOWLEDGE_BASE_EXCEPTIONS_HPP_
