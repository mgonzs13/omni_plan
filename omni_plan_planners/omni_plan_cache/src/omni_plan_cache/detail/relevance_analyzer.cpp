// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#include "omni_plan_cache/detail/relevance_analyzer.hpp"

#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"

namespace omni_plan_cache {
namespace detail {

namespace {

using ActionPtr = std::shared_ptr<omni_plan::pddl::Action>;

std::set<std::string> compute_relevant(
    const std::unordered_map<std::string, std::vector<const ActionPtr *>>
        &effect_index,
    const std::unordered_map<const omni_plan::pddl::Action *,
                             std::vector<std::string>> &conditions,
    const std::set<omni_plan::pddl::Predicate> &goals) {

  std::set<std::string> relevant;
  std::vector<std::string> frontier;
  for (const auto &goal : goals) {
    frontier.push_back(goal.get_name());
  }

  while (!frontier.empty()) {
    std::string current = frontier.back();
    frontier.pop_back();
    if (!relevant.insert(current).second) {
      continue;
    }
    auto it = effect_index.find(current);
    if (it == effect_index.end()) {
      continue;
    }
    for (const auto *action : it->second) {
      auto cond_it = conditions.find(action->get());
      if (cond_it == conditions.end()) {
        continue;
      }
      for (const auto &name : cond_it->second) {
        if (!relevant.count(name)) {
          frontier.push_back(name);
        }
      }
    }
  }
  return relevant;
}

} // namespace

std::set<std::string> RelevanceAnalyzer::relevant_predicates(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem) {
  std::unordered_map<std::string, std::vector<const ActionPtr *>> effect_index;
  std::unordered_map<const omni_plan::pddl::Action *, std::vector<std::string>>
      conditions;

  for (const auto &[name, action] : domain.get_actions()) {
    std::vector<std::string> cond_names;
    for (const auto &cond : action->get_conditions()) {
      cond_names.push_back(cond.get_name());
    }
    conditions[action.get()] = std::move(cond_names);
    for (const auto &effect : action->get_effects()) {
      effect_index[effect.get_name()].push_back(&action);
    }
  }

  return compute_relevant(effect_index, conditions, problem.get_goals());
}

RelevanceResult
RelevanceAnalyzer::analyze(const omni_plan::pddl::Domain &domain,
                           const omni_plan::pddl::Problem &problem,
                           const std::string &robot_type) {

  RelevanceResult result;
  result.relevant_predicates = relevant_predicates(domain, problem);

  for (const auto &pred : domain.get_predicates()) {
    result.full_static_predicates.insert(pred.get_name());
  }
  for (const auto &[name, action] : domain.get_actions()) {
    for (const auto &effect : action->get_effects()) {
      result.full_static_predicates.erase(effect.get_name());
    }
  }
  for (const auto &pred : result.relevant_predicates) {
    if (result.full_static_predicates.count(pred)) {
      result.static_predicates.insert(pred);
    }
  }

  for (const auto &goal : problem.get_goals()) {
    for (const auto &arg : goal.get_args()) {
      result.relevant_objects.insert(arg);
    }
  }
  for (const auto &obj : problem.get_objects()) {
    if (obj.get_type() == robot_type) {
      result.relevant_objects.insert(obj.get_name());
    }
  }
  for (const auto &fact : problem.get_facts()) {
    if (!result.static_predicates.count(fact.get_name())) {
      continue;
    }
    for (const auto &arg : fact.get_args()) {
      result.relevant_objects.insert(arg);
    }
  }

  for (const auto &fact : problem.get_facts()) {
    if (!result.relevant_predicates.count(fact.get_name())) {
      continue;
    }
    if (result.static_predicates.count(fact.get_name())) {
      result.relevant_facts.insert(fact);
      continue;
    }
    bool keep = false;
    for (const auto &arg : fact.get_args()) {
      if (result.relevant_objects.count(arg)) {
        keep = true;
        break;
      }
    }
    if (keep) {
      result.relevant_facts.insert(fact);
    }
  }

  return result;
}

} // namespace detail
} // namespace omni_plan_cache
