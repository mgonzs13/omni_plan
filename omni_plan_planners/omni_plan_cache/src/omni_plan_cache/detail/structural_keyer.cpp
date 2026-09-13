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

#include "omni_plan_cache/detail/structural_keyer.hpp"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/timing_predicate.hpp"
#include "omni_plan_cache/detail/sha256.hpp"

namespace omni_plan_cache {
namespace detail {

namespace {

namespace pddl = omni_plan::pddl;

constexpr const char *kSeparator = "|";

struct Contribution {
  std::string base;
  std::vector<std::string> co_args;
};

using Contributions =
    std::unordered_map<std::string, std::vector<Contribution>>;

Contributions collect_contributions(const std::set<pddl::Predicate> &facts,
                                    const std::set<pddl::Predicate> &goals) {
  Contributions out;
  auto add = [&out](const std::set<pddl::Predicate> &preds, bool is_goal) {
    for (const auto &pred : preds) {
      const auto args = pred.get_args();
      for (size_t i = 0; i < args.size(); ++i) {
        Contribution contribution;
        contribution.base = pred.get_name() + "_" + std::to_string(i) + "_" +
                            (is_goal ? "1" : "0");
        contribution.co_args.reserve(args.size() - 1);
        for (size_t j = 0; j < args.size(); ++j) {
          if (j != i) {
            contribution.co_args.push_back(args[j]);
          }
        }
        out[args[i]].push_back(std::move(contribution));
      }
    }
  };
  add(facts, false);
  add(goals, true);
  return out;
}

std::unordered_map<std::string, std::string> build_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const Contributions &contributions,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {

  std::unordered_map<std::string, std::string> result;
  for (const auto &group : objects_by_type) {
    for (const auto &name : group.names) {
      auto it = contributions.find(name);
      if (it == contributions.end()) {
        result[name].clear();
        continue;
      }
      std::vector<std::string> entries;
      entries.reserve(it->second.size());
      for (const auto &contribution : it->second) {
        std::string entry = contribution.base;
        if (name_to_alias != nullptr && !abstract_keys) {
          for (const auto &co_arg : contribution.co_args) {
            auto alias_it = name_to_alias->find(co_arg);
            entry += "_" + (alias_it != name_to_alias->end() ? alias_it->second
                                                             : co_arg);
          }
        }
        entries.push_back(std::move(entry));
      }
      std::sort(entries.begin(), entries.end());
      std::string key;
      for (const auto &entry : entries) {
        key += entry;
        key += "|";
      }
      result[name] = std::move(key);
    }
  }
  return result;
}

void update_predicate(Sha256 &hash, const pddl::Predicate &pred) {
  hash.update("p:" + pred.get_name());
  for (const auto &arg : pred.get_args()) {
    hash.update("?" + arg);
  }
  hash.update(pred.is_negated() ? "!|" : "|");
}

void update_timing_predicate(Sha256 &hash, const pddl::TimingPredicate &pred) {
  hash.update("t:" + std::to_string(static_cast<int>(pred.get_type())) + ":");
  update_predicate(hash, pred);
}

void update_action(Sha256 &hash, const pddl::Action &action) {
  hash.update("a:" + action.get_name() + ":" +
              std::to_string(action.get_duration()) + "|");
  for (const auto &param : action.get_parameters()) {
    hash.update("param:" + param.get_name() + ":" + param.get_type() + "|");
  }
  for (const auto &condition : action.get_conditions()) {
    update_timing_predicate(hash, condition);
  }
  for (const auto &effect : action.get_effects()) {
    update_timing_predicate(hash, effect);
  }
}

void update_domain(Sha256 &hash, const pddl::Domain &domain) {
  hash.update("D|");
  for (const auto &requirement : domain.get_requirements()) {
    hash.update("req:" + requirement + "|");
  }
  for (const auto &type : domain.get_types()) {
    hash.update("type:" + type + "|");
  }
  for (const auto &pred : domain.get_predicates()) {
    update_predicate(hash, pred);
  }
  for (const auto &[name, action] : domain.get_actions()) {
    update_action(hash, *action);
  }
}

void update_problem(Sha256 &hash, const pddl::Problem &problem) {
  hash.update("P|");
  for (const auto &obj : problem.get_objects()) {
    hash.update("obj:" + obj.get_name() + ":" + obj.get_type() + "|");
  }
  for (const auto &fact : problem.get_facts()) {
    update_predicate(hash, fact);
  }
  for (const auto &goal : problem.get_goals()) {
    update_predicate(hash, goal);
  }
}

void update_abstraction(
    Sha256 &hash, const pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<pddl::Predicate> *filtered_facts) {

  std::unordered_map<std::string, std::string> name_to_type;
  for (const auto &group : objects_by_type) {
    for (const auto &name : group.names) {
      name_to_type[name] = group.type;
    }
  }

  auto type_signature = [&name_to_type](const pddl::Predicate &pred) {
    std::string signature = pred.get_name();
    for (const auto &arg : pred.get_args()) {
      auto it = name_to_type.find(arg);
      signature += ":" + (it != name_to_type.end() ? it->second : arg);
    }
    return signature;
  };

  for (const auto &group : objects_by_type) {
    hash.update(group.type + ":" + std::to_string(group.names.size()) +
                kSeparator);
  }

  std::vector<std::string> entries;
  const auto &facts =
      filtered_facts != nullptr ? *filtered_facts : problem.get_facts();
  entries.reserve(facts.size());
  for (const auto &fact : facts) {
    entries.push_back(type_signature(fact));
  }
  std::sort(entries.begin(), entries.end());
  for (const auto &entry : entries) {
    hash.update("I:" + entry + kSeparator);
  }

  entries.clear();
  entries.reserve(problem.get_goals().size());
  for (const auto &goal : problem.get_goals()) {
    entries.push_back(type_signature(goal));
  }
  std::sort(entries.begin(), entries.end());
  for (const auto &entry : entries) {
    hash.update("G:" + entry + kSeparator);
  }

  std::vector<std::string> roles;
  roles.reserve(role_keys.size());
  for (const auto &[name, key] : role_keys) {
    roles.push_back(key);
  }
  std::sort(roles.begin(), roles.end());
  for (const auto &role : roles) {
    hash.update("R:" + role + kSeparator);
  }
}

} // namespace

std::vector<ObjectsByType> StructuralKeyer::group_objects_by_type(
    const std::set<omni_plan::pddl::Object> &objects) {
  std::unordered_map<std::string, std::vector<std::string>> type_to_names;
  std::vector<std::string> type_order;

  for (const auto &obj : objects) {
    const auto &type = obj.get_type();
    if (type_to_names.find(type) == type_to_names.end()) {
      type_order.push_back(type);
    }
    type_to_names[type].push_back(obj.get_name());
  }

  std::sort(type_order.begin(), type_order.end());

  std::vector<ObjectsByType> result;
  result.reserve(type_order.size());
  for (const auto &type : type_order) {
    result.push_back({type, type_to_names[type]});
  }
  return result;
}

std::unordered_map<std::string, std::string> StructuralKeyer::compute_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const std::set<omni_plan::pddl::Predicate> &facts,
    const std::set<omni_plan::pddl::Predicate> &goals,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {
  return build_role_keys(objects_by_type, collect_contributions(facts, goals),
                         name_to_alias, abstract_keys);
}

PreparedStructure
StructuralKeyer::prepare(const std::set<omni_plan::pddl::Object> &objects,
                         const std::set<omni_plan::pddl::Predicate> &facts,
                         const std::set<omni_plan::pddl::Predicate> &goals,
                         bool abstract_keys) {

  const auto objects_by_type = group_objects_by_type(objects);
  const auto contributions = collect_contributions(facts, goals);
  const auto abstract_keys_map =
      build_role_keys(objects_by_type, contributions, nullptr, true);

  PreparedStructure out;
  for (const auto &group : objects_by_type) {
    ObjectsByType filtered;
    filtered.type = group.type;
    for (const auto &name : group.names) {
      auto it = abstract_keys_map.find(name);
      if (it != abstract_keys_map.end() && !it->second.empty()) {
        filtered.names.push_back(name);
      }
    }
    if (!filtered.names.empty()) {
      out.objects_by_type.push_back(std::move(filtered));
    }
  }

  for (auto &group : out.objects_by_type) {
    std::sort(group.names.begin(), group.names.end(),
              [&abstract_keys_map](const std::string &a, const std::string &b) {
                const std::string &key_a = abstract_keys_map.at(a);
                const std::string &key_b = abstract_keys_map.at(b);
                if (key_a != key_b) {
                  return key_a < key_b;
                }
                return a < b;
              });
  }

  for (const auto &group : out.objects_by_type) {
    for (size_t i = 0; i < group.names.size(); ++i) {
      out.name_to_alias[group.names[i]] = group.type + "_" + std::to_string(i);
      out.placeholder_to_original["__obj_" + group.type + "_" +
                                  std::to_string(i) + "__"] = group.names[i];
    }
  }

  out.role_keys = build_role_keys(out.objects_by_type, contributions,
                                  &out.name_to_alias, abstract_keys);
  return out;
}

std::string
StructuralKeyer::exact_key(const omni_plan::pddl::Domain &domain,
                           const omni_plan::pddl::Problem &problem) {
  Sha256 hash;
  update_domain(hash, domain);
  hash.update("|EXACT|");
  update_problem(hash, problem);
  return hash.final_hex();
}

std::string StructuralKeyer::compute_key(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<omni_plan::pddl::Predicate> *filtered_facts) {
  Sha256 hash;
  update_domain(hash, domain);
  hash.update("|ABSTRACT|");
  update_abstraction(hash, problem, objects_by_type, role_keys, filtered_facts);
  return hash.final_hex();
}

std::string StructuralKeyer::compute_key(
    const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<omni_plan::pddl::Predicate> *filtered_facts) {
  Sha256 hash;
  hash.update(domain_pddl);
  hash.update("|ABSTRACT|");
  update_abstraction(hash, problem, objects_by_type, role_keys, filtered_facts);
  return hash.final_hex();
}

} // namespace detail
} // namespace omni_plan_cache
