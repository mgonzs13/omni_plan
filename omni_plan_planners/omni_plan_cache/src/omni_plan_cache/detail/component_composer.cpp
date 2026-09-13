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

#include "omni_plan_cache/detail/component_composer.hpp"

#include <algorithm>
#include <future>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <utility>

namespace omni_plan_cache {
namespace detail {

namespace pddl = omni_plan::pddl;

ComponentComposer::ComponentComposer(Options options, Solver solver,
                                     Logger info_log, Logger warn_log)
    : options_(std::move(options)), solver_(std::move(solver)),
      info_(std::move(info_log)), warn_(std::move(warn_log)) {}

namespace {

std::string component_key(const std::vector<pddl::Predicate> &component) {
  std::string key;
  for (const auto &goal : component) {
    key += goal.get_name();
    for (const auto &arg : goal.get_args()) {
      key += "_" + arg;
    }
    key += ";";
  }
  return key;
}

std::string find_root(std::unordered_map<std::string, std::string> &parent,
                      const std::string &name) {
  auto it = parent.find(name);
  if (it == parent.end()) {
    parent[name] = name;
    return name;
  }
  if (it->second == name) {
    return name;
  }
  it->second = find_root(parent, it->second);
  return it->second;
}

void unite(std::unordered_map<std::string, std::string> &parent,
           const std::string &a, const std::string &b) {
  const std::string root_a = find_root(parent, a);
  const std::string root_b = find_root(parent, b);
  if (root_a != root_b) {
    parent[root_b] = root_a;
  }
}

bool fact_inside(const pddl::Predicate &fact,
                 const std::set<std::string> &objects) {
  for (const auto &arg : fact.get_args()) {
    if (!objects.count(arg)) {
      return false;
    }
  }
  return true;
}

} // namespace

bool ComponentComposer::can_solve_in_parallel(
    const std::vector<std::set<std::string>> &mutable_objects) {
  for (size_t i = 0; i < mutable_objects.size(); ++i) {
    for (size_t j = i + 1; j < mutable_objects.size(); ++j) {
      for (const auto &object : mutable_objects[i]) {
        if (mutable_objects[j].count(object)) {
          return false;
        }
      }
    }
  }
  return true;
}

void ComponentComposer::apply_action_effects(
    std::set<pddl::Predicate> &facts,
    const std::shared_ptr<pddl::Action> &action,
    const std::vector<std::string> &params) {

  const auto parameters = action->get_parameters();
  std::unordered_map<std::string, int> index;
  index.reserve(parameters.size());
  for (size_t i = 0; i < parameters.size(); ++i) {
    index[parameters[i].get_name()] = static_cast<int>(i);
  }

  const auto effects = action->get_effects();
  auto apply = [&](pddl::Type timing) {
    for (const auto &effect : effects) {
      if (effect.get_type() != timing) {
        continue;
      }
      std::vector<std::string> args;
      const auto effect_args = effect.get_args();
      args.reserve(effect_args.size());
      for (const auto &arg : effect_args) {
        auto it = index.find(arg);
        if (it != index.end() && it->second < static_cast<int>(params.size())) {
          args.push_back(params[it->second]);
        } else {
          args.push_back(arg);
        }
      }
      pddl::Predicate predicate(effect.get_name(), args);
      if (effect.is_negated()) {
        facts.erase(predicate);
      } else {
        facts.insert(std::move(predicate));
      }
    }
  };

  apply(pddl::Type::START);
  apply(pddl::Type::END);
}

bool ComponentComposer::compose(
    const pddl::Domain &domain, const pddl::Problem &problem,
    const std::set<pddl::Predicate> &relevant_facts,
    const std::set<std::string> &full_static_predicates, pddl::Plan &out_plan) {

  // 1. Union-find grouping: goals sharing any object are one component.
  std::unordered_map<std::string, std::string> parent;
  for (const auto &goal : problem.get_goals()) {
    const auto args = goal.get_args();
    if (args.empty()) {
      continue;
    }
    for (size_t i = 1; i < args.size(); ++i) {
      unite(parent, args[0], args[i]);
    }
  }

  std::vector<std::vector<pddl::Predicate>> components;
  std::unordered_map<std::string, size_t> root_to_index;
  for (const auto &goal : problem.get_goals()) {
    const auto args = goal.get_args();
    const std::string root = args.empty()
                                 ? "__anon_" + std::to_string(components.size())
                                 : find_root(parent, args[0]);
    auto it = root_to_index.find(root);
    if (it == root_to_index.end()) {
      root_to_index[root] = components.size();
      components.push_back({});
    }
    components[root_to_index[root]].push_back(goal);
  }

  if (components.size() < 2) {
    return false;
  }
  for (const auto &component : components) {
    if (component.size() > options_.max_goals_per_component) {
      return false;
    }
  }

  // 2. Deterministic ordering, optional priority predicate first.
  auto has_priority = [this](const std::vector<pddl::Predicate> &component) {
    if (options_.priority_predicate.empty()) {
      return false;
    }
    return std::any_of(component.begin(), component.end(),
                       [this](const pddl::Predicate &goal) {
                         return goal.get_name() == options_.priority_predicate;
                       });
  };

  std::vector<std::string> keys;
  keys.reserve(components.size());
  for (const auto &component : components) {
    keys.push_back(component_key(component));
  }
  std::vector<size_t> order(components.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](size_t ia, size_t ib) {
    const auto &a = components[ia];
    const auto &b = components[ib];
    const bool a_priority = has_priority(a);
    const bool b_priority = has_priority(b);
    if (a_priority != b_priority) {
      return a_priority;
    }
    if (a.size() != b.size()) {
      return a.size() < b.size();
    }
    return keys[ia] < keys[ib];
  });
  std::vector<std::vector<pddl::Predicate>> ordered;
  ordered.reserve(components.size());
  for (const size_t index : order) {
    ordered.push_back(std::move(components[index]));
  }
  components = std::move(ordered);

  // 3. Static objects and facts; simulated state starts from relevant facts.
  std::set<std::string> static_objects;
  for (const auto &fact : problem.get_facts()) {
    if (!full_static_predicates.count(fact.get_name())) {
      continue;
    }
    for (const auto &arg : fact.get_args()) {
      static_objects.insert(arg);
    }
  }

  std::set<pddl::Predicate> state = relevant_facts;

  std::set<pddl::Predicate> static_facts;
  for (const auto &fact : problem.get_facts()) {
    if (full_static_predicates.count(fact.get_name())) {
      static_facts.insert(fact);
    }
  }

  // 4. Build one reduced sub-problem per component.
  struct PreparedComponent {
    pddl::Problem problem;
    std::set<std::string> mutable_objects;
    std::string key;
  };

  const auto prepare_component =
      [&](const std::vector<pddl::Predicate> &component,
          const std::set<pddl::Predicate> &state) -> PreparedComponent {
    std::set<std::string> comp_objects = static_objects;
    for (const auto &obj : problem.get_objects()) {
      if (obj.get_type() == options_.robot_type) {
        comp_objects.insert(obj.get_name());
      }
    }
    for (const auto &goal : component) {
      for (const auto &arg : goal.get_args()) {
        comp_objects.insert(arg);
      }
    }

    std::unordered_map<std::string, std::vector<const pddl::Predicate *>>
        object_facts;
    for (const auto &fact : state) {
      for (const auto &arg : fact.get_args()) {
        object_facts[arg].push_back(&fact);
      }
    }
    std::vector<std::string> queue(comp_objects.begin(), comp_objects.end());
    for (size_t qi = 0; qi < queue.size(); ++qi) {
      auto it = object_facts.find(queue[qi]);
      if (it == object_facts.end()) {
        continue;
      }
      for (const auto *fact : it->second) {
        for (const auto &arg : fact->get_args()) {
          if (comp_objects.insert(arg).second) {
            queue.push_back(arg);
          }
        }
      }
    }

    PreparedComponent pc;
    pc.key = component_key(component);
    for (const auto &obj : problem.get_objects()) {
      if (comp_objects.count(obj.get_name())) {
        pc.problem.add_object(obj);
      }
    }
    for (const auto &fact : state) {
      if (fact_inside(fact, comp_objects)) {
        pc.problem.add_fact(fact);
      }
    }
    for (const auto &fact : static_facts) {
      if (fact_inside(fact, comp_objects)) {
        pc.problem.add_fact(fact);
      }
    }
    for (const auto &goal : component) {
      pc.problem.add_goal(goal);
    }
    for (const auto &object : comp_objects) {
      if (!static_objects.count(object)) {
        pc.mutable_objects.insert(object);
      }
    }
    return pc;
  };

  // First pass from the initial state only to decide parallel eligibility.
  std::vector<PreparedComponent> prepared;
  prepared.reserve(components.size());
  for (const auto &component : components) {
    prepared.push_back(prepare_component(component, state));
  }

  // 5. Solve. Parallel only when mutable objects are pairwise disjoint.
  std::vector<std::set<std::string>> mutable_sets;
  mutable_sets.reserve(prepared.size());
  for (const auto &pc : prepared) {
    mutable_sets.push_back(pc.mutable_objects);
  }
  const bool parallel = options_.parallel_independent && prepared.size() > 1 &&
                        can_solve_in_parallel(mutable_sets);

  pddl::Plan composed;
  float t = 0.0f;

  auto append_plan = [&](const pddl::Plan &sub_plan) {
    for (size_t i = 0; i < sub_plan.size(); ++i) {
      auto [action, params] = sub_plan.get_action_with_params(i);
      composed.add_action(action, params, t);
      t += action->get_duration();
    }
  };

  if (parallel) {
    const size_t batch = std::max<size_t>(
        1, static_cast<size_t>(std::thread::hardware_concurrency()));
    std::vector<pddl::Plan> plans(prepared.size());
    for (size_t start = 0; start < prepared.size(); start += batch) {
      const size_t end = std::min(prepared.size(), start + batch);
      std::vector<std::future<pddl::Plan>> futures;
      futures.reserve(end - start);
      for (size_t i = start; i < end; ++i) {
        futures.push_back(
            std::async(std::launch::async, [this, &domain, &prepared, i]() {
              return this->solver_(domain, prepared[i].problem);
            }));
      }
      for (size_t i = start; i < end; ++i) {
        plans[i] = futures[i - start].get();
        if (!plans[i].has_solution()) {
          if (warn_) {
            warn_("CachePlanner: component " + prepared[i].key +
                  " has no solution, falling back to full problem");
          }
          return false;
        }
      }
    }
    for (const auto &plan : plans) {
      append_plan(plan);
    }
  } else {
    for (const auto &component : components) {
      PreparedComponent pc = prepare_component(component, state);
      pddl::Plan sub_plan = this->solver_(domain, pc.problem);
      if (!sub_plan.has_solution()) {
        if (warn_) {
          warn_("CachePlanner: component " + pc.key +
                " has no solution, falling back to full problem");
        }
        return false;
      }
      append_plan(sub_plan);
      for (size_t i = 0; i < sub_plan.size(); ++i) {
        auto [action, params] = sub_plan.get_action_with_params(i);
        apply_action_effects(state, action, params);
      }
    }
  }

  composed.set_has_solution(true);
  if (info_) {
    info_("CachePlanner: composed plan from " +
          std::to_string(components.size()) + " components (" +
          std::to_string(composed.size()) + " actions)");
  }
  out_plan = std::move(composed);
  return true;
}

} // namespace detail
} // namespace omni_plan_cache
