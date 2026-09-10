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

#include <openssl/sha.h>

#include <algorithm>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yasmin_ros/yasmin_node.hpp>

#include "omni_plan_cache/cache_planner.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;

CachePlanner::CachePlanner() : Planner() {
  this->add_ros_parameters({
      {"planner_plugin", std::string(), this->wrapped_planner_name_},
      {"validator_plugin", std::string(), this->validator_plugin_name_},
      {"validate_on_hit", true, this->validate_on_hit_},
  });

  try {
    this->planner_loader_ =
        std::make_unique<pluginlib::ClassLoader<omni_plan::Planner>>(
            "omni_plan", "omni_plan::Planner");
  } catch (const std::exception &e) {
    throw std::runtime_error(
        std::string("Failed to create planner ClassLoader: ") + e.what());
  }

  this->add_loaded_params_callback([this]() {
    this->node_ = yasmin_ros::YasminNode::get_instance();

    if (!this->wrapped_planner_name_.empty()) {
      try {
        this->wrapped_planner_ = std::shared_ptr<omni_plan::Planner>(
            this->planner_loader_->createUnmanagedInstance(
                this->wrapped_planner_name_));
        this->wrapped_planner_->set_namespace("planner.sub_planner");
        try {
          this->wrapped_planner_->load_ros_parameters(this->node_);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->node_->get_logger(),
                      "Wrapped planner '%s': error loading params (%s); "
                      "code defaults will be used.",
                      this->wrapped_planner_name_.c_str(), e.what());
        }
        RCLCPP_INFO(this->node_->get_logger(), "Wrapped planner '%s' loaded",
                    this->wrapped_planner_name_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "Failed to load wrapped planner '%s': %s",
                     this->wrapped_planner_name_.c_str(), e.what());
      }
    }

    // --- Load optional plan validator ---
    if (!this->validator_plugin_name_.empty()) {
      try {
        this->validator_loader_ =
            std::make_unique<pluginlib::ClassLoader<omni_plan::PlanValidator>>(
                "omni_plan", "omni_plan::PlanValidator");
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "Failed to create validator ClassLoader: %s", e.what());
        return;
      }

      try {
        this->validator_ = std::shared_ptr<omni_plan::PlanValidator>(
            this->validator_loader_->createUnmanagedInstance(
                this->validator_plugin_name_));
        this->validator_->set_namespace("planner.validator");
        try {
          this->validator_->load_ros_parameters(this->node_);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->node_->get_logger(),
                      "Validator '%s': error loading params (%s); "
                      "code defaults will be used.",
                      this->validator_plugin_name_.c_str(), e.what());
        }
        RCLCPP_INFO(this->node_->get_logger(), "Validator '%s' loaded",
                    this->validator_plugin_name_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "Failed to load validator '%s': %s",
                     this->validator_plugin_name_.c_str(), e.what());
      }
    }

    // Abstract structural keys are only safe when every hit is validated:
    // the validator rejects bad adaptations and the cache falls back to the
    // sub-planner. With abstract keys the structural key is invariant to the
    // concrete object identities (robot position, which item is at which
    // counter, which table is the goal), which is what makes repeated plan
    // structures actually hit.
    this->abstract_role_keys_ =
        this->validate_on_hit_ && (this->validator_ != nullptr);
    if (this->abstract_role_keys_) {
      RCLCPP_INFO(this->node_->get_logger(),
                  "CachePlanner: abstract structural keys enabled "
                  "(validated hits)");
    }
  });
}

std::string CachePlanner::sha256(const std::string &input) {
  unsigned char hash[SHA256_DIGEST_LENGTH];
  SHA256(reinterpret_cast<const unsigned char *>(input.c_str()), input.size(),
         hash);

  std::stringstream ss;
  for (int i = 0; i < SHA256_DIGEST_LENGTH; i++) {
    ss << std::hex << std::setw(2) << std::setfill('0')
       << static_cast<int>(hash[i]);
  }
  return ss.str();
}

std::vector<ObjectsByType>
CachePlanner::group_objects_by_type(const std::set<pddl::Object> &objects) {
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
  for (const auto &type : type_order) {
    result.push_back({type, type_to_names[type]});
  }
  return result;
}

std::string CachePlanner::compute_structural_key(
    const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<pddl::Predicate> *filtered_facts) {

  // Build name -> type lookup
  std::unordered_map<std::string, std::string> name_to_type;
  for (const auto &group : objects_by_type) {
    for (const auto &name : group.names) {
      name_to_type[name] = group.type;
    }
  }

  // Abstract a predicate to its type signature
  auto type_signature = [&name_to_type](const std::string &name,
                                        const std::vector<std::string> &args) {
    std::string s = name;
    for (const auto &arg : args) {
      auto it = name_to_type.find(arg);
      s += ":" + (it != name_to_type.end() ? it->second : arg);
    }
    return s;
  };

  std::vector<std::string> parts;

  // Object-type counts
  for (const auto &group : objects_by_type) {
    parts.push_back(group.type + ":" + std::to_string(group.names.size()));
  }

  // Type-abstracted init predicates (sorted)
  // Use filtered facts if provided (for relevance-based structural matching).
  {
    std::vector<std::string> entries;
    const auto &facts = filtered_facts ? *filtered_facts : problem.get_facts();
    for (const auto &fact : facts) {
      entries.push_back(type_signature(fact.get_name(), fact.get_args()));
    }
    std::sort(entries.begin(), entries.end());
    for (const auto &e : entries) {
      parts.push_back("I:" + e);
    }
  }

  // Type-abstracted goal predicates (sorted)
  {
    std::vector<std::string> entries;
    for (const auto &goal : problem.get_goals()) {
      entries.push_back(type_signature(goal.get_name(), goal.get_args()));
    }
    std::sort(entries.begin(), entries.end());
    for (const auto &e : entries) {
      parts.push_back("G:" + e);
    }
  }

  // Sorted role-key signatures - captures which objects have which
  // predicate-usage patterns, ignoring the concrete object names. Two
  // problems with the same sorted set of role keys have objects with
  // the same role distribution, enabling correct name mapping on hit.
  {
    std::vector<std::string> sorted_roles;
    for (const auto &kv : role_keys) {
      sorted_roles.push_back(kv.second);
    }
    std::sort(sorted_roles.begin(), sorted_roles.end());
    for (const auto &r : sorted_roles) {
      parts.push_back("R:" + r);
    }
  }

  // Build the abstraction string
  std::string abstraction;
  for (const auto &p : parts) {
    abstraction += p + "|";
  }

  return sha256(domain_pddl + "|ABSTRACT|" + abstraction);
}
std::unordered_map<std::string, std::string> CachePlanner::compute_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const std::set<pddl::Predicate> &facts,
    const std::set<pddl::Predicate> &goals,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {

  // Collect a list of "contributions" for each object: one entry per
  // predicate argument position the object occupies.  Each contribution
  // encodes:
  //   "{predicate_name}_{arg_index}_{is_goal?1:0}"
  // and, when @p name_to_alias is provided, also appends the alias (or
  // concrete name) of every co-occurring argument, forming a "concrete"
  // key that distinguishes attribute distributions.
  std::unordered_map<std::string, std::vector<std::string>> obj_entries;

  auto add_entries = [&](const std::set<pddl::Predicate> &preds, bool is_goal) {
    for (const auto &p : preds) {
      auto args = p.get_args();
      for (size_t i = 0; i < args.size(); i++) {
        std::string contrib = p.get_name() + "_" + std::to_string(i) + "_" +
                              (is_goal ? "1" : "0");
        if (name_to_alias && !abstract_keys) {
          for (size_t j = 0; j < args.size(); j++) {
            if (j != i) {
              auto it = name_to_alias->find(args[j]);
              if (it != name_to_alias->end()) {
                contrib += "_" + it->second;
              } else {
                contrib += "_" + args[j];
              }
            }
          }
        }
        obj_entries[args[i]].push_back(contrib);
      }
    }
  };
  add_entries(facts, false);
  add_entries(goals, true);

  std::unordered_map<std::string, std::string> result;
  for (const auto &group : objects_by_type) {
    for (const auto &name : group.names) {
      auto it = obj_entries.find(name);
      if (it != obj_entries.end()) {
        std::sort(it->second.begin(), it->second.end());
        std::string key;
        for (const auto &e : it->second) {
          key += e + "|";
        }
        result[name] = key;
      } else {
        result[name].clear();
      }
    }
  }

  return result;
}

std::unordered_map<std::string, std::string> CachePlanner::build_name_mapping(
    const std::unordered_map<std::string, std::string>
        &old_placeholder_to_original,
    const std::vector<ObjectsByType> &new_objects_by_type) {

  std::unordered_map<std::string, std::string> mapping;

  for (const auto &group : new_objects_by_type) {
    for (size_t i = 0; i < group.names.size(); i++) {
      std::string placeholder =
          "__obj_" + group.type + "_" + std::to_string(i) + "__";
      auto it = old_placeholder_to_original.find(placeholder);
      if (it != old_placeholder_to_original.end()) {
        mapping[it->second] = group.names[i];
      }
    }
  }

  return mapping;
}

std::set<std::string> CachePlanner::compute_relevant_predicates(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem) {

  // Backward-chaining relevance analysis: starting from the goal predicates,
  // trace through action effects to find precondition predicates that could
  // influence plan generation.  Predicates never reached are irrelevant.
  std::set<std::string> relevant;
  std::set<std::string> frontier;

  for (const auto &goal : problem.get_goals()) {
    frontier.insert(goal.get_name());
  }

  while (!frontier.empty()) {
    std::string current = *frontier.begin();
    frontier.erase(frontier.begin());

    if (!relevant.insert(current).second) {
      continue;
    }

    for (const auto &[name, action] : domain.get_actions()) {
      bool affects_current = false;
      for (const auto &effect : action->get_effects()) {
        if (effect.get_name() == current) {
          affects_current = true;
          break;
        }
      }

      if (!affects_current) {
        continue;
      }

      for (const auto &cond : action->get_conditions()) {
        if (!relevant.count(cond.get_name())) {
          frontier.insert(cond.get_name());
        }
      }
    }
  }

  return relevant;
}

omni_plan::pddl::Plan CachePlanner::adapt_cached_plan(
    const CachedPlan &cached,
    const std::unordered_map<std::string, std::string> &old_to_new) const {

  // Structural adaptation: rebuild the plan renaming the object names in the
  // action parameters, avoiding any raw-output text manipulation or
  // re-parsing.
  omni_plan::pddl::Plan adapted;
  for (size_t i = 0; i < cached.plan.size(); ++i) {
    auto [action, params] = cached.plan.get_action_with_params(i);
    for (auto &param : params) {
      auto it = old_to_new.find(param);
      if (it != old_to_new.end()) {
        param = it->second;
      }
    }
    adapted.add_action(action, params, cached.plan.get_action_start_time(i));
  }
  adapted.set_has_solution(cached.plan.has_solution());
  return adapted;
}

void CachePlanner::apply_plan_action_effects(
    std::set<pddl::Predicate> &facts,
    const std::shared_ptr<pddl::Action> &action,
    const std::vector<std::string> &params) const {

  auto apply = [&](const std::vector<pddl::Effect> &effects) {
    for (const auto &effect : effects) {
      // Instantiate the effect arguments using the plan parameters.
      std::vector<std::string> args;
      args.reserve(effect.get_args().size());
      for (const auto &arg : effect.get_args()) {
        int idx = action->get_parameter_index(arg);
        if (idx >= 0 && idx < static_cast<int>(params.size())) {
          args.push_back(params[idx]);
        } else {
          args.push_back(arg);
        }
      }

      pddl::Predicate pred(effect.get_name(), args);
      if (effect.is_negated()) {
        facts.erase(pred);
      } else {
        facts.insert(pred);
      }
    }
  };

  apply(action->get_on_start_effects());
  apply(action->get_on_end_effects());
}

bool CachePlanner::compose_from_components(
    const pddl::Domain &domain, const pddl::Problem &problem,
    const std::set<pddl::Predicate> &relevant_facts,
    const std::set<std::string> &full_static_predicates,
    pddl::Plan &out_plan) const {

  // Component composition is only safe when the resulting plan is
  // re-validated; without a validator we fall back to the full solve.
  if (!this->validator_) {
    return false;
  }

  static constexpr size_t MAX_COMPONENT_GOALS = 4;

  // --------------------------------------------------------------
  // 1. Decompose the goals into independent components: goals that
  //    share an object belong to the same component.
  // --------------------------------------------------------------
  std::vector<std::vector<pddl::Predicate>> components;
  for (const auto &goal : problem.get_goals()) {
    bool placed = false;
    for (auto &comp : components) {
      for (const auto &cgoal : comp) {
        bool overlap = false;
        for (const auto &a : goal.get_args()) {
          for (const auto &b : cgoal.get_args()) {
            if (a == b) {
              overlap = true;
              break;
            }
          }
          if (overlap) {
            break;
          }
        }
        if (overlap) {
          comp.push_back(goal);
          placed = true;
          break;
        }
      }
      if (placed) {
        break;
      }
    }
    if (!placed) {
      components.push_back({goal});
    }
  }

  // Only compose when there is more than one independent component and
  // every component is small enough to be solved in isolation.
  if (components.size() < 2) {
    return false;
  }
  for (const auto &comp : components) {
    if (comp.size() > MAX_COMPONENT_GOALS) {
      return false;
    }
  }

  // --------------------------------------------------------------
  // 2. Order the components: the battery/homeostatic component first
  //    (charge before any other action), then by size and name so the
  //    order is deterministic.
  // --------------------------------------------------------------
  auto comp_key = [](const std::vector<pddl::Predicate> &comp) {
    std::string s;
    for (const auto &g : comp) {
      s += g.get_name();
      for (const auto &arg : g.get_args()) {
        s += "_" + arg;
      }
      s += ";";
    }
    return s;
  };
  std::sort(components.begin(), components.end(),
            [&comp_key](const auto &a, const auto &b) {
              bool a_batt =
                  std::any_of(a.begin(), a.end(), [](const pddl::Predicate &g) {
                    return g.get_name() == "battery_ok";
                  });
              bool b_batt =
                  std::any_of(b.begin(), b.end(), [](const pddl::Predicate &g) {
                    return g.get_name() == "battery_ok";
                  });
              if (a_batt != b_batt) {
                return a_batt;
              }
              if (a.size() != b.size()) {
                return a.size() < b.size();
              }
              return comp_key(a) < comp_key(b);
            });

  // --------------------------------------------------------------
  // 3. Build the reduced object sets per component: robots, the
  //    component's goal objects and static-fact objects (rooms,
  //    chargers, ...), expanded through the current state so the
  //    component's spawn locations are included. Unrelated pending
  //    items stay out, keeping the sub-problems small and their
  //    structural keys stable.
  // --------------------------------------------------------------
  std::set<std::string> static_objects;
  for (const auto &fact : problem.get_facts()) {
    if (!full_static_predicates.count(fact.get_name())) {
      continue;
    }
    for (const auto &arg : fact.get_args()) {
      static_objects.insert(arg);
    }
  }

  // Current state, simulated forwards through the composed sub-plans.
  std::set<pddl::Predicate> state = relevant_facts;

  // Full static facts (never change): kept in every component problem so
  // the actions' static parameters (rooms, chargers) can be instantiated.
  std::set<pddl::Predicate> static_facts;
  for (const auto &fact : problem.get_facts()) {
    if (full_static_predicates.count(fact.get_name())) {
      static_facts.insert(fact);
    }
  }

  pddl::Plan composed;
  float t = 0.0f;

  for (const auto &component : components) {
    // Seed: robots + this component's goal objects + static objects.
    std::set<std::string> comp_objects = static_objects;
    for (const auto &obj : problem.get_objects()) {
      if (obj.get_type() == "robot") {
        comp_objects.insert(obj.get_name());
      }
    }
    for (const auto &goal : component) {
      for (const auto &arg : goal.get_args()) {
        comp_objects.insert(arg);
      }
    }

    // Closure: expand through the current state so the component's spawn
    // locations and related objects are included, while unrelated pending
    // items stay out.
    bool changed = true;
    while (changed) {
      changed = false;
      for (const auto &fact : state) {
        bool any = false;
        for (const auto &arg : fact.get_args()) {
          if (comp_objects.count(arg)) {
            any = true;
            break;
          }
        }
        if (any) {
          for (const auto &arg : fact.get_args()) {
            if (!comp_objects.count(arg)) {
              comp_objects.insert(arg);
              changed = true;
            }
          }
        }
      }
    }

    // Component sub-problem: reduced objects, the current state facts
    // restricted to the component objects, and the component goals.
    pddl::Problem sub_problem;
    for (const auto &obj : problem.get_objects()) {
      if (comp_objects.count(obj.get_name())) {
        sub_problem.add_object(obj);
      }
    }
    for (const auto &fact : state) {
      bool inside = true;
      for (const auto &arg : fact.get_args()) {
        if (!comp_objects.count(arg)) {
          inside = false;
          break;
        }
      }
      if (inside) {
        sub_problem.add_fact(fact);
      }
    }
    for (const auto &fact : static_facts) {
      bool inside = true;
      for (const auto &arg : fact.get_args()) {
        if (!comp_objects.count(arg)) {
          inside = false;
          break;
        }
      }
      if (inside) {
        sub_problem.add_fact(fact);
      }
    }
    for (const auto &goal : component) {
      sub_problem.add_goal(goal);
    }

    // Solve (or hit the cache for) the component sub-problem. The
    // recursion is safe: the sub-problem has a single component, so the
    // composition bails out inside and the normal exact/structural/delegate
    // path is used.
    pddl::Plan sub_plan = this->generate_plan(domain, sub_problem);
    if (!sub_plan.has_solution()) {
      RCLCPP_WARN(this->node_->get_logger(),
                  "CachePlanner: component %s has no solution, "
                  "falling back to full problem",
                  comp_key(component).c_str());
      return false;
    }

    // Append the sub-plan re-timed sequentially.
    for (size_t i = 0; i < sub_plan.size(); ++i) {
      auto [action, params] = sub_plan.get_action_with_params(i);
      composed.add_action(action, params, t);
      t += action->get_duration();
    }

    // Simulate the sub-plan's effects to obtain the state for the next
    // component.
    for (size_t i = 0; i < sub_plan.size(); ++i) {
      auto [action, params] = sub_plan.get_action_with_params(i);
      this->apply_plan_action_effects(state, action, params);
    }
  }

  composed.set_has_solution(true);

  // Safety net: validate the composed plan against the full problem.
  if (!this->validator_->validate_plan(domain, problem, composed)) {
    RCLCPP_WARN(this->node_->get_logger(),
                "CachePlanner: composed plan invalid, falling back to full "
                "problem");
    return false;
  }

  RCLCPP_INFO(this->node_->get_logger(),
              "CachePlanner: composed plan from %zu components (%zu actions)",
              components.size(), composed.size());
  out_plan = std::move(composed);
  return true;
}

pddl::Plan CachePlanner::generate_plan(const pddl::Domain &domain,
                                       const pddl::Problem &problem) const {
  std::string domain_pddl = domain.to_pddl();
  std::string problem_pddl = problem.to_pddl();

  // Level-1 key: SHA-256 of the full domain + problem PDDL (exact match).
  std::string exact_key = this->sha256(domain_pddl + problem_pddl);

  // Fast path: check exact cache first, before any structural work.
  {
    std::shared_lock lock(this->cache_mutex_);
    auto it = this->exact_cache_.find(exact_key);
    if (it != this->exact_cache_.end()) {
      RCLCPP_INFO(this->node_->get_logger(), "CachePlanner: Exact cache hit");
      return it->second;
    }
  }

  // ------------------------------------------------------------------
  // Structural path — compute keys and check structural cache
  // ------------------------------------------------------------------

  // Group objects by type for structural normalization.
  auto objects_by_type = this->group_objects_by_type(problem.get_objects());

  // --- Relevance analysis ---
  // Backward-chain through actions from the goals to find which predicates
  // can influence the plan.  Facts with irrelevant predicates (e.g. purely
  // observational state that no action precondition depends on) are excluded
  // from the structural key, enabling cache hits when only irrelevant parts
  // of the initial state differ.
  auto relevant_predicates = this->compute_relevant_predicates(domain, problem);

  // Static relevant predicates: never touched by any action effect. Their
  // facts describe fixed objects that the plan still references (e.g. the
  // chargers via is_charger).
  std::set<std::string> static_predicates;
  // Domain-wide static predicates (not necessarily relevant): needed to
  // keep the objects they mention (rooms, chargers, ...) in the reduced
  // component problems so the actions' static parameters can be
  // instantiated.
  std::set<std::string> full_static_predicates;
  for (const auto &pred : domain.get_predicates()) {
    full_static_predicates.insert(pred.get_name());
  }
  for (const auto &[name, action] : domain.get_actions()) {
    for (const auto &effect : action->get_effects()) {
      full_static_predicates.erase(effect.get_name());
    }
  }
  for (const auto &pred : relevant_predicates) {
    if (!full_static_predicates.count(pred)) {
      continue;
    }
    static_predicates.insert(pred);
  }

  // Objects that can influence the plan: objects appearing in the goals,
  // the robots (their state is always relevant), and the objects of static
  // relevant facts (e.g. chargers).
  std::set<std::string> relevant_objects;
  for (const auto &goal : problem.get_goals()) {
    for (const auto &arg : goal.get_args()) {
      relevant_objects.insert(arg);
    }
  }
  for (const auto &group : objects_by_type) {
    if (group.type == "robot") {
      relevant_objects.insert(group.names.begin(), group.names.end());
    }
  }
  for (const auto &fact : problem.get_facts()) {
    if (!static_predicates.count(fact.get_name())) {
      continue;
    }
    for (const auto &arg : fact.get_args()) {
      relevant_objects.insert(arg);
    }
  }

  // Object-level relevance: keep a fact when it is static, mentions a robot,
  // or touches a goal object. Facts about already-delivered items and other
  // irrelevant objects (e.g. item_at(delivered_item, table)) no longer
  // change the structural key: they do not constrain the plan, and keeping
  // them out is what makes structurally repeated plan requests hit the
  // cache instead of constantly missing because of monotonic bookkeeping.
  std::set<pddl::Predicate> relevant_facts;
  for (const auto &fact : problem.get_facts()) {
    if (!relevant_predicates.count(fact.get_name())) {
      continue;
    }

    if (static_predicates.count(fact.get_name())) {
      relevant_facts.insert(fact);
      continue;
    }

    bool keep = false;
    for (const auto &arg : fact.get_args()) {
      if (relevant_objects.count(arg)) {
        keep = true;
        break;
      }
    }

    if (keep) {
      relevant_facts.insert(fact);
    }
  }

  // --- Pass 1: Abstract role keys (concrete-name-free) ---
  auto abstract_keys = this->compute_role_keys(objects_by_type, relevant_facts,
                                               problem.get_goals());

  // Filter out objects that never appear in any relevant predicate.
  std::vector<ObjectsByType> filtered_objects_by_type;
  for (const auto &group : objects_by_type) {
    ObjectsByType fg;
    fg.type = group.type;
    for (const auto &name : group.names) {
      auto it = abstract_keys.find(name);
      if (it != abstract_keys.end() && !it->second.empty()) {
        fg.names.push_back(name);
      }
    }
    if (!fg.names.empty()) {
      filtered_objects_by_type.push_back(std::move(fg));
    }
  }

  for (auto &group : filtered_objects_by_type) {
    std::sort(group.names.begin(), group.names.end(),
              [&abstract_keys](const std::string &a, const std::string &b) {
                auto it_a = abstract_keys.find(a);
                auto it_b = abstract_keys.find(b);
                const std::string &key_a =
                    (it_a != abstract_keys.end()) ? it_a->second : "";
                const std::string &key_b =
                    (it_b != abstract_keys.end()) ? it_b->second : "";
                if (key_a != key_b)
                  return key_a < key_b;
                return a < b;
              });
  }

  // Build alias lookup from the sorted positions.
  std::unordered_map<std::string, std::string> name_to_alias;
  for (const auto &group : filtered_objects_by_type) {
    for (size_t i = 0; i < group.names.size(); i++) {
      name_to_alias[group.names[i]] = group.type + "_" + std::to_string(i);
    }
  }

  // --- Pass 2: Concrete role keys ---
  auto concrete_keys = this->compute_role_keys(
      filtered_objects_by_type, relevant_facts, problem.get_goals(),
      &name_to_alias, this->abstract_role_keys_);

  // Level-2 key: role-aware structural hash.
  std::string structural_key = this->compute_structural_key(
      domain_pddl, problem, filtered_objects_by_type, concrete_keys,
      &relevant_facts);

  // Build placeholder map from role-sorted objects.
  std::unordered_map<std::string, std::string> placeholder_map;
  for (const auto &group : filtered_objects_by_type) {
    for (size_t i = 0; i < group.names.size(); i++) {
      std::string placeholder =
          "__obj_" + group.type + "_" + std::to_string(i) + "__";
      placeholder_map[placeholder] = group.names[i];
    }
  }

  // ------------------------------------------------------------------
  // Check structural cache (shared lock)
  // ------------------------------------------------------------------
  {
    std::shared_lock lock(this->cache_mutex_);

    auto it = this->structural_cache_.find(structural_key);
    if (it != this->structural_cache_.end()) {
      // Build the object mapping once; when no renames are required the
      // parsed plan can be reused directly, skipping adaptation and parsing.
      auto old_to_new = this->build_name_mapping(
          it->second.placeholder_to_original, filtered_objects_by_type);
      bool needs_rename = false;
      for (const auto &[old_name, new_name] : old_to_new) {
        if (old_name != new_name) {
          needs_rename = true;
          break;
        }
      }

      pddl::Plan adapted_plan;
      if (needs_rename) {
        adapted_plan = this->adapt_cached_plan(it->second, old_to_new);
      } else {
        RCLCPP_INFO(this->node_->get_logger(),
                    "CachePlanner: Structural cache hit (parsed plan reuse)");
        adapted_plan = it->second.plan;
      }

      if (!needs_rename && !this->abstract_role_keys_) {
        // The plan is reused verbatim (same objects, same concrete
        // structure): no adaptation happened, so no validation is needed.
        RCLCPP_INFO(this->node_->get_logger(),
                    "CachePlanner: Structural cache hit (parsed plan reuse)");
        return it->second.plan;
      }

      if (!this->validator_ || !this->validate_on_hit_ ||

          this->validator_->validate_plan(domain, problem, adapted_plan)) {
        RCLCPP_INFO(this->node_->get_logger(),
                    "CachePlanner: Structural cache hit%s",
                    this->validator_ ? " (validated)" : "");
        return adapted_plan;
      }
      RCLCPP_WARN(this->node_->get_logger(),
                  "CachePlanner: Structural cache hit but adapted plan "
                  "invalid; falling through to cache miss");
    }
  }

  // ------------------------------------------------------------------
  // Component composition: when the goals decompose into independent
  // components, solve/cache each component separately (small problems
  // with highly recurring structures) and stitch the sub-plans together.
  // ------------------------------------------------------------------
  {
    pddl::Plan composed;
    if (this->compose_from_components(domain, problem, relevant_facts,
                                      full_static_predicates, composed)) {
      std::unique_lock lock(this->cache_mutex_);
      this->exact_cache_[exact_key] = composed;
      CachedPlan cp;
      cp.plan = composed;
      cp.placeholder_to_original = std::move(placeholder_map);
      this->structural_cache_[structural_key] = std::move(cp);
      return composed;
    }
  }

  // ------------------------------------------------------------------
  // Cache miss: delegate to the real planner, then store
  // ------------------------------------------------------------------
  auto plan = this->delegate_plan(domain, problem, structural_key);
  RCLCPP_INFO(this->node_->get_logger(),
              "CachePlanner: Cache miss, delegating to sub-planner");

  if (this->should_cache_result(plan)) {
    std::unique_lock lock(this->cache_mutex_);
    this->exact_cache_[exact_key] = plan;
    CachedPlan cp;
    cp.plan = plan;
    cp.placeholder_to_original = std::move(placeholder_map);
    this->structural_cache_[structural_key] = std::move(cp);
  }

  return plan;
}

omni_plan::pddl::Plan
CachePlanner::delegate_plan(const omni_plan::pddl::Domain &domain,
                            const omni_plan::pddl::Problem &problem,
                            const std::string & /*structural_key*/) const {
  if (!this->wrapped_planner_) {
    throw std::runtime_error("CachePlanner: no planner_plugin parameter set");
  }

  return this->wrapped_planner_->generate_plan(domain, problem);
}

bool CachePlanner::should_cache_result(
    const omni_plan::pddl::Plan &plan) const {
  return plan.has_solution();
}

PLUGINLIB_EXPORT_CLASS(omni_plan_cache::CachePlanner, omni_plan::Planner)
