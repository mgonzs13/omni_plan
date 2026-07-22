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

namespace {

bool is_boundary_char(char c) {
  return c == ' ' || c == '(' || c == ')' || c == '\n' || c == '\t' ||
         c == '\r';
}

size_t find_name(const std::string &text, const std::string &name, size_t pos) {
  while ((pos = text.find(name, pos)) != std::string::npos) {
    bool prev_boundary = pos == 0 || is_boundary_char(text[pos - 1]);
    bool next_boundary = pos + name.size() >= text.size() ||
                         is_boundary_char(text[pos + name.size()]);
    if (prev_boundary && next_boundary) {
      return pos;
    }
    pos += name.size();
  }
  return std::string::npos;
}

void replace_name(std::string &text, const std::string &old_name,
                  const std::string &new_name) {
  size_t pos = 0;
  while ((pos = find_name(text, old_name, pos)) != std::string::npos) {
    text.replace(pos, old_name.length(), new_name);
    pos += new_name.length();
  }
}

} // namespace

CachePlanner::CachePlanner() : Planner() {
  this->add_ros_parameters({
      {"planner_plugin", std::string(""), this->wrapped_planner_name_},
  });

  this->add_loaded_params_callback([this]() {
    this->node_ = yasmin_ros::YasminNode::get_instance();

    try {
      this->planner_loader_ =
          std::make_unique<pluginlib::ClassLoader<omni_plan::Planner>>(
              "omni_plan", "omni_plan::Planner");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Failed to create planner ClassLoader: %s", e.what());
      return;
    }

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
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Parameter 'planner_plugin' not set");
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

std::string CachePlanner::normalize_pddl(
    const std::string &pddl_str,
    const std::vector<ObjectsByType> &objects_by_type,
    std::unordered_map<std::string, std::string> &out_placeholder_map) {
  std::string result = pddl_str;

  std::vector<std::pair<std::string, std::string>> replacements;
  for (const auto &group : objects_by_type) {
    for (size_t i = 0; i < group.names.size(); i++) {
      std::string placeholder =
          "__obj_" + group.type + "_" + std::to_string(i) + "__";
      out_placeholder_map[placeholder] = group.names[i];
      replacements.push_back({group.names[i], placeholder});
    }
  }

  std::sort(replacements.begin(), replacements.end(),
            [](const auto &a, const auto &b) {
              return a.first.size() > b.first.size();
            });

  for (const auto &[original, placeholder] : replacements) {
    replace_name(result, original, placeholder);
  }

  return result;
}

std::string CachePlanner::compute_structural_key(
    const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys) {

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
  {
    std::vector<std::string> entries;
    for (const auto &fact : problem.get_facts()) {
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
    const std::set<pddl::Predicate> &goals) {

  std::unordered_map<std::string, std::vector<std::string>> obj_entries;

  auto add_entries = [&obj_entries](const std::set<pddl::Predicate> &preds,
                                    bool is_goal) {
    for (const auto &p : preds) {
      auto args = p.get_args();
      for (size_t i = 0; i < args.size(); i++) {
        obj_entries[args[i]].push_back(p.get_name() + "_" + std::to_string(i) +
                                       "_" + (is_goal ? "1" : "0"));
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

omni_plan::pddl::Plan
CachePlanner::parse_plan(const omni_plan::pddl::Domain &domain,
                         const std::string &str_plan) const {
  if (!this->wrapped_planner_) {
    throw std::runtime_error("CachePlanner: no planner_plugin parameter set");
  }
  return this->wrapped_planner_->parse_plan(domain, str_plan);
}

pddl::Plan CachePlanner::generate_plan(const pddl::Domain &domain,
                                       const pddl::Problem &problem) const {
  std::string domain_pddl = domain.to_pddl();
  std::string problem_pddl = problem.to_pddl();

  // Exact cache
  std::string exact_key = this->sha256(domain_pddl + problem_pddl);
  auto objects_by_type = this->group_objects_by_type(problem.get_objects());

  // Sort objects by role within each type so placeholder indices reflect
  // predicate usage rather than alphabetical name ordering. Two problems
  // with the same structure but different concrete object names will produce
  // the same normalized form and share a structural cache entry.
  auto role_keys = this->compute_role_keys(objects_by_type, problem.get_facts(),
                                           problem.get_goals());
  for (auto &group : objects_by_type) {
    std::sort(group.names.begin(), group.names.end(),
              [&role_keys](const std::string &a, const std::string &b) {
                auto it_a = role_keys.find(a);
                auto it_b = role_keys.find(b);
                const std::string &key_a =
                    (it_a != role_keys.end()) ? it_a->second : "";
                const std::string &key_b =
                    (it_b != role_keys.end()) ? it_b->second : "";
                if (key_a != key_b)
                  return key_a < key_b;
                return a < b;
              });
  }

  // Structural cache key - combines type-level abstraction with role-key
  // signatures. Two problems with the same type counts, same typed
  // predicates, AND the same multiset of role keys get the same hash.
  std::string structural_key = this->compute_structural_key(
      domain_pddl, problem, objects_by_type, role_keys);

  // Build placeholder map from role-sorted objects (used on structural hit
  // for name mapping, and stored on cache miss for future hits).
  std::unordered_map<std::string, std::string> placeholder_map;
  for (const auto &group : objects_by_type) {
    for (size_t i = 0; i < group.names.size(); i++) {
      std::string placeholder =
          "__obj_" + group.type + "_" + std::to_string(i) + "__";
      placeholder_map[placeholder] = group.names[i];
    }
  }

  {
    std::shared_lock lock(this->cache_mutex_);

    // Level 1: Exact match
    {
      auto it = this->exact_cache_.find(exact_key);
      if (it != this->exact_cache_.end()) {
        RCLCPP_INFO(this->node_->get_logger(), "CachePlanner: Exact cache hit");
        return it->second;
      }
    }

    // Level 2: Structural match
    {
      auto it = this->structural_cache_.find(structural_key);
      if (it != this->structural_cache_.end()) {
        // Adapt the cached plan by replacing old object names with new
        // names. Use a two-phase approach with unique intermediate markers
        // to correctly handle swaps (a->b and b->a simultaneously).
        auto old_to_new = this->build_name_mapping(
            it->second.placeholder_to_original, objects_by_type);

        // Phase 1: replace each old name with a unique temporary marker
        std::vector<std::pair<std::string, std::string>> temp_to_new;
        std::string adapted_raw = it->second.raw_output;
        for (const auto &[old_name, new_name] : old_to_new) {
          if (old_name == new_name)
            continue;
          std::string marker = "__TMP_" + old_name + "__";
          replace_name(adapted_raw, old_name, marker);
          temp_to_new.push_back({marker, new_name});
        }

        // Phase 2: replace markers with new names (longest first)
        std::sort(temp_to_new.begin(), temp_to_new.end(),
                  [](const auto &a, const auto &b) {
                    return a.first.size() > b.first.size();
                  });
        for (const auto &[marker, new_name] : temp_to_new) {
          replace_name(adapted_raw, marker, new_name);
        }

        RCLCPP_INFO(this->node_->get_logger(),
                    "CachePlanner: Structural cache hit");
        return this->parse_plan(domain, adapted_raw);
      }
    }
  }

  // Miss: delegate to wrapped planner
  if (!this->wrapped_planner_) {
    throw std::runtime_error("CachePlanner: no planner_plugin parameter set");
  }
  pddl::Plan plan = this->wrapped_planner_->generate_plan(domain, problem);
  RCLCPP_INFO(this->node_->get_logger(),
              "CachePlanner: Cache miss, delegating to wrapped planner");

  {
    std::unique_lock lock(this->cache_mutex_);
    this->exact_cache_[exact_key] = plan;
    CachedPlan cp;
    cp.raw_output = plan.get_raw_output();
    cp.has_solution = plan.has_solution();
    cp.placeholder_to_original = std::move(placeholder_map);
    this->structural_cache_[structural_key] = std::move(cp);
  }

  return plan;
}

PLUGINLIB_EXPORT_CLASS(omni_plan_cache::CachePlanner, omni_plan::Planner)
