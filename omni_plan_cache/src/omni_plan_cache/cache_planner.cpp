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
#include <mutex>
#include <sstream>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>

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

CachePlanner::CachePlanner()
    : Planner(), planner_loader_("omni_plan", "omni_plan::Planner") {
  this->add_ros_parameters({
      {"planner_plugin", std::string(""), this->wrapped_planner_name_},
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

void CachePlanner::ensure_wrapped_planner_loaded() const {
  static std::once_flag flag;
  std::call_once(flag, [this]() {
    if (this->wrapped_planner_) {
      return;
    }

    if (this->wrapped_planner_name_.empty()) {
      throw std::runtime_error("CachePlanner: no planner_plugin parameter set");
    }

    this->wrapped_planner_.reset(this->planner_loader_.createUnmanagedInstance(
        this->wrapped_planner_name_));
    this->wrapped_planner_->set_namespace("planner.sub_planner");
  });
}

pddl::Plan CachePlanner::parse_plan_output(const pddl::Domain &domain,
                                           const std::string &raw_output,
                                           bool has_solution) const {
  pddl::Plan plan;
  plan.set_raw_output(raw_output);
  plan.set_has_solution(has_solution);

  if (!has_solution) {
    return plan;
  }

  const auto &actions = domain.get_actions();
  std::vector<std::string> lines = this->get_lines_with_actions(raw_output);

  for (const auto &line : lines) {
    auto [action_name, parameters] = this->parse_action_line(line);
    if (action_name.empty()) {
      continue;
    }
    float start_time = this->parse_start_time(line);
    plan.add_action(actions.at(action_name), parameters, start_time);
  }

  return plan;
}

pddl::Plan CachePlanner::generate_plan(const pddl::Domain &domain,
                                       const pddl::Problem &problem) const {
  std::string domain_pddl = domain.to_pddl();
  std::string problem_pddl = problem.to_pddl();

  // Exact cache
  std::string exact_key = this->sha256(domain_pddl + problem_pddl);
  auto objects_by_type = this->group_objects_by_type(problem.get_objects());

  // Structural cache key
  std::unordered_map<std::string, std::string> placeholder_map;
  std::string normalized =
      this->normalize_pddl(problem_pddl, objects_by_type, placeholder_map);
  std::string structural_key = this->sha256(domain_pddl + normalized);

  {
    std::shared_lock lock(this->cache_mutex_);

    // Level 1: Exact match
    {
      auto it = this->exact_cache_.find(exact_key);
      if (it != this->exact_cache_.end())
        return it->second;
    }

    // Level 2: Structural match
    {
      auto it = this->structural_cache_.find(structural_key);
      if (it != this->structural_cache_.end()) {

        // Adapt the cached plan to the new problem by replacing old object
        // names with new object names based on the placeholder mapping.
        auto old_to_new = this->build_name_mapping(
            it->second.placeholder_to_original, objects_by_type);
        std::vector<std::pair<std::string, std::string>> sorted_mapping(
            old_to_new.begin(), old_to_new.end());
        std::sort(sorted_mapping.begin(), sorted_mapping.end(),
                  [](const auto &a, const auto &b) {
                    return a.first.size() > b.first.size();
                  });
        std::string adapted_raw = it->second.raw_output;

        for (const auto &[old_name, new_name] : sorted_mapping) {
          replace_name(adapted_raw, old_name, new_name);
        }

        return this->parse_plan_output(domain, adapted_raw,
                                       it->second.has_solution);
      }
    }
  }

  // Miss: delegate to wrapped planner
  this->ensure_wrapped_planner_loaded();
  pddl::Plan plan = wrapped_planner_->generate_plan(domain, problem);

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
