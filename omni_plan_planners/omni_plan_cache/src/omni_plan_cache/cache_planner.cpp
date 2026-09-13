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

#include <algorithm>
#include <optional>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yasmin_ros/yasmin_node.hpp>

#include "omni_plan_cache/cache_planner.hpp"
#include "omni_plan_cache/detail/component_composer.hpp"
#include "omni_plan_cache/detail/plan_adapter.hpp"
#include "omni_plan_cache/detail/relevance_analyzer.hpp"
#include "omni_plan_cache/detail/sha256.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;

CachePlanner::CachePlanner() : Planner() {
  this->add_ros_parameters({
      {"planner_plugin", std::string(), this->wrapped_planner_name_},
      {"validator_plugin", std::string(), this->validator_plugin_name_},
      {"validate_on_hit", true, this->validate_on_hit_},
      {"robot_type", std::string("robot"), this->robot_type_},
      {"component_goal_limit", 4, this->component_goal_limit_},
      {"component_priority_predicate", std::string(),
       this->component_priority_predicate_},
      {"max_exact_cache_entries", 0, this->max_exact_cache_entries_},
      {"max_structural_cache_entries", 0, this->max_structural_cache_entries_},
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

    this->plan_cache_.configure(
        static_cast<size_t>(std::max(0, this->max_exact_cache_entries_)),
        static_cast<size_t>(std::max(0, this->max_structural_cache_entries_)));
  });
}

std::string CachePlanner::sha256(const std::string &input) {
  return detail::sha256_hex(input);
}

std::vector<ObjectsByType>
CachePlanner::group_objects_by_type(const std::set<pddl::Object> &objects) {
  return detail::StructuralKeyer::group_objects_by_type(objects);
}

std::string CachePlanner::compute_structural_key(
    const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<pddl::Predicate> *filtered_facts) {
  return detail::StructuralKeyer::compute_key(
      domain_pddl, problem, objects_by_type, role_keys, filtered_facts);
}
std::unordered_map<std::string, std::string> CachePlanner::compute_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const std::set<pddl::Predicate> &facts,
    const std::set<pddl::Predicate> &goals,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {
  return detail::StructuralKeyer::compute_role_keys(
      objects_by_type, facts, goals, name_to_alias, abstract_keys);
}

std::unordered_map<std::string, std::string> CachePlanner::build_name_mapping(
    const std::unordered_map<std::string, std::string>
        &old_placeholder_to_original,
    const std::vector<ObjectsByType> &new_objects_by_type) {
  return detail::PlanAdapter::build_name_mapping(old_placeholder_to_original,
                                                 new_objects_by_type);
}

std::set<std::string> CachePlanner::compute_relevant_predicates(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem) {
  return detail::RelevanceAnalyzer::relevant_predicates(domain, problem);
}

rclcpp::Logger CachePlanner::log() const {
  return this->node_ ? this->node_->get_logger()
                     : rclcpp::get_logger("CachePlanner");
}

std::optional<omni_plan::pddl::Plan> CachePlanner::try_structural_hit(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem, const std::string &structural_key,
    const detail::PreparedStructure &prepared, bool abstract_keys) const {
  auto entry = this->plan_cache_.get_structural(structural_key);
  if (!entry) {
    return std::nullopt;
  }
  return this->serve_cached_entry(*entry, domain, problem, prepared,
                                  abstract_keys);
}

std::optional<omni_plan::pddl::Plan> CachePlanner::serve_cached_entry(
    const CachedPlanData &entry, const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem,
    const detail::PreparedStructure &prepared, bool abstract_keys) const {

  const auto old_to_new = detail::PlanAdapter::build_name_mapping(
      entry.placeholder_to_original, prepared.objects_by_type);
  bool needs_rename = false;
  for (const auto &[old_name, new_name] : old_to_new) {
    if (old_name != new_name) {
      needs_rename = true;
      break;
    }
  }

  omni_plan::pddl::Plan plan =
      needs_rename ? detail::PlanAdapter::adapt(entry, old_to_new) : entry.plan;
  if (needs_rename) {
    this->plan_cache_.record_adaptation();
  }

  const bool must_validate = this->validator_ && this->validate_on_hit_ &&
                             (needs_rename || abstract_keys);
  if (must_validate) {
    this->plan_cache_.record_validation();
    if (!this->validator_->validate_plan(domain, problem, plan)) {
      RCLCPP_WARN(this->log(),
                  "CachePlanner: structural cache hit but adapted plan "
                  "invalid; falling through to cache miss");
      return std::nullopt;
    }
  }

  RCLCPP_INFO(this->log(), "CachePlanner: Structural cache hit%s",
              must_validate ? " (validated)" : "");
  return plan;
}

omni_plan::pddl::Plan CachePlanner::compute_miss_plan(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem, const std::string &exact_key,
    const std::string &structural_key, const detail::RelevanceResult &relevance,
    const detail::PreparedStructure &prepared) const {

  if (this->validator_) {
    detail::ComponentComposer composer(
        detail::ComponentComposer::Options{
            this->robot_type_, this->component_priority_predicate_,
            static_cast<size_t>(std::max(0, this->component_goal_limit_)),
            true},
        [this](const omni_plan::pddl::Domain &d,
               const omni_plan::pddl::Problem &p) {
          return this->generate_plan(d, p);
        },
        [this](const std::string &message) {
          RCLCPP_INFO(this->log(), "%s", message.c_str());
        },
        [this](const std::string &message) {
          RCLCPP_WARN(this->log(), "%s", message.c_str());
        });

    omni_plan::pddl::Plan composed;
    if (composer.compose(domain, problem, relevance.relevant_facts,
                         relevance.full_static_predicates, composed)) {
      composed.set_raw_output(composed.to_pddl());
      this->plan_cache_.record_validation();
      if (this->validator_->validate_plan(domain, problem, composed)) {
        auto data = std::make_shared<CachedPlanData>(
            CachedPlanData{composed, prepared.placeholder_to_original});
        this->plan_cache_.record_composition();
        this->plan_cache_.put(exact_key, structural_key, data);
        this->plan_cache_.publish(structural_key, data);
        return composed;
      }
      this->plan_cache_.record_composition_fallback();
      RCLCPP_WARN(this->log(),
                  "CachePlanner: composed plan invalid, falling back to full "
                  "problem");
    }
  }

  auto plan = this->delegate_plan(domain, problem, structural_key);
  RCLCPP_INFO(this->log(),
              "CachePlanner: Cache miss, delegating to sub-planner");

  if (this->should_cache_result(plan)) {
    auto data = std::make_shared<CachedPlanData>(
        CachedPlanData{plan, prepared.placeholder_to_original});
    this->plan_cache_.put(exact_key, structural_key, data);
    this->plan_cache_.publish(structural_key, data);
  } else {
    this->plan_cache_.publish(structural_key, nullptr);
  }
  return plan;
}

omni_plan::pddl::Plan
CachePlanner::generate_plan(const omni_plan::pddl::Domain &domain,
                            const omni_plan::pddl::Problem &problem) const {

  const bool abstract_keys =
      this->validate_on_hit_ && (this->validator_ != nullptr);

  const std::string exact_key =
      detail::StructuralKeyer::exact_key(domain, problem);

  if (auto cached = this->plan_cache_.get_exact(exact_key)) {
    RCLCPP_INFO(this->log(), "CachePlanner: Exact cache hit");
    return cached->plan;
  }

  const auto relevance =
      detail::RelevanceAnalyzer::analyze(domain, problem, this->robot_type_);
  const auto prepared = detail::StructuralKeyer::prepare(
      problem.get_objects(), relevance.relevant_facts, problem.get_goals(),
      abstract_keys);
  const std::string structural_key = detail::StructuralKeyer::compute_key(
      domain, problem, prepared.objects_by_type, prepared.role_keys,
      &relevance.relevant_facts);

  if (auto plan = this->try_structural_hit(domain, problem, structural_key,
                                           prepared, abstract_keys)) {
    return *plan;
  }

  this->plan_cache_.record_full_miss();

  auto flight = this->plan_cache_.begin_or_join(structural_key);

  if (flight.leader) {
    try {
      return this->compute_miss_plan(domain, problem, exact_key, structural_key,
                                     relevance, prepared);
    } catch (...) {
      this->plan_cache_.publish_failure(structural_key,
                                        std::current_exception());
      throw;
    }
  }

  std::shared_ptr<const CachedPlanData> shared;
  try {
    shared = flight.future.get();
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->log(), "CachePlanner: in-flight planning failed (%s)",
                e.what());
    shared = nullptr;
  } catch (...) {
    RCLCPP_WARN(this->log(),
                "CachePlanner: in-flight planning failed (unknown error)");
    shared = nullptr;
  }
  if (shared) {
    if (auto plan = this->serve_cached_entry(*shared, domain, problem, prepared,
                                             abstract_keys)) {
      this->plan_cache_.record_structural_hit();
      this->plan_cache_.put_exact(
          exact_key, std::make_shared<CachedPlanData>(CachedPlanData{
                         *plan, prepared.placeholder_to_original}));
      return *plan;
    }
  }

  return this->compute_miss_plan(domain, problem, exact_key, structural_key,
                                 relevance, prepared);
}

CacheStats CachePlanner::get_cache_stats() const {
  return this->plan_cache_.stats();
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
