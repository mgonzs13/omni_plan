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
#include <future>
#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "pluginlib/class_loader.hpp"
#include "poirot/poirot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "yasmin_ros/yasmin_node.hpp"

#include "omni_plan_mrta/mrta_planner.hpp"

using namespace omni_plan_mrta;

MrtaPlanner::MrtaPlanner() : omni_plan::Planner() {
  this->add_ros_parameters({
      {"robot_type", std::string("robot"), this->robot_type_},
      {"planner_plugin", std::string(), this->planner_plugin_},
      {"allocator_plugin", std::string(), this->allocator_plugin_},
  });

  this->add_loaded_params_callback([this]() {
    this->node_ = yasmin_ros::YasminNode::get_instance();

    // Load planner ClassLoader
    try {
      this->planner_loader_ =
          std::make_unique<pluginlib::ClassLoader<omni_plan::Planner>>(
              "omni_plan", "omni_plan::Planner");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Failed to create planner ClassLoader: %s", e.what());
      return;
    }

    // Load allocator ClassLoader
    try {
      this->allocator_loader_ =
          std::make_unique<pluginlib::ClassLoader<TaskAllocator>>(
              "omni_plan_mrta", "omni_plan_mrta::TaskAllocator");
    } catch (const std::exception &e) {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Failed to create allocator ClassLoader: %s", e.what());
      return;
    }

    // Instantiate sub-planner
    if (!this->planner_plugin_.empty()) {
      try {
        this->sub_planner_ = std::shared_ptr<omni_plan::Planner>(
            this->planner_loader_->createUnmanagedInstance(
                this->planner_plugin_));
        this->sub_planner_->set_namespace("planner.sub_planner");
        try {
          this->sub_planner_->load_ros_parameters(this->node_);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->node_->get_logger(),
                      "Sub-planner '%s': error loading params (%s); "
                      "code defaults will be used.",
                      this->planner_plugin_.c_str(), e.what());
        }
        RCLCPP_INFO(this->node_->get_logger(), "Sub-planner '%s' loaded",
                    this->planner_plugin_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "Failed to load sub-planner '%s': %s",
                     this->planner_plugin_.c_str(), e.what());
      }
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Parameter 'planner_plugin' not set");
    }

    // Instantiate task allocator
    if (!this->allocator_plugin_.empty()) {
      try {
        this->allocator_ = std::shared_ptr<TaskAllocator>(
            this->allocator_loader_->createUnmanagedInstance(
                this->allocator_plugin_));
        this->allocator_->set_namespace("planner.allocator");
        try {
          this->allocator_->load_ros_parameters(this->node_);
        } catch (const std::exception &e) {
          RCLCPP_WARN(this->node_->get_logger(),
                      "Allocator '%s': error loading params (%s); "
                      "code defaults will be used.",
                      this->allocator_plugin_.c_str(), e.what());
        }
        RCLCPP_INFO(this->node_->get_logger(), "Task allocator '%s' loaded",
                    this->allocator_plugin_.c_str());
      } catch (const std::exception &e) {
        RCLCPP_ERROR(this->node_->get_logger(),
                     "Failed to load task allocator '%s': %s",
                     this->allocator_plugin_.c_str(), e.what());
      }
    } else {
      RCLCPP_ERROR(this->node_->get_logger(),
                   "Parameter 'allocator_plugin' not set");
    }
  });
}

std::vector<std::string>
MrtaPlanner::extract_robots(const omni_plan::pddl::Problem &problem) const {
  std::vector<std::string> robots;
  for (const auto &obj : problem.get_objects()) {
    if (obj.get_type() == this->robot_type_) {
      robots.push_back(obj.get_name());
    }
  }
  return robots;
}

std::vector<omni_plan_mrta::TeamAllocation> MrtaPlanner::allocate_goals(
    const std::vector<std::string> &robots,
    const std::vector<omni_plan::pddl::Predicate> &goals,
    const omni_plan::pddl::Problem &problem,
    const std::map<std::string, std::shared_ptr<omni_plan::pddl::Action>>
        &actions) const {
  PROFILE_FUNCTION();
  return this->allocator_->allocate(robots, goals, problem, actions);
}

omni_plan::pddl::Plan MrtaPlanner::merge_plans(
    const std::vector<omni_plan::pddl::Plan> &plans) const {
  struct TimedAction {
    float start_time;
    std::shared_ptr<omni_plan::pddl::Action> action;
    std::vector<std::string> params;
    float duration;
  };

  std::vector<TimedAction> all_actions;
  for (const auto &plan : plans) {
    for (size_t i = 0; i < plan.size(); ++i) {
      auto action = plan.get_action(i);
      all_actions.push_back({plan.get_action_start_time(i), action,
                             plan.get_action_params(i),
                             action->get_duration()});
    }
  }

  std::sort(all_actions.begin(), all_actions.end(),
            [](const TimedAction &a, const TimedAction &b) {
              return a.start_time < b.start_time;
            });

  omni_plan::pddl::Plan merged;
  merged.set_has_solution(!all_actions.empty());
  for (const auto &ta : all_actions) {
    merged.add_action(ta.action, ta.params, ta.start_time);
  }
  return merged;
}

omni_plan::pddl::Plan
MrtaPlanner::generate_plan(const omni_plan::pddl::Domain &domain,
                           const omni_plan::pddl::Problem &problem) const {

  omni_plan::pddl::Plan empty_plan;
  empty_plan.set_has_solution(false);

  if (!this->sub_planner_) {
    RCLCPP_ERROR(this->node_->get_logger(), "No sub-planner available");
    return empty_plan;
  }

  if (!this->allocator_) {
    RCLCPP_ERROR(this->node_->get_logger(), "No task allocator available");
    return empty_plan;
  }

  auto robots = this->extract_robots(problem);

  RCLCPP_INFO(this->node_->get_logger(), "Found %zu robots of type '%s'",
              robots.size(), this->robot_type_.c_str());
  for (const auto &r : robots) {
    RCLCPP_INFO(this->node_->get_logger(), "  Robot: '%s'", r.c_str());
  }

  // Single robot or no robots: delegate directly
  if (robots.size() <= 1) {
    RCLCPP_INFO(this->node_->get_logger(),
                "Single robot — delegating to sub-planner directly");
    return this->sub_planner_->generate_plan(domain, problem);
  }

  std::vector<omni_plan::pddl::Predicate> goals_vec(problem.get_goals().begin(),
                                                    problem.get_goals().end());
  if (goals_vec.empty()) {
    RCLCPP_WARN(this->node_->get_logger(), "No goals in problem");
    return empty_plan;
  }

  RCLCPP_INFO(this->node_->get_logger(),
              "Allocating %zu goals to %zu robots via '%s'", goals_vec.size(),
              robots.size(), this->allocator_plugin_.c_str());

  auto allocation =
      this->allocate_goals(robots, goals_vec, problem, domain.get_actions());

  if (allocation.empty()) {
    RCLCPP_ERROR(this->node_->get_logger(), "Allocator returned empty list");
    return empty_plan;
  }

  const std::set<std::string> all_robot_names(robots.begin(), robots.end());

  // Build per-team sub-problems and launch planners in parallel.
  // A team may contain one or more robots; all team members are included in
  // the shared sub-problem so the sub-planner can produce coordinated plans.
  std::vector<std::future<omni_plan::pddl::Plan>> futures;
  std::vector<std::string> team_labels; // human-readable label per entry

  for (const auto &team : allocation) {
    if (team.goal_indices.empty()) {
      continue;
    }

    const std::set<std::string> team_robots(team.robots.begin(),
                                            team.robots.end());

    omni_plan::pddl::Problem sub_problem;

    // Objects: keep all non-robot objects plus the team's robots.
    for (const auto &obj : problem.get_objects()) {
      if (all_robot_names.count(obj.get_name()) == 0 ||
          team_robots.count(obj.get_name()) > 0) {
        sub_problem.add_object(obj);
      }
    }

    // Facts: keep facts that do not mention robots outside this team.
    for (const auto &fact : problem.get_facts()) {
      bool mentions_other = false;
      for (const auto &arg : fact.get_args()) {
        if (all_robot_names.count(arg) > 0 && team_robots.count(arg) == 0) {
          mentions_other = true;
          break;
        }
      }
      if (!mentions_other) {
        sub_problem.add_fact(fact);
      }
    }

    // Goals: add only the allocated goal indices, filtering out any that
    // reference robots outside the team (safety guard).
    for (int idx : team.goal_indices) {
      if (idx < 0 || static_cast<size_t>(idx) >= goals_vec.size()) {
        continue;
      }
      const auto &goal = goals_vec[static_cast<size_t>(idx)];
      bool mentions_other = false;
      for (const auto &arg : goal.get_args()) {
        if (all_robot_names.count(arg) > 0 && team_robots.count(arg) == 0) {
          mentions_other = true;
          break;
        }
      }
      if (!mentions_other) {
        sub_problem.add_goal(goal);
      }
    }

    if (sub_problem.get_goals().empty()) {
      continue;
    }

    // Build a human-readable team label for logging.
    std::string label;
    for (size_t k = 0; k < team.robots.size(); ++k) {
      if (k > 0) {
        label += '+';
      }
      label += team.robots[k];
    }

    RCLCPP_INFO(this->node_->get_logger(),
                "Team [%s]: %zu goals — launching sub-planner", label.c_str(),
                team.goal_indices.size());

    team_labels.push_back(label);
    futures.push_back(
        std::async(std::launch::async, [this, &domain, sub_problem]() {
          return this->sub_planner_->generate_plan(domain, sub_problem);
        }));
  }

  std::vector<omni_plan::pddl::Plan> successful_plans;
  for (size_t i = 0; i < futures.size(); ++i) {
    omni_plan::pddl::Plan sub_plan = futures[i].get();
    if (sub_plan.has_solution()) {
      successful_plans.push_back(sub_plan);
      RCLCPP_INFO(this->node_->get_logger(), "Team [%s]: plan found",
                  team_labels[i].c_str());
    } else {
      RCLCPP_WARN(this->node_->get_logger(), "Team [%s]: no solution found",
                  team_labels[i].c_str());
    }
  }

  if (successful_plans.empty()) {
    RCLCPP_ERROR(this->node_->get_logger(), "No sub-planner found a solution");
    return empty_plan;
  }

  omni_plan::pddl::Plan merged = this->merge_plans(successful_plans);

  RCLCPP_INFO(this->node_->get_logger(), "Merged plan from %zu sub-plans: %s",
              successful_plans.size(), merged.to_pddl().c_str());

  return merged;
}

omni_plan::pddl::Plan
MrtaPlanner::parse_plan(const omni_plan::pddl::Domain &domain,
                        const std::string &str_plan) const {
  if (!this->sub_planner_) {
    throw std::runtime_error("MrtaPlanner: no sub_planner loaded");
  }
  return this->sub_planner_->parse_plan(domain, str_plan);
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(omni_plan_mrta::MrtaPlanner, omni_plan::Planner)
