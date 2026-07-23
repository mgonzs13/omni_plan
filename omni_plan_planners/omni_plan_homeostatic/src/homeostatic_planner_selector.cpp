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
#include <cmath>
#include <limits>
#include <sstream>
#include <string>

#include "omni_plan_homeostatic/homeostatic_planner_selector.hpp"

using namespace omni_plan_homeostatic;

HomeostaticPlannerSelector::HomeostaticPlannerSelector(
    double ucb_exploration_constant)
    : ucb_exploration_constant_(ucb_exploration_constant), total_calls_(0) {}

void HomeostaticPlannerSelector::add_planner(
    const std::string &name, std::shared_ptr<omni_plan::Planner> planner) {
  this->planners_[name] = planner;
}

std::shared_ptr<omni_plan::Planner>
HomeostaticPlannerSelector::select_planner(const std::string &hash_key,
                                           std::string &out_planner_name,
                                           std::string *out_reason) {

  std::lock_guard<std::mutex> lock(this->selector_mutex_);
  this->total_calls_++;

  auto hash_it = this->cost_table_.find(hash_key);

  // ---- UCB selection per hash ----
  if (hash_it != this->cost_table_.end() && !hash_it->second.empty()) {
    size_t N = 0;
    for (const auto &[name, record] : hash_it->second) {
      N += record.times_selected;
    }

    std::string best_planner;
    double best_score = std::numeric_limits<double>::max();

    for (const auto &[name, planner] : this->planners_) {
      auto rec_it = hash_it->second.find(name);
      size_t times_selected = 0;
      double total_cost = 0;

      if (rec_it != hash_it->second.end()) {
        times_selected = rec_it->second.times_selected;
        total_cost = rec_it->second.total_cost;
      }

      if (times_selected == 0) {
        out_planner_name = name;
        if (out_reason) {
          *out_reason = "ucb (n_i=0, first trial)";
        }
        return planner;
      }

      double avg_cost = total_cost / static_cast<double>(times_selected);
      double exploration_bonus = this->ucb_exploration_constant_ *
                                 std::sqrt(std::log(static_cast<double>(N)) /
                                           static_cast<double>(times_selected));
      double score = avg_cost - exploration_bonus;

      if (score < best_score) {
        best_score = score;
        best_planner = name;
      }
    }

    if (!best_planner.empty()) {
      out_planner_name = best_planner;
      if (out_reason) {
        *out_reason = "ucb (best score=" + std::to_string(best_score) + ")";
      }
      return this->planners_.at(best_planner);
    }
  }

  // ---- Global UCB fallback ----
  size_t global_N = 0;
  std::unordered_map<std::string, double> global_avg;
  std::unordered_map<std::string, size_t> global_n;
  for (const auto &[hkey, planners] : this->cost_table_) {
    for (const auto &[name, record] : planners) {
      global_N += record.times_selected;
      global_avg[name] += record.total_cost;
      global_n[name] += record.times_selected;
    }
  }
  for (auto &[name, total] : global_avg) {
    if (global_n[name] > 0) {
      total /= static_cast<double>(global_n[name]);
    }
  }

  std::string best_planner;
  double best_score = std::numeric_limits<double>::max();
  for (const auto &[name, planner] : this->planners_) {
    size_t n_i = global_n[name];
    if (n_i == 0) {
      out_planner_name = name;
      if (out_reason) {
        *out_reason = "ucb global (n_i=0, first trial)";
      }
      return planner;
    }
    double exploration_bonus =
        this->ucb_exploration_constant_ *
        std::sqrt(std::log(static_cast<double>(global_N)) /
                  static_cast<double>(n_i));
    double score = global_avg[name] - exploration_bonus;
    if (score < best_score) {
      best_score = score;
      best_planner = name;
    }
  }

  if (!best_planner.empty()) {
    out_planner_name = best_planner;
    if (out_reason) {
      *out_reason =
          "ucb global (best score=" + std::to_string(best_score) + ")";
    }
    return this->planners_.at(best_planner);
  }

  out_planner_name = this->planners_.begin()->first;
  if (out_reason) {
    *out_reason = "fallback (no cost data)";
  }
  return this->planners_.begin()->second;
}

void HomeostaticPlannerSelector::record_observation(
    const std::string &hash_key, const std::string &planner_name, double cost,
    bool succeeded) {

  std::lock_guard<std::mutex> lock(this->selector_mutex_);
  PlannerCostRecord &rec = this->cost_table_[hash_key][planner_name];
  rec.planner_name = planner_name;
  rec.total_cost += cost;
  rec.times_selected++;
  if (succeeded) {
    rec.times_succeeded++;
  }
}

bool HomeostaticPlannerSelector::needs_cold_start(size_t min_steps) const {
  std::lock_guard<std::mutex> lock(this->selector_mutex_);
  for (const auto &[name, planner] : this->planners_) {
    size_t observed = 0;
    for (const auto &[hkey, records] : this->cost_table_) {
      auto rec_it = records.find(name);
      if (rec_it != records.end()) {
        observed += rec_it->second.times_selected;
      }
    }
    if (observed < min_steps) {
      return true;
    }
  }
  return false;
}

const std::map<std::string, std::shared_ptr<omni_plan::Planner>> &
HomeostaticPlannerSelector::get_all_planners() const {
  return this->planners_;
}

std::string HomeostaticPlannerSelector::get_planner_cost_table() const {
  std::lock_guard<std::mutex> lock(this->selector_mutex_);
  std::ostringstream oss;
  oss << "Homeostatic Planner Cost Table:\n";
  for (const auto &[hash, planners] : this->cost_table_) {
    oss << "  hash=" << hash.substr(0, 8) << "...\n";
    for (const auto &[name, record] : planners) {
      double avg =
          record.times_selected > 0
              ? record.total_cost / static_cast<double>(record.times_selected)
              : 0.0;
      oss << "    " << name << ": avg=" << avg
          << ", times=" << record.times_selected
          << ", succ=" << record.times_succeeded << "\n";
    }
  }
  return oss.str();
}
