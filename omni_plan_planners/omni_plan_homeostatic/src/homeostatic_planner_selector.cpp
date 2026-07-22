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
#include <limits>
#include <sstream>
#include <string>

#include "omni_plan_homeostatic/homeostatic_planner_selector.hpp"

using namespace omni_plan_homeostatic;

HomeostaticPlannerSelector::HomeostaticPlannerSelector(double exploration_prob,
                                                       double decay_rate,
                                                       double min_exploration)
    : exploration_prob_(exploration_prob), decay_rate_(decay_rate),
      min_exploration_(min_exploration), total_calls_(0),
      rng_(std::random_device{}()) {}

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

  // Decay exploration probability over time
  double current_eps = this->exploration_prob_ *
                       std::pow(this->decay_rate_,
                                static_cast<double>(this->total_calls_) / 10.0);
  current_eps = std::max(current_eps, this->min_exploration_);

  // Exploration: pick a random planner
  std::uniform_real_distribution<double> dist(0.0, 1.0);
  if (dist(this->rng_) < current_eps) {
    std::uniform_int_distribution<size_t> pick(0, this->planners_.size() - 1);
    size_t idx = pick(this->rng_);
    auto it = this->planners_.begin();
    std::advance(it, idx);
    out_planner_name = it->first;
    if (out_reason) {
      *out_reason =
          "exploration (eps=" + std::to_string(current_eps) + ", random pick)";
    }
    return it->second;
  }

  // Exploitation: pick the cheapest planner for this hash
  auto hash_it = this->cost_table_.find(hash_key);
  if (hash_it != this->cost_table_.end() && !hash_it->second.empty()) {
    std::string best_planner;
    double best_cost = std::numeric_limits<double>::max();
    for (const auto &[name, record] : hash_it->second) {
      if (record.times_selected > 0) {
        double avg_cost =
            record.total_cost / static_cast<double>(record.times_selected);
        if (avg_cost < best_cost) {
          best_cost = avg_cost;
          best_planner = name;
        }
      }
    }
    if (!best_planner.empty()) {
      out_planner_name = best_planner;
      if (out_reason) {
        *out_reason = "exploitation (best avg for hash, cost=" +
                      std::to_string(best_cost) + ")";
      }
      return this->planners_.at(best_planner);
    }
  }

  // No data for this hash yet: pick the planner with lowest overall average
  std::string best_planner;
  double best_cost = std::numeric_limits<double>::max();
  for (const auto &[name, planner] : this->planners_) {
    double total = 0.0;
    size_t count = 0;
    for (const auto &[hkey, records] : this->cost_table_) {
      auto rec_it = records.find(name);
      if (rec_it != records.end()) {
        total += rec_it->second.total_cost;
        count += rec_it->second.times_selected;
      }
    }
    if (count > 0) {
      double avg = total / static_cast<double>(count);
      if (avg < best_cost) {
        best_cost = avg;
        best_planner = name;
      }
    }
  }

  if (!best_planner.empty()) {
    out_planner_name = best_planner;
    if (out_reason) {
      *out_reason =
          "exploitation (best global avg, cost=" + std::to_string(best_cost) +
          ")";
    }
    return this->planners_.at(best_planner);
  }

  // Fallback: pick the first registered planner
  out_planner_name = this->planners_.begin()->first;
  if (out_reason) {
    *out_reason = "fallback (no cost data, first planner)";
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
