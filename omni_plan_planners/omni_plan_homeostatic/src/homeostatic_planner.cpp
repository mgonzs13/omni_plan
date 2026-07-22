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

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>

#include "poirot/poirot.hpp"
#include "poirot_msgs/msg/data.hpp"

#include "omni_plan_homeostatic/homeostatic_planner.hpp"

using namespace omni_plan_homeostatic;

HomeostaticPlanner::HomeostaticPlanner() : CachePlanner() {

  this->add_ros_parameters({
      {"planner_plugins",
       std::vector<std::string>(
           {"popf_planner", "smtp_planner", "vhpop_planner"}),
       this->planner_plugins_},
      {"exploration_prob", 0.3, this->exploration_prob_},
      {"decay_rate", 0.95, this->decay_rate_},
      {"min_exploration", 0.05, this->min_exploration_},
      {"selection_field", std::string("wall_time_us"), this->selection_field_},
      {"enable_cache", false, this->enable_cache_},
  });

  this->add_loaded_params_callback([this]() {
    this->selector_ = std::make_shared<HomeostaticPlannerSelector>(
        this->exploration_prob_, this->decay_rate_, this->min_exploration_);

    for (const auto &short_name : this->planner_plugins_) {
      try {
        auto plugin_param = "planner." + short_name + ".plugin";
        if (!this->node_->has_parameter(plugin_param)) {
          this->node_->declare_parameter(
              plugin_param, rclcpp::ParameterValue(std::string("")));
        }

        std::string plugin_class;
        this->node_->get_parameter(plugin_param, plugin_class);

        auto planner =
            this->planner_loader_->createSharedInstance(plugin_class);

        planner->set_namespace("planner." + short_name);
        planner->load_ros_parameters(this->node_);

        this->selector_->add_planner(short_name, planner);
        RCLCPP_INFO(this->node_->get_logger(), "Loaded planner plugin: %s",
                    plugin_class.c_str());

      } catch (const std::exception &e) {
        RCLCPP_WARN(this->node_->get_logger(),
                    "Failed to load planner plugin for %s: %s",
                    short_name.c_str(), e.what());
      }
    }

    RCLCPP_INFO(this->node_->get_logger(),
                "Loaded %zu planners via homeostatic selector",
                this->selector_->get_num_planners());

    this->poirot_sub_ =
        this->node_->create_subscription<poirot_msgs::msg::ProfilingData>(
            "poirot/data", rclcpp::QoS(100),
            [this](const poirot_msgs::msg::ProfilingData::SharedPtr msg) {
              if (msg->function.name.rfind("HomeostaticPlanner::", 0) == 0) {
                std::lock_guard<std::mutex> lock(this->poirot_results_mutex_);
                this->poirot_results_[msg->function.name] =
                    msg->function.call.data;
                this->poirot_cv_.notify_all();
              }
            });
  });
}

double HomeostaticPlanner::get_field_from_data(
    const poirot_msgs::msg::Data &data) const {

  if (this->selection_field_ == "wall_time_us") {
    return static_cast<double>(data.wall_time_us);
  } else if (this->selection_field_ == "cpu_time_us") {
    return static_cast<double>(data.cpu_time_us);
  } else if (this->selection_field_ == "total_energy_uj") {
    return data.total_energy_uj;
  } else if (this->selection_field_ == "co2_ug") {
    return data.co2_ug;
  } else if (this->selection_field_ == "mem_kb") {
    return static_cast<double>(data.mem_kb);
  } else if (this->selection_field_ == "ctx_switches") {
    return static_cast<double>(data.ctx_switches);
  }

  return data.total_energy_uj;
}

std::pair<omni_plan::pddl::Plan, double> HomeostaticPlanner::call_sub_planner(
    const std::string &planner_name,
    std::shared_ptr<omni_plan::Planner> planner,
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem) const {

  size_t seq = this->call_seq_++;
  std::string profiler_name =
      "HomeostaticPlanner::" + planner_name + "::" + std::to_string(seq);

  auto &poirot = poirot::Poirot::get_instance();
  poirot.start_profiling(profiler_name, __FILE__, __LINE__);
  auto plan = planner->generate_plan(domain, problem);
  poirot.stop_profiling();

  std::unique_lock<std::mutex> lock(this->poirot_results_mutex_);
  this->poirot_cv_.wait(lock, [this, &profiler_name] {
    return this->poirot_results_.count(profiler_name) > 0;
  });

  double cost =
      this->get_field_from_data(this->poirot_results_.at(profiler_name));
  this->poirot_results_.erase(profiler_name);

  return {std::move(plan), cost};
}

omni_plan::pddl::Plan
HomeostaticPlanner::delegate_plan(const omni_plan::pddl::Domain &domain,
                                  const omni_plan::pddl::Problem &problem,
                                  const std::string &hash_key) const {

  std::string selected_planner_name;
  std::string selection_reason;
  auto planner = this->selector_->select_planner(
      hash_key, selected_planner_name, &selection_reason);
  RCLCPP_INFO(this->node_->get_logger(),
              "Homeostatic selection: %s (reason: %s)",
              selected_planner_name.c_str(), selection_reason.c_str());

  auto [plan, cost] =
      this->call_sub_planner(selected_planner_name, planner, domain, problem);

  RCLCPP_INFO(this->node_->get_logger(), "Planner %s (%s: %.0f)",
              selected_planner_name.c_str(), this->selection_field_.c_str(),
              cost);
  RCLCPP_INFO(this->node_->get_logger(), "%s", plan.get_raw_output().c_str());

  bool succeeded = plan.has_solution();
  this->selector_->record_observation(hash_key, selected_planner_name, cost,
                                      succeeded);
  RCLCPP_INFO(this->node_->get_logger(), "Homeostatic cost table:\n%s",
              this->selector_->get_planner_cost_table().c_str());

  return plan;
}

bool HomeostaticPlanner::should_cache_result(
    const omni_plan::pddl::Plan &plan) const {
  return this->enable_cache_ && plan.has_solution();
}

PLUGINLIB_EXPORT_CLASS(HomeostaticPlanner, omni_plan::Planner)
