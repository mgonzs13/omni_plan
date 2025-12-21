// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <unistd.h>

#include <fstream>
#include <iomanip>
#include <string>

#include "easy_plan_popf/popf_validator.hpp"

using namespace easy_plan_popf;

PopfValidator::PopfValidator() : PlanValidator() {
  // Add POPF validator options as parameters
  this->add_parameters(
      {{"tolerance", 0.0f, tolerance_},
       {"robustness_n", 0.0f, robustness_n_},
       {"robustness_p", 0.0f, robustness_p_},
       {"robustness_m", 0, robustness_m_},
       {"robustness_action_p", 0.0f, robustness_action_p_},
       {"robustness_pne_n", 0.0f, robustness_pne_n_},
       {"robustness_metric", std::string("m"), robustness_metric_},
       {"robustness_distribution", std::string("u"), robustness_distribution_},
       {"vary_event_preconditions", false, vary_event_preconditions_},
       {"use_graphplan_length", false, use_graphplan_length_},
       {"check_derived_predicates", true, check_derived_predicates_},
       {"continue_on_precondition_fail", false, continue_on_precondition_fail_},
       {"produce_error_report", false, produce_error_report_},
       {"warn_invariants", false, warn_invariants_},
       {"use_makespan_metric", false, use_makespan_metric_}});
}

std::string PopfValidator::parse_pddl(const easy_plan::pddl::Plan &plan) const {

  std::ostringstream oss;

  for (size_t i = 0; i < plan.size(); ++i) {
    auto [action, params] = plan.get_action_with_params(i);
    double start_time = i * 20.0;
    oss << std::fixed << std::setprecision(3) << start_time << ": ("
        << action->get_name();
    for (const auto &param : params) {
      oss << " " << param;
    }
    oss << ")  [10.000]\n";
  }

  return oss.str();
}

bool PopfValidator::validate_plan(const std::string &domain_path,
                                  const std::string &problem_path,
                                  const std::string &plan_path) const {

  // Build command with options
  std::string command = "ros2 run popf validate";
  if (this->tolerance_ != 0.0f)
    command += " -t <" + std::to_string(this->tolerance_) + ">";
  if (this->robustness_m_ > 0)
    command += " -r <" + std::to_string(this->robustness_n_) + " " +
               std::to_string(this->robustness_p_) + " " +
               std::to_string(this->robustness_m_) + ">";
  if (this->robustness_action_p_ != 0.0f)
    command += " -ra <" + std::to_string(this->robustness_action_p_) + ">";
  if (this->robustness_pne_n_ != 0.0f)
    command += " -rp <" + std::to_string(this->robustness_pne_n_) + ">";
  if (!this->robustness_metric_.empty())
    command += " -rm <" + this->robustness_metric_ + ">";
  if (!this->robustness_distribution_.empty())
    command += " -rd <" + this->robustness_distribution_ + ">";
  if (this->vary_event_preconditions_)
    command += " -j";
  if (this->use_graphplan_length_)
    command += " -g";
  if (!this->check_derived_predicates_)
    command += " -d";
  if (this->continue_on_precondition_fail_)
    command += " -c";
  if (this->produce_error_report_)
    command += " -e";
  if (this->warn_invariants_)
    command += " -i";
  if (this->use_makespan_metric_)
    command += " -m";

  command += " " + domain_path + " " + problem_path + " " + plan_path + " 2>&1";

  FILE *pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return false;
  }

  // Read and discard output to prevent broken pipe errors
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    // Optionally log output for debugging
    // std::cout << buffer;
  }

  int status = pclose(pipe);
  return status == 0;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PopfValidator, easy_plan::PlanValidator)