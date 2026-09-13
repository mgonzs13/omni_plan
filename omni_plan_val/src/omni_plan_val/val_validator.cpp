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

#include <sys/wait.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <initializer_list>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "omni_plan/utils/package_share_path.hpp"
#include "omni_plan/utils/temp_file_guard.hpp"

#include "omni_plan_val/val_validator.hpp"

using namespace omni_plan_val;

namespace {

std::string shell_quote(const std::string &value) {
  std::string quoted = "'";
  for (const char c : value) {
    if (c == '\'') {
      quoted += "'\\''";
    } else {
      quoted += c;
    }
  }
  quoted += "'";
  return quoted;
}

bool is_one_of(const std::string &value,
               std::initializer_list<const char *> allowed) {
  for (const char *candidate : allowed) {
    if (value == candidate) {
      return true;
    }
  }
  return false;
}

} // namespace

ValValidator::ValValidator() : PlanValidator() {
  // Add VAL validator options as parameters
  this->add_ros_parameters(
      {{"tolerance", 0.0f, tolerance_},
       {"robustness_n", 0.0f, robustness_n_},
       {"robustness_p", 0.0f, robustness_p_},
       {"robustness_m", 0, robustness_m_},
       {"robustness_action_p", 0.0f, robustness_action_p_},
       {"robustness_pne_n", 0.0f, robustness_pne_n_},
       {"robustness_metric", std::string(""), robustness_metric_},
       {"robustness_distribution", std::string(""), robustness_distribution_},
       {"vary_event_preconditions", false, vary_event_preconditions_},
       {"use_graphplan_length", false, use_graphplan_length_},
       {"check_derived_predicates", true, check_derived_predicates_},
       {"continue_on_precondition_fail", false, continue_on_precondition_fail_},
       {"produce_error_report", false, produce_error_report_},
       {"warn_invariants", false, warn_invariants_},
       {"use_makespan_metric", false, use_makespan_metric_},
       {"timeout", 0, timeout_}});
}

bool ValValidator::validate_plan(const std::string &domain_path,
                                 const std::string &problem_path,
                                 const std::string &plan_path) const {

  // Capture diagnostics separately so that only validation output is parsed.
  std::filesystem::path temp_dir = std::filesystem::temp_directory_path();
  std::string err_template = (temp_dir / "val_stderr_XXXXXX").string();
  std::vector<char> err_buffer(err_template.begin(), err_template.end());
  err_buffer.push_back('\0');
  int err_fd = mkstemp(err_buffer.data());
  if (err_fd == -1) {
    std::cerr << "[val] Failed to create temporary stderr file" << std::endl;
    return false;
  }
  close(err_fd);
  std::string err_path(err_buffer.data());
  omni_plan::utils::TempFileGuard err_guard(err_path.c_str());

  // Build command with options
  std::string command =
      shell_quote(omni_plan::utils::get_package_share_path("omni_plan_val") +
                  "/bin/validate");

  if (this->tolerance_ != 0.0f)
    command += " -t " + std::to_string(this->tolerance_);
  if (this->robustness_m_ > 0) {
    command += " -r " + std::to_string(this->robustness_n_) + " " +
               std::to_string(this->robustness_p_) + " " +
               std::to_string(this->robustness_m_);
    if (this->robustness_action_p_ != 0.0f)
      command += " -ra " + std::to_string(this->robustness_action_p_);
    if (this->robustness_pne_n_ != 0.0f)
      command += " -rp " + std::to_string(this->robustness_pne_n_);
    if (is_one_of(this->robustness_metric_, {"m", "a", "d"})) {
      command += " -rm " + shell_quote(this->robustness_metric_);
    } else if (!this->robustness_metric_.empty()) {
      std::cerr << "[val] Ignoring invalid robustness_metric parameter: "
                << this->robustness_metric_ << std::endl;
    }
    if (is_one_of(this->robustness_distribution_, {"u", "n", "p"})) {
      command += " -rd " + shell_quote(this->robustness_distribution_);
    } else if (!this->robustness_distribution_.empty()) {
      std::cerr << "[val] Ignoring invalid robustness_distribution parameter: "
                << this->robustness_distribution_ << std::endl;
    }
  }
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

  command += " " + shell_quote(domain_path) + " " + shell_quote(problem_path) +
             " " + shell_quote(plan_path);
  command += " 2> " + shell_quote(err_path);

  if (this->timeout_ > 0)
    command = "timeout -k 5 " + std::to_string(this->timeout_) + " " + command;

  FILE *pipe = popen(command.c_str(), "r");
  if (!pipe) {
    return false;
  }

  std::string output;
  char buffer[128];
  while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
    output += buffer;
  }

  int status = pclose(pipe);
  bool exited_normally = status != -1 && WIFEXITED(status);
  int exit_code = exited_normally ? WEXITSTATUS(status) : -1;

  std::ifstream err_stream(err_path);
  std::string err_output((std::istreambuf_iterator<char>(err_stream)),
                         std::istreambuf_iterator<char>());

  if (!exited_normally) {
    std::cerr << "[val] VAL terminated abnormally";
    if (!err_output.empty()) {
      std::cerr << ": " << err_output;
    } else {
      std::cerr << std::endl;
    }
    return false;
  }

  // Robustness mode reports "<n> plan(s) are valid from <m> plans".
  if (this->robustness_m_ > 0) {
    std::istringstream iss(output);
    std::string line;
    while (std::getline(iss, line)) {
      if (line.find("plans are valid from") != std::string::npos ||
          line.find("plan is valid from") != std::string::npos) {
        std::istringstream line_stream(line);
        int num_valid = 0;
        if (line_stream >> num_valid) {
          return num_valid > 0;
        }
        std::cerr << "[val] Failed to parse robustness result: " << line
                  << std::endl;
        return false;
      }
    }
    if (exit_code != 0 && !err_output.empty()) {
      std::cerr << "[val] VAL error: " << err_output;
    }
    return false;
  }

  // Normal validation mode.
  if (output.find("Plan valid") != std::string::npos) {
    return true;
  }
  if (output.find("Plan invalid") != std::string::npos ||
      output.find("Plan failed") != std::string::npos ||
      output.find("Failed plans") != std::string::npos) {
    return false;
  }

  if (exit_code != 0 && !err_output.empty()) {
    std::cerr << "[val] VAL error: " << err_output;
  }
  return exit_code == 0;
}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(ValValidator, omni_plan::PlanValidator)