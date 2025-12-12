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

#include <memory>
#include <string>

#include "yasmin/state.hpp"
#include "yasmin_ros/get_parameters_state.hpp"

class GetParamsState : public yasmin_ros::GetParametersState {

public:
  GetParamsState()
      : yasmin_ros::GetParametersState(std::map<std::string, std::any>{
            {"generate_domain_plugin", std::string("generate_domain")},
            {"generate_problem_plugin", std::string("generate_problem")},
            {"planner_plugin", std::string("popf")},
        }) {};
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GetParamsState, yasmin::State)