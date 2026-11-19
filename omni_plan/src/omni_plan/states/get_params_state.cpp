// Copyright (C) 2025 Miguel Ángel González Santamarta
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <string>
#include <vector>

#include "yasmin/state.hpp"
#include "yasmin_ros/get_parameters_state.hpp"

class GetParamsState : public yasmin_ros::GetParametersState {

public:
  GetParamsState()
      : yasmin_ros::GetParametersState(
            yasmin_ros::GetParametersState::Parameters{
                {"pddl_manager.plugin", std::string("")},
                {"planner.plugin", std::string("")},
                {"plan_validator.plugin", std::string("")},
                {"plan_dispatcher.plugin", std::string("")},
                {"actions_plugins", std::vector<std::string>()},
            }) {
    this->set_description("Get the parameters for the PDDL manager, planner, "
                          "plan validator and actions plugins.");
  };
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(GetParamsState, yasmin::State)