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

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_msgs/msg/pddl.hpp"

#include "yasmin_ros/publisher_state.hpp"

using std::placeholders::_1;

class PublishPDDLState
    : public yasmin_ros::PublisherState<omni_plan_msgs::msg::PDDL> {

public:
  PublishPDDLState()
      : yasmin_ros::PublisherState<omni_plan_msgs::msg::PDDL>(
            "pddl", std::bind(&PublishPDDLState::create_pddl_msg, this, _1)) {
    this->set_description("Publish the PDDL domain, problem and plan.");
    this->add_input_key("domain", "The PDDL domain.");
    this->add_input_key("problem", "The PDDL problem.");
    this->add_input_key("plan", "The PDDL plan.");
  }

  omni_plan_msgs::msg::PDDL
  create_pddl_msg(const yasmin::Blackboard::SharedPtr &blackboard) {
    omni_plan_msgs::msg::PDDL msg;

    try {
      msg.domain = blackboard->get<omni_plan::pddl::Domain>("domain").to_msg();
      msg.problem =
          blackboard->get<omni_plan::pddl::Problem>("problem").to_msg();
      msg.plan = blackboard->get<omni_plan::pddl::Plan>("plan").to_msg();
    } catch (const std::exception &e) {
      YASMIN_LOG_ERROR("Failed to get PDDL data from blackboard: %s", e.what());
    }

    return msg;
  };
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PublishPDDLState, yasmin::State)