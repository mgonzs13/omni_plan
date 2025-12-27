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

#include "easy_plan/pddl/domain.hpp"
#include "easy_plan/pddl/plan.hpp"
#include "easy_plan/pddl/problem.hpp"
#include "easy_plan_msgs/msg/pddl.hpp"

#include "yasmin_ros/publisher_state.hpp"

using std::placeholders::_1;

class PublishPDDLState
    : public yasmin_ros::PublisherState<easy_plan_msgs::msg::PDDL> {

public:
  PublishPDDLState()
      : yasmin_ros::PublisherState<easy_plan_msgs::msg::PDDL>(
            "pddl", std::bind(&PublishPDDLState::create_int_msg, this, _1)) {}

  easy_plan_msgs::msg::PDDL
  create_int_msg(yasmin::Blackboard::SharedPtr blackboard) {
    easy_plan_msgs::msg::PDDL msg;
    msg.domain = blackboard->get<easy_plan::pddl::Domain>("domain").to_msg();
    msg.problem = blackboard->get<easy_plan::pddl::Problem>("problem").to_msg();
    msg.plan = blackboard->get<easy_plan::pddl::Plan>("plan").to_msg();
    return msg;
  };
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PublishPDDLState, yasmin::State)