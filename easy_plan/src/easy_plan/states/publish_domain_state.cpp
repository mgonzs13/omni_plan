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
#include "easy_plan_msgs/msg/domain.hpp"

#include "yasmin_ros/publisher_state.hpp"

using std::placeholders::_1;

class PublishDomainState
    : public yasmin_ros::PublisherState<easy_plan_msgs::msg::Domain> {

public:
  PublishDomainState()
      : yasmin_ros::PublisherState<easy_plan_msgs::msg::Domain>(
            "domain",
            std::bind(&PublishDomainState::create_int_msg, this, _1)) {}

  easy_plan_msgs::msg::Domain
  create_int_msg(yasmin::Blackboard::SharedPtr blackboard) {
    return blackboard->get<easy_plan::pddl::Domain>("domain").to_msg();
  };
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(PublishDomainState, yasmin::State)