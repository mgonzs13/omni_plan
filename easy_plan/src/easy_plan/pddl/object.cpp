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

#include "easy_plan_msgs/msg/object.hpp"

#include "easy_plan/pddl/object.hpp"

using namespace easy_plan::pddl;

Object::Object(const std::string &n, const std::string &t) : name(n), type(t) {}

std::string Object::get_name() const { return this->name; }

std::string Object::get_type() const { return this->type; }

std::string Object::to_pddl() const { return this->name + " - " + this->type; }

easy_plan_msgs::msg::Object Object::to_msg() const {
  easy_plan_msgs::msg::Object obj_msg;
  obj_msg.name = this->name;
  obj_msg.type = this->type;
  return obj_msg;
}
