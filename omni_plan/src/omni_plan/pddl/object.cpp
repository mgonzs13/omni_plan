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

#include "omni_plan_msgs/msg/object.hpp"

#include "omni_plan/pddl/object.hpp"

using namespace omni_plan::pddl;

Object::Object(const std::string &n, const std::string &t) : name(n), type(t) {}

std::string Object::get_name() const { return this->name; }

std::string Object::get_type() const { return this->type; }

std::string Object::to_pddl() const { return this->name + " - " + this->type; }

omni_plan_msgs::msg::Object Object::to_msg() const {
  omni_plan_msgs::msg::Object obj_msg;
  obj_msg.name = this->name;
  obj_msg.type = this->type;
  return obj_msg;
}
