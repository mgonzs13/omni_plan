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

#ifndef EASY_PLAN__ACTION_HPP__
#define EASY_PLAN__ACTION_HPP__

#include <string>
#include <vector>

namespace easy_plan {

class Action {
public:
  Action(const std::string &name, const std::vector<std::string> &params = {})
      : name_(name), parameters_(params), start_condition_(""),
        over_condition_(""), end_condition_(""), start_effect_(""),
        end_effect_("") {};

  virtual ~Action() = default;

  std::string get_name() const { return this->name_; };

  std::string to_pddl() const {
    std::string pddl = "(:durative-action " + this->name_ + "\n";
    pddl += "  :parameters (";
    for (size_t i = 0; i < this->parameters_.size(); ++i) {
      pddl += this->parameters_[i];
      if (i < this->parameters_.size() - 1)
        pddl += " ";
    }
    pddl += ")\n";
    pddl += "  :duration (= ?duration 10)\n";
    pddl += "  :condition (and";
    if (!this->start_condition_.empty())
      pddl += " (at start " + this->start_condition_ + ")";
    if (!this->over_condition_.empty())
      pddl += " (over all " + this->over_condition_ + ")";
    if (!this->end_condition_.empty())
      pddl += " (at end " + this->end_condition_ + ")";
    pddl += ")\n";
    pddl += "  :effect (and";
    if (!this->start_effect_.empty())
      pddl += " (at start " + this->start_effect_ + ")";
    if (!this->end_effect_.empty())
      pddl += " (at end " + this->end_effect_ + ")";
    pddl += ")\n";
    pddl += ")";
    return pddl;
  };

  virtual void run() = 0;

private:
  std::string name_;
  std::vector<std::string> parameters_;
  std::string start_condition_;
  std::string over_condition_;
  std::string end_condition_;
  std::string start_effect_;
  std::string end_effect_;
};

} // namespace easy_plan
#endif // EASY_PLAN__ACTION_HPP__