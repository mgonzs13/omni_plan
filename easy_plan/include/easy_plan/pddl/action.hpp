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

#include <memory>
#include <string>
#include <vector>

#include "easy_plan/pddl/expression.hpp"

namespace easy_plan {
namespace pddl {

enum ActionStatus { SUCCEEDED, CANCELED, FAILED };

class Action {
public:
  Action(const std::string &name, const std::vector<Parameter> &params = {});

  virtual ~Action() = default;

  std::string get_name() const;

  void add_condition(Condition::Type type, std::shared_ptr<Predicate> pred);

  void add_effect(Effect::Type type, std::shared_ptr<Predicate> pred);

  std::vector<Parameter> get_parameters() const;

  std::vector<Condition> get_conditions() const;

  std::vector<Condition> get_on_start_conditions() const;

  std::vector<Condition> get_on_end_conditions() const;

  std::vector<Condition> get_over_all_conditions() const;

  std::vector<Effect> get_effects() const;

  std::vector<Effect> get_on_start_effects() const;

  std::vector<Effect> get_on_end_effects() const;

  std::vector<Effect> get_over_all_effects() const;

private:
  std::string get_timing(TimingExpression::Type type) const;

  std::string
  build_timing_section(const std::string &section,
                       const std::vector<TimingExpression> &items) const;

public:
  std::string to_pddl() const;

  bool validate_pddl() const;

  virtual ActionStatus run(std::vector<std::string> params) = 0;

  virtual void cancel() = 0;

private:
  std::string name_;
  std::vector<Parameter> parameters_;
  std::vector<Condition> conditions_;
  std::vector<Effect> effects_;
};

} // namespace pddl
} // namespace easy_plan
#endif // EASY_PLAN__ACTION_HPP__