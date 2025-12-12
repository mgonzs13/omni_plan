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

#ifndef EASY_PLAN__PLAN_HPP__
#define EASY_PLAN__PLAN_HPP__

#include <memory>
#include <string>
#include <vector>

#include "easy_plan/action.hpp"

namespace easy_plan {

class Plan {
public:
  Plan(bool has_solution = false);

  bool has_solution() const;

  void add_action(const std::shared_ptr<Action> &action,
                  const std::vector<std::string> &params = {});

  bool get_next_action(std::shared_ptr<Action> &action,
                       std::vector<std::string> &params);

private:
  bool has_solution_ = false;
  std::vector<std::shared_ptr<Action>> actions_;
  std::vector<std::vector<std::string>> params_;
  mutable size_t current_index_ = 0;
};

} // namespace easy_plan
#endif // EASY_PLAN__PLAN_HPP__