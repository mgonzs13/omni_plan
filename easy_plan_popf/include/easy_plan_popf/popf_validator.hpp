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

#ifndef EASY_PLAN__POPF_VALIDATOR_HPP__
#define EASY_PLAN__POPF_VALIDATOR_HPP__

#include <memory>
#include <string>

#include "easy_plan/plan.hpp"
#include "easy_plan/plan_validator.hpp"

namespace easy_plan_popf {

class PopfValidator : public easy_plan::PlanValidator {
public:
  PopfValidator();

  bool validate_plan(const std::string &domain, const std::string &problem,
                     easy_plan::Plan plan) const override;

  std::string plan_to_string(easy_plan::Plan plan) const;

private:
};

} // namespace easy_plan_popf
#endif // EASY_PLAN__POPF_VALIDATOR_HPP__