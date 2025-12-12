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

#ifndef EASY_PLAN__STATES__OUTCOMES_HPP__
#define EASY_PLAN__STATES__OUTCOMES_HPP_

namespace easy_plan {
namespace states {
namespace outcomes {
static const char *const HAS_GOALS = "HAS_GOALS";
static const char *const NO_GOALS = "NO_GOALS";
static const char *const SUCCEED = "SUCCEED";
static const char *const FAILED = "FAILED";
static const char *const CANCELED = "CANCELED";
static const char *const VALID = "VALID";
static const char *const INVALID = "INVALID";
} // namespace outcomes
} // namespace states
} // namespace easy_plan

#endif // EASY_PLAN__STATES__OUTCOMES_HPP__