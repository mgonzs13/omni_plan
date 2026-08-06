// Copyright (C) 2026 Miguel Ángel González Santamarta
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

#ifndef OMNI_PLAN__UTILS__TEMP_FILE_GUARD_HPP_
#define OMNI_PLAN__UTILS__TEMP_FILE_GUARD_HPP_

#include <cstdio>

namespace omni_plan {
namespace utils {

struct TempFileGuard {
  const char *path;
  TempFileGuard(const char *p) : path(p) {}
  ~TempFileGuard() {
    if (path) {
      std::remove(path);
    }
  }
  TempFileGuard(const TempFileGuard &) = delete;
  TempFileGuard &operator=(const TempFileGuard &) = delete;
};

} // namespace utils
} // namespace omni_plan

#endif // OMNI_PLAN__UTILS__TEMP_FILE_GUARD_HPP_
