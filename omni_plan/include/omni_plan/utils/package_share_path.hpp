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

#ifndef OMNI_PLAN__UTILS__PACKAGE_SHARE_PATH_HPP_
#define OMNI_PLAN__UTILS__PACKAGE_SHARE_PATH_HPP_

#include <string>

#if __has_include("rclcpp/version.h")
#include "rclcpp/version.h"
#if RCLCPP_VERSION_GTE(32, 0, 0)
#include <ament_index_cpp/get_package_share_path.hpp>
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif
#else
#include <ament_index_cpp/get_package_share_directory.hpp>
#endif

namespace omni_plan {
namespace utils {

inline std::string get_package_share_path(const std::string &package) {
#if __has_include("rclcpp/version.h")
#if RCLCPP_VERSION_GTE(32, 0, 0)
  return ament_index_cpp::get_package_share_path(package).string();
#else
  return ament_index_cpp::get_package_share_directory(package);
#endif
#else
  return ament_index_cpp::get_package_share_directory(package);
#endif
}

} // namespace utils
} // namespace omni_plan

#endif // OMNI_PLAN__UTILS__PACKAGE_SHARE_PATH_HPP_
