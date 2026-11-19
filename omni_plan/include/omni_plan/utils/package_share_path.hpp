// Copyright (C) 2026 Miguel Ángel González Santamarta
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
