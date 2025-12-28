

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

#ifndef EASY_PLAN__UTILS__PARAMETER_LOADER_HPP_
#define EASY_PLAN__UTILS__PARAMETER_LOADER_HPP_

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <vector>

#include "rclcpp/parameter_value.hpp"
#include "rclcpp/rclcpp.hpp"

namespace easy_plan {
namespace utils {

/**
 * @class ParameterLoader
 * @brief Base class for loading parameters.
 * @details This abstract base class defines the interface for components that
 * load parameters from a specified namespace. Derived classes should implement
 * the logic to retrieve and manage parameters as needed.
 */
class ParameterLoader {
public:
  /**
   * @brief Constructor with namespace.
   */
  ParameterLoader(const std::string &namespace_) : namespace_(namespace_) {};

  /**
   * @brief Default destructor.
   */
  virtual ~ParameterLoader() = default;

  /**
   * @struct ParameterInfo
   * @brief Structure to hold parameter information.
   */
  struct ParameterInfo {
    std::string name;
    rclcpp::ParameterValue default_value;
    std::function<void(rclcpp::ParameterValue)> setter;
    mutable bool declared = false;

    template <typename T>
    ParameterInfo(const std::string &name, const T &default_val, T &output)
        : name(name), default_value(rclcpp::ParameterValue(default_val)),
          setter([&output](rclcpp::ParameterValue val) {
            output = val.get<T>();
          }) {}
  };

  /**
   * @brief Add a parameter to the loader.
   * @param name The parameter name.
   * @param default_val The default value.
   * @param output The output variable reference.
   */
  template <typename T>
  void add_ros_parameter(const std::string &name, const T &default_val,
                         T &output) {
    this->params_.emplace_back(name, default_val, output);
  }

  /**
   * @brief Add multiple parameters to the loader at once.
   * @param params The parameters to add, as { {name, default_value, output},
   * ... }.
   */
  void add_ros_parameters(std::initializer_list<ParameterInfo> params) {
    for (const auto &param : params) {
      params_.emplace_back(param);
    }
  }

  /**
   * @brief Declare all parameters on the ROS 2 node.
   * @param node The ROS 2 node to declare parameters on.
   */
  void declare_ros_parameters(rclcpp::Node::SharedPtr node) const {
    for (const auto &param : this->params_) {
      if (!param.declared) {
        std::string full_name = this->namespace_ + "." + param.name;
        node->declare_parameter(full_name, param.default_value);
        param.declared = true;
      }
    }
  }

  /**
   * @brief Add a callback to be called after loading parameters.
   * @param cb The callback function.
   */
  void add_load_ros_parameters_callback(std::function<void()> cb) {
    this->callbacks_.push_back(cb);
  }

  /**
   * @brief Get all parameters from the ROS 2 node.
   * @param node The ROS 2 node to get parameters from.
   */
  void load_ros_parameters(rclcpp::Node::SharedPtr node) const {
    this->declare_ros_parameters(node);

    for (const auto &param : this->params_) {
      this->load_single_ros_parameter(node, param);
    }

    for (const auto &cb : this->callbacks_) {
      cb();
    }
  }

private:
  void load_single_ros_parameter(rclcpp::Node::SharedPtr node,
                                 const ParameterInfo &param) const {
    std::string full_name = this->namespace_ + "." + param.name;
    rclcpp::ParameterValue val;
    if (node->get_parameter(full_name, val)) {
      param.setter(val);
    } else {
      param.setter(param.default_value);
    }
  }

private:
  /// @brief Namespace from which to load parameters.
  std::string namespace_;
  /// @brief Stored parameters.
  std::vector<ParameterInfo> params_;
  /// @brief Callbacks to be called after loading parameters.
  std::vector<std::function<void()>> callbacks_;
};

} // namespace utils
} // namespace easy_plan

#endif // EASY_PLAN__UTILS__PARAMETER_LOADER_HPP_