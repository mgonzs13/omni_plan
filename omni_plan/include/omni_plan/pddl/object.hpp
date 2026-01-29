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

#ifndef OMNI_PLAN__PDDL__OBJECT_HPP_
#define OMNI_PLAN__PDDL__OBJECT_HPP_

#include <string>

#include "omni_plan_msgs/msg/object.hpp"

namespace omni_plan {
namespace pddl {

/**
 * @class Object
 * @brief Represents an object in a PDDL domain.
 * @details Objects are the entities that populate the planning world.
 * Each object has a name and a type, which determines its category and
 * the predicates that can apply to it.
 */
class Object {
public:
  /**
   * @brief Constructs an Object with a given name and type.
   * @param n The name of the object.
   * @param t The type of the object.
   */
  Object(const std::string &n, const std::string &t);

  /**
   * @brief Gets the name of the object.
   * @return The name of the object as a string.
   */
  std::string get_name() const;

  /**
   * @brief Gets the type of the object.
   * @return The type of the object as a string.
   */
  std::string get_type() const;

  /**
   * @brief Less-than comparison operator for ordering objects.
   * @details Objects are ordered first by name, then by type.
   * @param other The other object to compare with.
   * @return True if this object is less than the other, false otherwise.
   */
  bool operator<(const Object &other) const {
    if (this->name != other.name) {
      return this->name < other.name;
    }
    return this->type < other.type;
  }

  /**
   * @brief Equality comparison operator for objects.
   * @details Two objects are considered equal if they have the same name.
   * @param other The other object to compare with.
   * @return True if the objects are equal, false otherwise.
   */
  bool operator==(const Object &other) const {
    return this->name == other.name;
  }

  /**
   * @brief Converts the object to its PDDL representation.
   * @return A string representing the object in PDDL format.
   */
  std::string to_pddl() const;

  /**
   * @brief Converts the object to its ROS message representation.
   * @return A ROS message of type omni_plan_msgs::msg::Object representing the
   * object.
   */
  omni_plan_msgs::msg::Object to_msg() const;

private:
  /// The name of the object.
  std::string name;
  /// The type of the object.
  std::string type;
};

using Parameter = Object;

} // namespace pddl
} // namespace omni_plan

#endif // OMNI_PLAN__PDDL__OBJECT_HPP_
