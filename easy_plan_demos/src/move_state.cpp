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

#include <chrono>
#include <iostream>
#include <thread>

#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

class MoveState : public yasmin::State {
public:
  MoveState()
      : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}), progress_(0.0f),
        increment_(0.05f) {}

  std::string execute(yasmin::Blackboard::SharedPtr bb) override {
    std::string robot = bb->get<std::string>("robot");
    std::string r1 = bb->get<std::string>("r1");
    std::string r2 = bb->get<std::string>("r2");
    std::cout << "Moving " << robot << " from " << r1 << " to " << r2
              << " using YasminAction" << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += this->increment_;
      std::cout << "Moving robot ... ["
                << std::min(100.0f, this->progress_ * 100.0f) << "%]"
                << std::endl;
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    this->progress_ = 0.0f;

    return yasmin_ros::basic_outcomes::SUCCEED;
  }

private:
  float progress_;
  float increment_;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MoveState, yasmin::State)