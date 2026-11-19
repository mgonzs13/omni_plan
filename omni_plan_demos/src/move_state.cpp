// Copyright (C) 2025 Miguel Ángel González Santamarta
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

#include <chrono>
#include <iostream>
#include <thread>

#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

class MoveState : public yasmin::State {
public:
  MoveState()
      : yasmin::State({yasmin_ros::basic_outcomes::SUCCEED}), progress_(0.0f) {}

  std::string execute(yasmin::Blackboard::SharedPtr blackboard) override {
    std::string robot = blackboard->get<std::string>("robot");
    std::string r1 = blackboard->get<std::string>("r1");
    std::string r2 = blackboard->get<std::string>("r2");
    std::cout << "Moving " << robot << " from " << r1 << " to " << r2
              << " using YasminAction" << std::endl;

    while (this->progress_ < 1.0) {
      this->progress_ += blackboard->get<float>("increment");
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
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MoveState, yasmin::State)