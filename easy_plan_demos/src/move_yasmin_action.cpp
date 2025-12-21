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

#include <algorithm>
#include <chrono>
#include <iostream>
#include <thread>

#include "yasmin/blackboard.hpp"
#include "yasmin/cb_state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

#include "easy_plan/pddl/action.hpp"
#include "easy_plan_yasmin/yasmin_action.hpp"

using namespace easy_plan;

class MoveYasminAction : public easy_plan_yasmin::YasminAction {
public:
  MoveYasminAction()
      : YasminAction("move", {
                                 {"robot", "robot"},
                                 {"r1", "room"},
                                 {"r2", "room"},
                             }) {

    this->add_condition(pddl::START, "robot_at",
                        std::vector<std::string>{"robot", "r1"});
    this->add_condition(pddl::OVER_ALL, "battery_full",
                        std::vector<std::string>{"robot"});
    this->add_condition(pddl::START, "connected",
                        std::vector<std::string>{"r1", "r2"});

    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r1"}, true);
    this->add_effect(pddl::END, "robot_at",
                     std::vector<std::string>{"robot", "r2"});

    this->add_parameters({
        {"increment", 0.05f, this->increment_},
    });

    // Create the states
    auto start_state = std::make_shared<yasmin::CbState>(
        yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED},
        [](yasmin::Blackboard::SharedPtr bb) -> std::string {
          std::string robot = bb->get<std::string>("robot");
          std::string r1 = bb->get<std::string>("r1");
          std::string r2 = bb->get<std::string>("r2");
          std::cout << "Moving " << robot << " from " << r1 << " to " << r2
                    << std::endl;
          bb->set<float>("progress", 0.0f);
          return yasmin_ros::basic_outcomes::SUCCEED;
        });

    auto check_progress_state = std::make_shared<yasmin::CbState>(
        yasmin::Outcomes{"continue", "finish"},
        [](yasmin::Blackboard::SharedPtr bb) -> std::string {
          float progress = bb->get<float>("progress");
          if (progress < 1.0f) {
            return "continue";
          } else {
            return "finish";
          }
        });

    auto increase_progress_state = std::make_shared<yasmin::CbState>(
        yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED},
        [this](yasmin::Blackboard::SharedPtr bb) -> std::string {
          float progress = bb->get<float>("progress");
          progress += this->increment_;
          bb->set<float>("progress", progress);
          return yasmin_ros::basic_outcomes::SUCCEED;
        });

    auto print_progress_state = std::make_shared<yasmin::CbState>(
        yasmin::Outcomes{yasmin_ros::basic_outcomes::SUCCEED},
        [](yasmin::Blackboard::SharedPtr bb) -> std::string {
          float progress = bb->get<float>("progress");
          std::cout << "Moving robot ... ["
                    << std::min(100.0f, progress * 100.0f) << "%]  "
                    << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(200));
          return yasmin_ros::basic_outcomes::SUCCEED;
        });

    this->add_state(
        "STARTING", start_state,
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "CHECKING_PROGRESS"},
        });
    this->add_state("CHECKING_PROGRESS", check_progress_state,
                    {
                        {"continue", "INCREASING_PROGRESS"},
                        {"finish", yasmin_ros::basic_outcomes::SUCCEED},
                    });
    this->add_state(
        "INCREASING_PROGRESS", increase_progress_state,
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "PRINTING_PROGRESS"},
        });
    this->add_state(
        "PRINTING_PROGRESS", print_progress_state,
        {
            {yasmin_ros::basic_outcomes::SUCCEED, "CHECKING_PROGRESS"},
        });
  }

private:
  /// @brief Increment per iteration.
  float increment_ = 0.05;
};

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(MoveYasminAction, easy_plan::pddl::Action)