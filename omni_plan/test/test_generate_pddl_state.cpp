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

#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <utility>

#include <pluginlib/class_loader.hpp>

#include "rclcpp/rclcpp.hpp"

#include "yasmin/blackboard.hpp"
#include "yasmin/state.hpp"
#include "yasmin_ros/basic_outcomes.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"

using namespace omni_plan;

namespace {

/// Pddl manager stub that can produce a problem with or without goals.
class GoalLessPddlManager : public PddlManager {
public:
  explicit GoalLessPddlManager(bool with_goal) : with_goal_(with_goal) {}

  std::pair<pddl::Domain, pddl::Problem> get_pddl() const override {
    pddl::Problem problem;
    if (this->with_goal_) {
      problem.add_goal(pddl::Predicate("at", {"robot1", "room1"}));
    }
    return {pddl::Domain(), problem};
  }

  bool has_goals() const override { return this->with_goal_; }
  bool clear_goals() const override { return true; }
  bool predicate_exists(const pddl::Predicate & /*predicate*/) const override {
    return false;
  }
  bool predicate_is_goal(const pddl::Predicate & /*predicate*/) const override {
    return false;
  }
  void apply_effect(const pddl::Effect & /*effect*/) override {}

private:
  bool with_goal_;
};

class GeneratePddlStateTest : public ::testing::Test {
protected:
  void SetUp() override {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
    // The state plugins are exported through the yasmin plugin index.
    this->loader_ = std::make_unique<pluginlib::ClassLoader<yasmin::State>>(
        "yasmin", "yasmin::State");
  }

  void TearDown() override { this->loader_.reset(); }

  std::string run_state(bool with_goal) {
    auto state =
        this->loader_->createSharedInstance("omni_plan/GeneratePddlState");

    auto blackboard = std::make_shared<yasmin::Blackboard>();
    blackboard->set<std::shared_ptr<PddlManager>>(
        "pddl_manager", std::make_shared<GoalLessPddlManager>(with_goal));
    std::unordered_map<std::string, std::shared_ptr<pddl::Action>> actions;
    blackboard
        ->set<std::unordered_map<std::string, std::shared_ptr<pddl::Action>>>(
            "actions", actions);

    return state->execute(blackboard);
  }

  std::unique_ptr<pluginlib::ClassLoader<yasmin::State>> loader_;
};

} // namespace

// A goal-less problem must not be planned: the empty plan would abort the
// plan state, whose abort transition re-enters the PDDL generation without
// checking the goals again (an infinite reasoning loop / replanning storm).
// The state must abort instead, so the state machine returns to IDLE.
TEST_F(GeneratePddlStateTest, GoalLessProblemAborts) {
  EXPECT_EQ(this->run_state(false), yasmin_ros::basic_outcomes::ABORT);
}

TEST_F(GeneratePddlStateTest, ProblemWithGoalsSucceeds) {
  EXPECT_EQ(this->run_state(true), yasmin_ros::basic_outcomes::SUCCEED);
}
