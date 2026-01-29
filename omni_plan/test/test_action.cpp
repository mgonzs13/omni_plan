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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/timing_predicate.hpp"

using namespace omni_plan::pddl;

/**
 * @brief Concrete implementation of Action for testing
 */
class TestAction : public Action {
public:
  TestAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params), run_called_(false), cancel_called_(false),
        last_params_() {}

  ActionStatus run(const std::vector<std::string> &params) override {
    run_called_ = true;
    last_params_ = params;
    return return_status_;
  }

  void cancel() override { cancel_called_ = true; }

  void set_return_status(ActionStatus status) { return_status_ = status; }

  bool run_called_;
  bool cancel_called_;
  std::vector<std::string> last_params_;
  ActionStatus return_status_ = ActionStatus::SUCCEED;
};

// ==================== Action Tests ====================

class ActionTest : public ::testing::Test {
protected:
  void SetUp() override {
    simple_action_ = std::make_shared<TestAction>("simple");
    move_action_ = std::make_shared<TestAction>(
        "move", std::vector<std::pair<std::string, std::string>>{
                    {"r", "robot"}, {"from", "location"}, {"to", "location"}});
  }

  std::shared_ptr<TestAction> simple_action_;
  std::shared_ptr<TestAction> move_action_;
};

TEST_F(ActionTest, GetName) {
  EXPECT_EQ(simple_action_->get_name(), "simple");
  EXPECT_EQ(move_action_->get_name(), "move");
}

TEST_F(ActionTest, GetParametersEmpty) {
  auto params = simple_action_->get_parameters();
  EXPECT_EQ(params.size(), 0u);
}

TEST_F(ActionTest, GetParameters) {
  auto params = move_action_->get_parameters();

  EXPECT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0].get_name(), "r");
  EXPECT_EQ(params[0].get_type(), "robot");
  EXPECT_EQ(params[1].get_name(), "from");
  EXPECT_EQ(params[1].get_type(), "location");
  EXPECT_EQ(params[2].get_name(), "to");
  EXPECT_EQ(params[2].get_type(), "location");
}

TEST_F(ActionTest, GetParameterType) {
  EXPECT_EQ(move_action_->get_parameter_type("r"), "robot");
  EXPECT_EQ(move_action_->get_parameter_type("from"), "location");
  EXPECT_EQ(move_action_->get_parameter_type("to"), "location");
}

TEST_F(ActionTest, GetParameterTypeUnknown) {
  EXPECT_EQ(move_action_->get_parameter_type("unknown"), "unknown_type");
}

TEST_F(ActionTest, GetParameterIndex) {
  EXPECT_EQ(move_action_->get_parameter_index("r"), 0);
  EXPECT_EQ(move_action_->get_parameter_index("from"), 1);
  EXPECT_EQ(move_action_->get_parameter_index("to"), 2);
}

TEST_F(ActionTest, GetParameterIndexNotFound) {
  EXPECT_EQ(move_action_->get_parameter_index("unknown"), -1);
}

TEST_F(ActionTest, AddCondition) {
  move_action_->add_condition(Type::START, "at", {"r", "from"});
  move_action_->add_condition(Type::START, "connected", {"from", "to"});

  auto conditions = move_action_->get_conditions();
  EXPECT_EQ(conditions.size(), 2u);
}

TEST_F(ActionTest, AddConditionNegated) {
  move_action_->add_condition(Type::START, "blocked", {"to"}, true);

  auto conditions = move_action_->get_conditions();
  EXPECT_EQ(conditions.size(), 1u);
  EXPECT_TRUE(conditions[0].is_negated());
}

TEST_F(ActionTest, AddEffect) {
  move_action_->add_effect(Type::END, "at", {"r", "to"});
  move_action_->add_effect(Type::END, "at", {"r", "from"}, true);

  auto effects = move_action_->get_effects();
  EXPECT_EQ(effects.size(), 2u);
}

TEST_F(ActionTest, GetOnStartConditions) {
  move_action_->add_condition(Type::START, "at", {"r", "from"});
  move_action_->add_condition(Type::OVER_ALL, "available", {"r"});
  move_action_->add_condition(Type::END, "clear", {"to"});

  auto start_conditions = move_action_->get_on_start_conditions();
  EXPECT_EQ(start_conditions.size(), 1u);
  EXPECT_EQ(start_conditions[0].get_type(), Type::START);
}

TEST_F(ActionTest, GetOverAllConditions) {
  move_action_->add_condition(Type::START, "at", {"r", "from"});
  move_action_->add_condition(Type::OVER_ALL, "available", {"r"});
  move_action_->add_condition(Type::END, "clear", {"to"});

  auto over_all_conditions = move_action_->get_over_all_conditions();
  EXPECT_EQ(over_all_conditions.size(), 1u);
  EXPECT_EQ(over_all_conditions[0].get_type(), Type::OVER_ALL);
}

TEST_F(ActionTest, GetOnEndConditions) {
  move_action_->add_condition(Type::START, "at", {"r", "from"});
  move_action_->add_condition(Type::OVER_ALL, "available", {"r"});
  move_action_->add_condition(Type::END, "clear", {"to"});

  auto end_conditions = move_action_->get_on_end_conditions();
  EXPECT_EQ(end_conditions.size(), 1u);
  EXPECT_EQ(end_conditions[0].get_type(), Type::END);
}

TEST_F(ActionTest, GetOnStartEffects) {
  move_action_->add_effect(Type::START, "at", {"r", "from"}, true);
  move_action_->add_effect(Type::OVER_ALL, "moving", {"r"});
  move_action_->add_effect(Type::END, "at", {"r", "to"});

  auto start_effects = move_action_->get_on_start_effects();
  EXPECT_EQ(start_effects.size(), 1u);
  EXPECT_EQ(start_effects[0].get_type(), Type::START);
}

TEST_F(ActionTest, GetOverAllEffects) {
  move_action_->add_effect(Type::START, "at", {"r", "from"}, true);
  move_action_->add_effect(Type::OVER_ALL, "moving", {"r"});
  move_action_->add_effect(Type::END, "at", {"r", "to"});

  auto over_all_effects = move_action_->get_over_all_effects();
  EXPECT_EQ(over_all_effects.size(), 1u);
  EXPECT_EQ(over_all_effects[0].get_type(), Type::OVER_ALL);
}

TEST_F(ActionTest, GetOnEndEffects) {
  move_action_->add_effect(Type::START, "at", {"r", "from"}, true);
  move_action_->add_effect(Type::OVER_ALL, "moving", {"r"});
  move_action_->add_effect(Type::END, "at", {"r", "to"});

  auto end_effects = move_action_->get_on_end_effects();
  EXPECT_EQ(end_effects.size(), 1u);
  EXPECT_EQ(end_effects[0].get_type(), Type::END);
}

TEST_F(ActionTest, ToPddlBasic) {
  std::string pddl = move_action_->to_pddl();

  EXPECT_TRUE(pddl.find("(:durative-action move") != std::string::npos);
  EXPECT_TRUE(pddl.find(":parameters") != std::string::npos);
  EXPECT_TRUE(pddl.find("?r - robot") != std::string::npos);
  EXPECT_TRUE(pddl.find("?from - location") != std::string::npos);
  EXPECT_TRUE(pddl.find("?to - location") != std::string::npos);
  EXPECT_TRUE(pddl.find(":duration") != std::string::npos);
  EXPECT_TRUE(pddl.find(":condition") != std::string::npos);
  EXPECT_TRUE(pddl.find(":effect") != std::string::npos);
}

TEST_F(ActionTest, ToPddlWithConditionsAndEffects) {
  move_action_->add_condition(Type::START, "at", {"r", "from"});
  move_action_->add_condition(Type::START, "connected", {"from", "to"});
  move_action_->add_effect(Type::START, "at", {"r", "from"}, true);
  move_action_->add_effect(Type::END, "at", {"r", "to"});

  std::string pddl = move_action_->to_pddl();

  EXPECT_TRUE(pddl.find(":condition") != std::string::npos);
  EXPECT_TRUE(pddl.find(":effect") != std::string::npos);
}

TEST_F(ActionTest, ToPddlNoParams) {
  auto action = std::make_shared<TestAction>("wait");

  std::string pddl = action->to_pddl();

  EXPECT_TRUE(pddl.find("(:durative-action wait") != std::string::npos);
  EXPECT_TRUE(pddl.find(":parameters ()") != std::string::npos);
}

TEST_F(ActionTest, RunReturnsStatus) {
  move_action_->set_return_status(ActionStatus::SUCCEED);
  ActionStatus status = move_action_->run({"robot1", "loc1", "loc2"});

  EXPECT_EQ(status, ActionStatus::SUCCEED);
  EXPECT_TRUE(move_action_->run_called_);
}

TEST_F(ActionTest, RunWithAbortStatus) {
  move_action_->set_return_status(ActionStatus::ABORT);
  ActionStatus status = move_action_->run({"robot1", "loc1", "loc2"});

  EXPECT_EQ(status, ActionStatus::ABORT);
}

TEST_F(ActionTest, RunWithCancelStatus) {
  move_action_->set_return_status(ActionStatus::CANCEL);
  ActionStatus status = move_action_->run({"robot1", "loc1", "loc2"});

  EXPECT_EQ(status, ActionStatus::CANCEL);
}

TEST_F(ActionTest, RunStoresParams) {
  move_action_->run({"robot1", "loc1", "loc2"});

  EXPECT_EQ(move_action_->last_params_.size(), 3u);
  EXPECT_EQ(move_action_->last_params_[0], "robot1");
  EXPECT_EQ(move_action_->last_params_[1], "loc1");
  EXPECT_EQ(move_action_->last_params_[2], "loc2");
}

TEST_F(ActionTest, Cancel) {
  EXPECT_FALSE(move_action_->cancel_called_);

  move_action_->cancel();

  EXPECT_TRUE(move_action_->cancel_called_);
}

TEST_F(ActionTest, EmptyConditions) {
  auto conditions = simple_action_->get_conditions();
  EXPECT_EQ(conditions.size(), 0u);
}

TEST_F(ActionTest, EmptyEffects) {
  auto effects = simple_action_->get_effects();
  EXPECT_EQ(effects.size(), 0u);
}

TEST_F(ActionTest, EmptyOnStartConditions) {
  auto conditions = simple_action_->get_on_start_conditions();
  EXPECT_EQ(conditions.size(), 0u);
}

TEST_F(ActionTest, EmptyOverAllConditions) {
  auto conditions = simple_action_->get_over_all_conditions();
  EXPECT_EQ(conditions.size(), 0u);
}

TEST_F(ActionTest, EmptyOnEndConditions) {
  auto conditions = simple_action_->get_on_end_conditions();
  EXPECT_EQ(conditions.size(), 0u);
}

TEST_F(ActionTest, EmptyOnStartEffects) {
  auto effects = simple_action_->get_on_start_effects();
  EXPECT_EQ(effects.size(), 0u);
}

TEST_F(ActionTest, EmptyOverAllEffects) {
  auto effects = simple_action_->get_over_all_effects();
  EXPECT_EQ(effects.size(), 0u);
}

TEST_F(ActionTest, EmptyOnEndEffects) {
  auto effects = simple_action_->get_on_end_effects();
  EXPECT_EQ(effects.size(), 0u);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
