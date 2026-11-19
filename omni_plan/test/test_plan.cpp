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

#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/plan.hpp"

using namespace omni_plan::pddl;

/**
 * @brief Mock action for Plan tests
 */
class MockPlanAction : public Action {
public:
  MockPlanAction(const std::string &name,
                 std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params) {}

  ActionStatus run(const std::vector<std::string> & /*params*/) override {
    return ActionStatus::SUCCEEDED;
  }

  void cancel() override {}
};

// ==================== Plan Tests ====================

class PlanTest : public ::testing::Test {
protected:
  void SetUp() override {
    move_action_ = std::make_shared<MockPlanAction>(
        "move", std::vector<std::pair<std::string, std::string>>{
                    {"r", "robot"}, {"from", "location"}, {"to", "location"}});

    pick_action_ = std::make_shared<MockPlanAction>(
        "pick", std::vector<std::pair<std::string, std::string>>{
                    {"r", "robot"}, {"i", "item"}, {"l", "location"}});

    drop_action_ = std::make_shared<MockPlanAction>(
        "drop", std::vector<std::pair<std::string, std::string>>{
                    {"r", "robot"}, {"i", "item"}, {"l", "location"}});
  }

  std::shared_ptr<MockPlanAction> move_action_;
  std::shared_ptr<MockPlanAction> pick_action_;
  std::shared_ptr<MockPlanAction> drop_action_;
};

TEST_F(PlanTest, DefaultConstructorNoSolution) {
  Plan plan;

  EXPECT_FALSE(plan.has_solution());
  EXPECT_EQ(plan.size(), 0u);
}

TEST_F(PlanTest, ConstructorWithSolution) {
  Plan plan;
  plan.set_has_solution(true);

  EXPECT_TRUE(plan.has_solution());
  EXPECT_EQ(plan.size(), 0u);
}

TEST_F(PlanTest, ConstructorWithoutSolution) {
  Plan plan;
  plan.set_has_solution(false);

  EXPECT_FALSE(plan.has_solution());
}

TEST_F(PlanTest, SetHasSolution) {
  Plan plan;

  EXPECT_FALSE(plan.has_solution());

  plan.set_has_solution(true);
  EXPECT_TRUE(plan.has_solution());

  plan.set_has_solution(false);
  EXPECT_FALSE(plan.has_solution());
}

TEST_F(PlanTest, AddAction) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});

  EXPECT_EQ(plan.size(), 1u);
}

TEST_F(PlanTest, AddMultipleActions) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});
  plan.add_action(pick_action_, {"robot1", "item1", "loc2"});
  plan.add_action(move_action_, {"robot1", "loc2", "loc3"});
  plan.add_action(drop_action_, {"robot1", "item1", "loc3"});

  EXPECT_EQ(plan.size(), 4u);
}

TEST_F(PlanTest, AddActionWithEmptyParams) {
  Plan plan;
  plan.set_has_solution(true);

  auto no_param_action = std::make_shared<MockPlanAction>("wait");
  plan.add_action(no_param_action);

  EXPECT_EQ(plan.size(), 1u);
  EXPECT_EQ(plan.get_action_params(0).size(), 0u);
}

TEST_F(PlanTest, GetAction) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});
  plan.add_action(pick_action_, {"robot1", "item1", "loc2"});

  EXPECT_EQ(plan.get_action(0)->get_name(), "move");
  EXPECT_EQ(plan.get_action(1)->get_name(), "pick");
}

TEST_F(PlanTest, GetActionParams) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});

  auto params = plan.get_action_params(0);

  EXPECT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "loc1");
  EXPECT_EQ(params[2], "loc2");
}

TEST_F(PlanTest, GetActionWithParams) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});

  auto [action, params] = plan.get_action_with_params(0);

  EXPECT_EQ(action->get_name(), "move");
  EXPECT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0], "robot1");
}

TEST_F(PlanTest, GetActionOutOfRange) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});

  EXPECT_THROW(plan.get_action(5), std::out_of_range);
}

TEST_F(PlanTest, GetActionParamsOutOfRange) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});

  EXPECT_THROW(plan.get_action_params(5), std::out_of_range);
}

TEST_F(PlanTest, ToPddlEmpty) {
  Plan plan;

  std::string pddl = plan.to_pddl();

  EXPECT_EQ(pddl, "\n");
}

TEST_F(PlanTest, ToPddlSingleAction) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});

  std::string pddl = plan.to_pddl();

  EXPECT_TRUE(pddl.find("(move robot1 loc1 loc2)") != std::string::npos);
}

TEST_F(PlanTest, ToPddlMultipleActions) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});
  plan.add_action(pick_action_, {"robot1", "item1", "loc2"});

  std::string pddl = plan.to_pddl();

  EXPECT_TRUE(pddl.find("(move robot1 loc1 loc2)") != std::string::npos);
  EXPECT_TRUE(pddl.find("(pick robot1 item1 loc2)") != std::string::npos);
}

TEST_F(PlanTest, ToPddlNoParams) {
  Plan plan;
  plan.set_has_solution(true);

  auto wait_action = std::make_shared<MockPlanAction>("wait");
  plan.add_action(wait_action);

  std::string pddl = plan.to_pddl();

  EXPECT_TRUE(pddl.find("(wait)") != std::string::npos);
}

TEST_F(PlanTest, SizeEmpty) {
  Plan plan;

  EXPECT_EQ(plan.size(), 0u);
}

TEST_F(PlanTest, SizeAfterAdditions) {
  Plan plan;
  plan.set_has_solution(true);

  EXPECT_EQ(plan.size(), 0u);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});
  EXPECT_EQ(plan.size(), 1u);

  plan.add_action(pick_action_, {"robot1", "item1", "loc2"});
  EXPECT_EQ(plan.size(), 2u);

  plan.add_action(drop_action_, {"robot1", "item1", "loc3"});
  EXPECT_EQ(plan.size(), 3u);
}

TEST_F(PlanTest, ActionOrderPreserved) {
  Plan plan;
  plan.set_has_solution(true);

  plan.add_action(move_action_, {"robot1", "loc1", "loc2"});
  plan.add_action(pick_action_, {"robot1", "item1", "loc2"});
  plan.add_action(drop_action_, {"robot1", "item1", "loc3"});

  EXPECT_EQ(plan.get_action(0)->get_name(), "move");
  EXPECT_EQ(plan.get_action(1)->get_name(), "pick");
  EXPECT_EQ(plan.get_action(2)->get_name(), "drop");
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
