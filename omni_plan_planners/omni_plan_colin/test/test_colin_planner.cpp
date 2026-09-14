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

#include <cstdlib>
#include <gtest/gtest.h>
#include <memory>
#include <string>
#include <vector>

#include <sys/wait.h>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/utils/package_share_path.hpp"
#include "omni_plan_colin/colin_planner.hpp"

using namespace omni_plan_colin;

namespace {} // namespace

// Mock Action class for testing
class MockAction : public omni_plan::pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params = {})
      : Action(name, params), cancel_called_(false) {}

  omni_plan::pddl::ActionStatus
  run(const std::vector<std::string> & /*params*/) override {
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override { cancel_called_ = true; }

  bool cancel_called_;
};

class ColinPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    rclcpp::init(0, nullptr);
    node_ = std::make_shared<rclcpp::Node>("test_node");
    planner_ = std::make_unique<ColinPlanner>();
    planner_->load_ros_parameters(node_);

    // Build simple domain.  Predicate arguments are types; action parameters
    // and condition/effect arguments are bare names (the PDDL "?" prefix is
    // added by the serializers).
    simple_domain_obj_.add_requirement("strips");
    simple_domain_obj_.add_requirement("typing");
    simple_domain_obj_.add_requirement("durative-actions");
    simple_domain_obj_.add_type("location");
    simple_domain_obj_.add_type("robot");
    simple_domain_obj_.add_predicate(
        omni_plan::pddl::Predicate("at", {"robot", "location"}));
    simple_domain_obj_.add_predicate(
        omni_plan::pddl::Predicate("connected", {"location", "location"}));
    std::vector<std::pair<std::string, std::string>> params = {
        {"r", "robot"}, {"from", "location"}, {"to", "location"}};
    auto move_action = std::make_shared<MockAction>("move", params);
    move_action->add_condition(omni_plan::pddl::Type::START, "at",
                               {"r", "from"});
    move_action->add_condition(omni_plan::pddl::Type::START, "connected",
                               {"from", "to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"r", "to"});
    move_action->add_effect(omni_plan::pddl::Type::END, "at", {"r", "from"},
                            true);
    simple_domain_obj_.add_action(move_action);

    // Build simple problem
    simple_problem_obj_.add_object(omni_plan::pddl::Object("robot1", "robot"));
    simple_problem_obj_.add_object(omni_plan::pddl::Object("loc1", "location"));
    simple_problem_obj_.add_object(omni_plan::pddl::Object("loc2", "location"));
    simple_problem_obj_.add_fact(
        omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    simple_problem_obj_.add_fact(
        omni_plan::pddl::Predicate("connected", {"loc1", "loc2"}));
    simple_problem_obj_.add_goal(
        omni_plan::pddl::Predicate("at", {"robot1", "loc2"}));

    // Build unsolvable domain (same as simple)
    unsolvable_domain_obj_ = simple_domain_obj_;

    // Build unsolvable problem
    unsolvable_problem_obj_.add_object(
        omni_plan::pddl::Object("robot1", "robot"));
    unsolvable_problem_obj_.add_object(
        omni_plan::pddl::Object("loc1", "location"));
    unsolvable_problem_obj_.add_object(
        omni_plan::pddl::Object("loc2", "location"));
    unsolvable_problem_obj_.add_fact(
        omni_plan::pddl::Predicate("at", {"robot1", "loc1"}));
    unsolvable_problem_obj_.add_goal(
        omni_plan::pddl::Predicate("at", {"robot1", "loc2"}));
  }

  void TearDown() override { rclcpp::shutdown(); }

  std::unique_ptr<ColinPlanner> planner_;
  std::shared_ptr<rclcpp::Node> node_;
  omni_plan::pddl::Domain simple_domain_obj_;
  omni_plan::pddl::Problem simple_problem_obj_;
  omni_plan::pddl::Domain unsolvable_domain_obj_;
  omni_plan::pddl::Problem unsolvable_problem_obj_;
};

// Test: ColinPlanner constructor
TEST_F(ColinPlannerTest, ConstructorCreatesPlanner) {
  EXPECT_NE(planner_, nullptr);
}

// Test: generate_plan with invalid domain returns failed plan
TEST_F(ColinPlannerTest, GetPlanWithInvalidDomainReturnsFailed) {
  auto plan =
      planner_->generate_plan(omni_plan::pddl::Domain(), simple_problem_obj_);

  EXPECT_FALSE(plan.has_solution());
}

// Test: generate_plan with empty domain returns failed plan
TEST_F(ColinPlannerTest, GetPlanWithEmptyDomainReturnsFailed) {
  auto plan = planner_->generate_plan(omni_plan::pddl::Domain(),
                                      omni_plan::pddl::Problem());

  EXPECT_FALSE(plan.has_solution());
}

// Test: generate_plan with unsolvable problem returns failed plan
TEST_F(ColinPlannerTest, GetPlanWithUnsolvableProblemReturnsFailed) {
  auto plan =
      planner_->generate_plan(simple_domain_obj_, unsolvable_problem_obj_);

  EXPECT_FALSE(plan.has_solution());
}

// Test: Plan size is 0 for failed plans
TEST_F(ColinPlannerTest, FailedPlanHasSizeZero) {
  auto plan =
      planner_->generate_plan(omni_plan::pddl::Domain(), simple_problem_obj_);

  EXPECT_EQ(plan.size(), 0u);
}

// Test: Multiple calls to generate_plan work correctly
TEST_F(ColinPlannerTest, MultiplePlannerCalls) {
  auto plan1 =
      planner_->generate_plan(omni_plan::pddl::Domain(), simple_problem_obj_);
  auto plan2 =
      planner_->generate_plan(omni_plan::pddl::Domain(), simple_problem_obj_);

  EXPECT_FALSE(plan1.has_solution());
  EXPECT_FALSE(plan2.has_solution());
}

// Integration test: Valid domain and problem (requires COLIN to be installed)
TEST_F(ColinPlannerTest, ValidDomainAndProblemReturnsPlan) {
  const std::string colin_binary =
      omni_plan::utils::get_package_share_path("omni_plan_colin") +
      "/bin/colin";

  auto plan = planner_->generate_plan(simple_domain_obj_, simple_problem_obj_);

  // The fixture is known-solvable: a missing solution means COLIN is broken.
  ASSERT_TRUE(plan.has_solution()) << "COLIN failed to solve the fixture";
  EXPECT_GT(plan.size(), 0u);

  // Verify the plan contains the expected action
  auto action = plan.get_action(0);
  ASSERT_NE(action, nullptr);
  EXPECT_EQ(action->get_name(), "move");
}
