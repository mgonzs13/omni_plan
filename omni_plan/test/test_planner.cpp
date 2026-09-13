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

#include <filesystem>
#include <gtest/gtest.h>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/pddl_manager.hpp"
#include "omni_plan/plan_validator.hpp"
#include "omni_plan/planner.hpp"

using namespace omni_plan;

namespace {

/**
 * @brief Minimal concrete action used to populate domains in tests.
 */
class TestActionStub : public pddl::Action {
public:
  explicit TestActionStub(const std::string &name) : Action(name) {}

  pddl::ActionStatus run(const std::vector<std::string> &) override {
    return pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override {}
};

bool has_owner_only_perms(const std::string &path) {
  std::error_code ec;
  auto perms = std::filesystem::status(path, ec).permissions();
  if (ec) {
    return false;
  }
  return perms == (std::filesystem::perms::owner_read |
                   std::filesystem::perms::owner_write);
}

bool is_inside_private_dir(const std::string &path) {
  std::filesystem::path parent = std::filesystem::path(path).parent_path();
  return parent != std::filesystem::temp_directory_path();
}

pddl::Predicate unnegated(const pddl::Predicate &predicate) {
  return pddl::Predicate(predicate.get_name(), predicate.get_args(), false);
}

} // namespace

/**
 * @brief Concrete planner for testing time parsing methods.
 */
class TestPlanner : public Planner {
public:
  TestPlanner() : Planner() {}

  std::string
  generate_plan(const std::string & /*domain_path*/,
                const std::string & /*problem_path*/) const override {
    return "";
  }

  bool has_solution(const std::string & /*plan_str*/) const override {
    return false;
  }

  // Expose protected methods for testing
  float test_parse_start_time(const std::string &line) const {
    return parse_start_time(line);
  }

  std::pair<std::string, std::vector<std::string>>
  test_parse_action_line(const std::string &line) const {
    return parse_action_line(line);
  }

  std::vector<std::string>
  test_get_lines_with_actions(const std::string &plan_str) const {
    return get_lines_with_actions(plan_str);
  }
};

// ==================== Start Time Parsing Tests ====================
class PlannerParseTest : public ::testing::Test {
protected:
  TestPlanner planner_;
};

TEST_F(PlannerParseTest, ParseStartTimePopfFormat) {
  // POPF output format: "0.000: (move robot1 room1 room2) [10.000]"
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(
                      "0.000: (move robot1 room1 room2) [10.000]"),
                  0.0f);
}

TEST_F(PlannerParseTest, ParseStartTimeNonZero) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(
                      "10.500: (pick robot1 item1 room2) [5.000]"),
                  10.5f);
}

TEST_F(PlannerParseTest, ParseStartTimeLargeValue) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(
                      "123.456: (drop robot1 item1 room3) [2.500]"),
                  123.456f);
}

TEST_F(PlannerParseTest, ParseStartTimeNoColon) {
  EXPECT_FLOAT_EQ(
      planner_.test_parse_start_time("(move robot1 room1 room2) [10.000]"),
      0.0f);
}

TEST_F(PlannerParseTest, ParseStartTimeEmptyLine) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time(""), 0.0f);
}

TEST_F(PlannerParseTest, ParseStartTimeInvalidNumber) {
  EXPECT_FLOAT_EQ(planner_.test_parse_start_time("abc: (move) [10.0]"), 0.0f);
}

// ==================== Action Line Parsing Tests ====================
TEST_F(PlannerParseTest, ParseActionLinePopfFormat) {
  auto [name, params] = planner_.test_parse_action_line(
      "0.000: (move robot1 room1 room2) [10.000]");
  EXPECT_EQ(name, "move");
  ASSERT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "room1");
  EXPECT_EQ(params[2], "room2");
}

TEST_F(PlannerParseTest, ParseActionLineNoParams) {
  auto [name, params] =
      planner_.test_parse_action_line("0.000: (wait) [5.000]");
  EXPECT_EQ(name, "wait");
  EXPECT_EQ(params.size(), 0u);
}

// ==================== Lines Filtering Tests ====================
TEST_F(PlannerParseTest, GetLinesWithActionsMultiple) {
  std::string plan_str = "Some header line\n"
                         "0.000: (move robot1 room1 room2) [10.000]\n"
                         "0.000: (pick robot2 item2 room2) [5.000]\n"
                         "10.001: (pick robot1 item1 room2) [5.000]\n"
                         "Some footer line\n";

  auto lines = planner_.test_get_lines_with_actions(plan_str);
  ASSERT_EQ(lines.size(), 3u);
  EXPECT_TRUE(lines[0].find("move") != std::string::npos);
  EXPECT_TRUE(lines[1].find("robot2") != std::string::npos);
  EXPECT_TRUE(lines[2].find("10.001") != std::string::npos);
}

TEST_F(PlannerParseTest, GetLinesWithActionsEmpty) {
  auto lines = planner_.test_get_lines_with_actions("");
  EXPECT_EQ(lines.size(), 0u);
}

// ==================== Combined Time + Action Parsing ====================
TEST_F(PlannerParseTest, FullLineParsing) {
  std::string line = "15.250: (drop robot1 item1 room3) [3.500]";

  auto [name, params] = planner_.test_parse_action_line(line);
  float start_time = planner_.test_parse_start_time(line);

  EXPECT_EQ(name, "drop");
  ASSERT_EQ(params.size(), 3u);
  EXPECT_EQ(params[0], "robot1");
  EXPECT_EQ(params[1], "item1");
  EXPECT_EQ(params[2], "room3");
  EXPECT_FLOAT_EQ(start_time, 15.25f);
}

// ==================== Plan Parsing Tests ====================

class ParsePlanPlanner : public Planner {
public:
  ParsePlanPlanner() : Planner() {}

  std::string
  generate_plan(const std::string & /*domain_path*/,
                const std::string & /*problem_path*/) const override {
    return "";
  }

  bool has_solution(const std::string &plan_str) const override {
    return plan_str.find("Solution found") != std::string::npos;
  }
};

TEST(PlannerParsePlanTest, KnownActionsAreParsed) {
  ParsePlanPlanner planner;
  pddl::Domain domain;
  domain.add_action(std::make_shared<TestActionStub>("move"));
  domain.add_action(std::make_shared<TestActionStub>("pick"));

  std::string raw = "Solution found\n"
                    "0.000: (move robot1 room1 room2) [10.000]\n"
                    "10.000: (pick robot1 item1 room2) [5.000]\n";

  pddl::Plan plan = planner.parse_plan(domain, raw);

  EXPECT_TRUE(plan.has_solution());
  EXPECT_EQ(plan.size(), 2u);
}

TEST(PlannerParsePlanTest, UnknownActionMarksPlanAsInvalid) {
  ParsePlanPlanner planner;
  pddl::Domain domain;
  domain.add_action(std::make_shared<TestActionStub>("move"));

  std::string raw = "Solution found\n"
                    "0.000: (move robot1 room1 room2) [10.000]\n"
                    "10.000: (teleport robot1 room2 room3) [1.000]\n";

  pddl::Plan plan = planner.parse_plan(domain, raw);

  EXPECT_FALSE(plan.has_solution());
}

TEST(PlannerParsePlanTest, SolutionMarkerWithoutActionsMarksPlanAsInvalid) {
  ParsePlanPlanner planner;
  pddl::Domain domain;
  domain.add_action(std::make_shared<TestActionStub>("move"));

  pddl::Plan plan = planner.parse_plan(domain, "Solution found\n");

  EXPECT_FALSE(plan.has_solution());
}

// ==================== Temporary File Tests ====================

class TempFilePlanner : public Planner {
public:
  using Planner::generate_plan;

  TempFilePlanner() : Planner() {}

  std::string generate_plan(const std::string &domain_path,
                            const std::string &problem_path) const override {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->domain_paths.push_back(domain_path);
    this->problem_paths.push_back(problem_path);
    this->perms_ok = this->perms_ok && has_owner_only_perms(domain_path) &&
                     has_owner_only_perms(problem_path);
    this->in_private_dirs = this->in_private_dirs &&
                            is_inside_private_dir(domain_path) &&
                            is_inside_private_dir(problem_path);
    return "";
  }

  bool has_solution(const std::string &) const override { return true; }

  mutable std::mutex mutex;
  mutable std::vector<std::string> domain_paths;
  mutable std::vector<std::string> problem_paths;
  mutable bool perms_ok = true;
  mutable bool in_private_dirs = true;
};

TEST(PlannerTempFileTest, FilesArePrivateAndRemovedAfterPlanning) {
  TempFilePlanner planner;
  pddl::Domain domain;
  domain.add_action(std::make_shared<TestActionStub>("move"));
  pddl::Problem problem;

  planner.generate_plan(domain, problem);

  ASSERT_EQ(planner.domain_paths.size(), 1u);
  ASSERT_EQ(planner.problem_paths.size(), 1u);
  EXPECT_TRUE(planner.perms_ok);
  EXPECT_TRUE(planner.in_private_dirs);
  EXPECT_FALSE(std::filesystem::exists(planner.domain_paths[0]));
  EXPECT_FALSE(std::filesystem::exists(planner.problem_paths[0]));
  EXPECT_FALSE(std::filesystem::exists(
      std::filesystem::path(planner.domain_paths[0]).parent_path()));
}

TEST(PlannerTempFileTest, ConcurrentCallsDoNotCollide) {
  TempFilePlanner planner;
  pddl::Domain domain;
  domain.add_action(std::make_shared<TestActionStub>("move"));
  pddl::Problem problem;

  std::thread first([&planner, &domain, &problem] {
    planner.generate_plan(domain, problem);
  });
  std::thread second([&planner, &domain, &problem] {
    planner.generate_plan(domain, problem);
  });
  first.join();
  second.join();

  ASSERT_EQ(planner.domain_paths.size(), 2u);
  ASSERT_EQ(planner.problem_paths.size(), 2u);
  EXPECT_NE(planner.domain_paths[0], planner.domain_paths[1]);
  EXPECT_NE(planner.problem_paths[0], planner.problem_paths[1]);
  for (const auto &path : planner.domain_paths) {
    EXPECT_FALSE(std::filesystem::exists(path));
  }
  for (const auto &path : planner.problem_paths) {
    EXPECT_FALSE(std::filesystem::exists(path));
  }
}

class TempFileValidator : public PlanValidator {
public:
  using PlanValidator::validate_plan;

  TempFileValidator() : PlanValidator() {}

  mutable std::mutex mutex;
  mutable std::vector<std::string> paths;
  mutable bool perms_ok = true;
  mutable bool in_private_dirs = true;

protected:
  bool validate_plan(const std::string &domain_path,
                     const std::string &problem_path,
                     const std::string &plan_path) const override {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->paths.push_back(domain_path);
    this->paths.push_back(problem_path);
    this->paths.push_back(plan_path);
    this->perms_ok = this->perms_ok && has_owner_only_perms(domain_path) &&
                     has_owner_only_perms(problem_path) &&
                     has_owner_only_perms(plan_path);
    this->in_private_dirs =
        this->in_private_dirs && is_inside_private_dir(domain_path) &&
        is_inside_private_dir(problem_path) && is_inside_private_dir(plan_path);
    return true;
  }
};

TEST(PlanValidatorTempFileTest, FilesArePrivateAndRemovedAfterValidation) {
  TempFileValidator validator;
  pddl::Domain domain;
  domain.add_action(std::make_shared<TestActionStub>("move"));
  pddl::Problem problem;
  pddl::Plan plan;

  EXPECT_TRUE(validator.validate_plan(domain, problem, plan));

  ASSERT_EQ(validator.paths.size(), 3u);
  EXPECT_TRUE(validator.perms_ok);
  EXPECT_TRUE(validator.in_private_dirs);
  for (const auto &path : validator.paths) {
    EXPECT_FALSE(std::filesystem::exists(path));
  }
  EXPECT_FALSE(std::filesystem::exists(
      std::filesystem::path(validator.paths[0]).parent_path()));
}

// ==================== PddlManager Effect Tests ====================

class MockPddlManager : public PddlManager {
public:
  MockPddlManager() : PddlManager() {}

  std::pair<pddl::Domain, pddl::Problem> get_pddl() const override {
    return {};
  }

  bool has_goals() const override { return !this->goals.empty(); }

  bool clear_goals() const override {
    this->goals.clear();
    return true;
  }

  bool predicate_exists(const pddl::Predicate &predicate) const override {
    ++this->predicate_exists_calls;
    return this->facts.count(unnegated(predicate)) > 0;
  }

  bool predicate_is_goal(const pddl::Predicate &predicate) const override {
    return this->goals.count(unnegated(predicate)) > 0;
  }

  void apply_effect(const pddl::Effect &effect) override {
    this->applied.push_back(effect);
    if (effect.is_negated()) {
      this->facts.erase(unnegated(effect));
    } else {
      this->facts.insert(unnegated(effect));
      this->goals.erase(unnegated(effect));
    }
  }

  mutable int predicate_exists_calls = 0;
  mutable std::set<pddl::Predicate> facts;
  mutable std::set<pddl::Predicate> goals;
  mutable std::vector<pddl::Effect> applied;
};

TEST(PddlManagerApplyEffectsTest, PredicateExistsIsEvaluatedOnce) {
  MockPddlManager manager;
  manager.facts.insert(pddl::Predicate("at", {"robot1", "room1"}));

  manager.apply_effects(
      {pddl::Effect(pddl::Type::END, "at", {"robot1", "room1"})});

  EXPECT_EQ(manager.predicate_exists_calls, 1);
}

TEST(PddlManagerApplyEffectsTest, AlreadyTrueGoalEffectConsumesGoal) {
  MockPddlManager manager;
  pddl::Predicate goal("at", {"robot1", "room1"});
  manager.facts.insert(goal);
  manager.goals.insert(goal);

  auto applied = manager.apply_effects(
      {pddl::Effect(pddl::Type::END, "at", {"robot1", "room1"})});

  ASSERT_EQ(manager.applied.size(), 1u);
  EXPECT_TRUE(manager.goals.empty());
  // The world state did not change, so nothing must be registered for undo.
  EXPECT_TRUE(applied.empty());
}

TEST(PddlManagerApplyEffectsTest, AlreadyTrueNonGoalEffectIsNotApplied) {
  MockPddlManager manager;
  manager.facts.insert(pddl::Predicate("at", {"robot1", "room1"}));

  auto applied = manager.apply_effects(
      {pddl::Effect(pddl::Type::END, "at", {"robot1", "room1"})});

  EXPECT_TRUE(manager.applied.empty());
  EXPECT_TRUE(applied.empty());
}

TEST(PddlManagerApplyEffectsTest, ChangedEffectsAreRecordedAndUndoable) {
  MockPddlManager manager;
  manager.facts.insert(pddl::Predicate("at", {"robot1", "room1"}));

  auto applied = manager.apply_effects(
      {pddl::Effect(pddl::Type::END, "at", {"robot1", "room2"}),
       pddl::Effect(pddl::Type::START, "at", {"robot1", "room1"}, true)});

  ASSERT_EQ(applied.size(), 2u);
  ASSERT_EQ(manager.applied.size(), 2u);
  EXPECT_EQ(manager.facts.count(pddl::Predicate("at", {"robot1", "room1"})),
            0u);
  EXPECT_EQ(manager.facts.count(pddl::Predicate("at", {"robot1", "room2"})),
            1u);

  // Mirror PlanDispatcher::undo_effects: reverse and invert the negation.
  std::vector<pddl::Effect> reversed = applied;
  for (auto &effect : reversed) {
    effect.set_negation(!effect.is_negated());
  }
  manager.apply_effects(reversed);

  EXPECT_EQ(manager.facts.count(pddl::Predicate("at", {"robot1", "room1"})),
            1u);
  EXPECT_EQ(manager.facts.count(pddl::Predicate("at", {"robot1", "room2"})),
            0u);
}

TEST(PddlManagerApplyEffectsTest, RepeatedPositiveEffectIsRecordedOnce) {
  MockPddlManager manager;

  auto first = manager.apply_effects(
      {pddl::Effect(pddl::Type::END, "at", {"robot1", "room1"})});
  auto second = manager.apply_effects(
      {pddl::Effect(pddl::Type::END, "at", {"robot1", "room1"})});

  EXPECT_EQ(first.size(), 1u);
  EXPECT_TRUE(second.empty());
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
