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
#include <thread>
#include <vector>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/planner.hpp"

#include "omni_plan_homeostatic/homeostatic_planner_selector.hpp"

using namespace omni_plan_homeostatic;

class TestPlanner : public omni_plan::Planner {
public:
  TestPlanner() = default;
};

class HomeostaticSelectorTest : public ::testing::Test {
protected:
  void SetUp() override {
    planner_a_ = std::make_shared<TestPlanner>();
    planner_b_ = std::make_shared<TestPlanner>();
    planner_c_ = std::make_shared<TestPlanner>();
  }

  std::shared_ptr<TestPlanner> planner_a_;
  std::shared_ptr<TestPlanner> planner_b_;
  std::shared_ptr<TestPlanner> planner_c_;
};

// ---- Constructor ----

TEST_F(HomeostaticSelectorTest, ConstructorDefaultParams) {
  HomeostaticPlannerSelector sel(0.3);
  EXPECT_EQ(sel.get_num_planners(), 0u);
}

TEST_F(HomeostaticSelectorTest, AddPlanners) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);
  sel.add_planner("VHPOP", planner_c_);
  EXPECT_EQ(sel.get_num_planners(), 3u);
}

// ---- Fallback with no data ----

TEST_F(HomeostaticSelectorTest, FallbackSelectsFirstPlanner) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  std::string selected;
  auto planner = sel.select_planner("hash1", selected);
  EXPECT_EQ(planner, planner_a_);
  EXPECT_EQ(selected, "POPF");
}

// ---- Record observations ----

TEST_F(HomeostaticSelectorTest, RecordObservationAccumulatesCost) {
  HomeostaticPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);

  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 200.0, true);

  // select_planner should pick the only planner
  std::string selected;
  sel.select_planner("hash1", selected);

  // The cost table should show 2 observations, total cost = 300
  std::string table = sel.get_planner_cost_table();
  EXPECT_NE(table.find("avg=150"), std::string::npos);
  EXPECT_NE(table.find("times=2"), std::string::npos);
  EXPECT_NE(table.find("succ=2"), std::string::npos);
}

TEST_F(HomeostaticSelectorTest, RecordObservationSeparateHashes) {
  HomeostaticPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hashA", "POPF", 50.0, true);
  sel.record_observation("hashA", "SMTP", 150.0, true);
  sel.record_observation("hashB", "POPF", 150.0, true);

  std::string selected;
  sel.select_planner("hashA", selected);
  EXPECT_EQ(selected, "POPF");
}

TEST_F(HomeostaticSelectorTest, RecordSucceededFalse) {
  HomeostaticPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);

  sel.record_observation("hash1", "POPF", 100.0, false);

  std::string table = sel.get_planner_cost_table();
  EXPECT_NE(table.find("succ=0"), std::string::npos);
}

TEST_F(HomeostaticSelectorTest, UCBZeroTrialsSelectedImmediately) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 100.0, true);

  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "SMTP");
}

// ---- Exploitation: picks cheapest ----

TEST_F(HomeostaticSelectorTest, ExploitationPicksCheapest) {
  HomeostaticPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 200.0, true);
  sel.record_observation("hash1", "SMTP", 50.0, true);

  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "SMTP");
}

TEST_F(HomeostaticSelectorTest, ExploitationAveragesOverMultipleCalls) {
  HomeostaticPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  // POPF: avg = 100, SMTP: avg = 125
  sel.record_observation("hash1", "POPF", 50.0, true);
  sel.record_observation("hash1", "POPF", 150.0, true);
  sel.record_observation("hash1", "SMTP", 100.0, true);
  sel.record_observation("hash1", "SMTP", 150.0, true);

  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "POPF");
}

TEST_F(HomeostaticSelectorTest, ExploitationFallsBackToGlobalAverage) {
  HomeostaticPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  // Only hashA has data — SMTP is cheaper there
  sel.record_observation("hashA", "POPF", 200.0, true);
  sel.record_observation("hashA", "SMTP", 50.0, true);

  // No data for hashB → falls back to global average → SMTP wins
  std::string selected;
  sel.select_planner("hashB", selected);
  EXPECT_EQ(selected, "SMTP");
}

// ---- Cold-start ----

TEST_F(HomeostaticSelectorTest, NeedsColdStartTrueWithNoData) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  EXPECT_TRUE(sel.needs_cold_start(1));
  EXPECT_TRUE(sel.needs_cold_start(3));
}

TEST_F(HomeostaticSelectorTest, NeedsColdStartFalseAfterSufficientData) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "SMTP", 100.0, true);
  sel.record_observation("hash1", "SMTP", 100.0, true);
  sel.record_observation("hash1", "SMTP", 100.0, true);

  EXPECT_FALSE(sel.needs_cold_start(3));
}

TEST_F(HomeostaticSelectorTest, NeedsColdStartTrueWhenOnePlannerLags) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 100.0, true);

  EXPECT_TRUE(sel.needs_cold_start(3));
}

TEST_F(HomeostaticSelectorTest, GetPlannerCostTableWithColdStart) {
  HomeostaticPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);

  sel.record_observation("abcdef123456", "POPF", 100.0, true);

  std::string table = sel.get_planner_cost_table();
  EXPECT_NE(table.find("Cost Table"), std::string::npos);
  EXPECT_NE(table.find("abcdef12"), std::string::npos);
  EXPECT_NE(table.find("POPF"), std::string::npos);
}

// ---- Thread safety ----

TEST_F(HomeostaticSelectorTest, ConcurrentAccess) {
  HomeostaticPlannerSelector sel(0.5);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);
  sel.add_planner("VHPOP", planner_c_);

  std::vector<std::thread> threads;
  for (int t = 0; t < 4; t++) {
    threads.emplace_back([&sel]() {
      for (int i = 0; i < 25; i++) {
        std::string selected;
        sel.select_planner("hash_concurrent", selected);
        sel.record_observation("hash_concurrent", selected, 100.0, true);
      }
    });
  }
  for (auto &th : threads) {
    th.join();
  }

  std::string table = sel.get_planner_cost_table();
  EXPECT_NE(table.find("Cost Table"), std::string::npos);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
