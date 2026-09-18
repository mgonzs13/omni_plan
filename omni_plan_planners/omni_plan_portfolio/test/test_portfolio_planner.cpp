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

#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/planner.hpp"

#include "omni_plan_portfolio/portfolio_planner.hpp"
#include "omni_plan_portfolio/portfolio_planner_selector.hpp"

using namespace omni_plan_portfolio;

class TestPlanner : public omni_plan::Planner {
public:
  TestPlanner() = default;
};

class ThrowingPlanner : public omni_plan::Planner {
public:
  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &,
                const omni_plan::pddl::Problem &) const override {
    throw std::runtime_error("sub-planner failure");
  }
};

class SolutionPlanner : public omni_plan::Planner {
public:
  omni_plan::pddl::Plan
  generate_plan(const omni_plan::pddl::Domain &,
                const omni_plan::pddl::Problem &) const override {
    omni_plan::pddl::Plan plan;
    plan.set_has_solution(true);
    return plan;
  }
};

class TestablePortfolioPlanner : public PortfolioPlanner {
public:
  using PortfolioPlanner::delegate_plan;

  std::shared_ptr<PortfolioPlannerSelector> &selector() {
    return this->selector_;
  }
};

class PortfolioSelectorTest : public ::testing::Test {
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

TEST_F(PortfolioSelectorTest, ConstructorDefaultParams) {
  PortfolioPlannerSelector sel(0.3);
  EXPECT_EQ(sel.get_num_planners(), 0u);
}

TEST_F(PortfolioSelectorTest, AddPlanners) {
  PortfolioPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);
  sel.add_planner("VHPOP", planner_c_);
  EXPECT_EQ(sel.get_num_planners(), 3u);
}

// ---- Fallback with no data ----

TEST_F(PortfolioSelectorTest, FallbackSelectsFirstPlanner) {
  PortfolioPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  std::string selected;
  auto planner = sel.select_planner("hash1", selected);
  EXPECT_EQ(planner, planner_a_);
  EXPECT_EQ(selected, "POPF");
}

// ---- Record observations ----

TEST_F(PortfolioSelectorTest, RecordObservationAccumulatesCost) {
  PortfolioPlannerSelector sel(0.0);
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

TEST_F(PortfolioSelectorTest, RecordObservationSeparateHashes) {
  PortfolioPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hashA", "POPF", 50.0, true);
  sel.record_observation("hashA", "SMTP", 150.0, true);
  sel.record_observation("hashB", "POPF", 150.0, true);

  std::string selected;
  sel.select_planner("hashA", selected);
  EXPECT_EQ(selected, "POPF");
}

TEST_F(PortfolioSelectorTest, RecordSucceededFalse) {
  PortfolioPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);

  sel.record_observation("hash1", "POPF", 100.0, false);

  std::string table = sel.get_planner_cost_table();
  EXPECT_NE(table.find("succ=0"), std::string::npos);
}

TEST_F(PortfolioSelectorTest, ZeroTrialPlannerUsesGlobalPrior) {
  PortfolioPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 100.0, true);

  // The planner without observations in this hash is scored with a global
  // prior instead of being selected immediately (no alphabetical bias).
  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "POPF");
}

// ---- Empty selector ----

TEST_F(PortfolioSelectorTest, SelectPlannerWithNoPlannersThrows) {
  PortfolioPlannerSelector sel;

  std::string selected;
  EXPECT_THROW(sel.select_planner("hash1", selected), std::runtime_error);
}

// ---- Reliability: failures must penalize a planner ----

TEST_F(PortfolioSelectorTest, FailurePenaltyPrefersSuccessfulPlanner) {
  PortfolioPlannerSelector sel(0.0);
  sel.add_planner("FAIL", planner_a_);
  sel.add_planner("GOOD", planner_b_);

  // FAIL is very fast but never returns a solution; GOOD is slower.
  for (int i = 0; i < 3; ++i) {
    sel.record_observation("hash1", "FAIL", 1.0, false);
    sel.record_observation("hash1", "GOOD", 100.0, true);
  }

  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "GOOD");
}

TEST_F(PortfolioSelectorTest, PartialFailuresReduceReliability) {
  PortfolioPlannerSelector sel(0.0);
  sel.add_planner("FLAKY", planner_a_);
  sel.add_planner("STEADY", planner_b_);

  // FLAKY has a lower average cost but only succeeds one third of the time.
  sel.record_observation("hash1", "FLAKY", 100.0, true);
  sel.record_observation("hash1", "FLAKY", 1.0, false);
  sel.record_observation("hash1", "FLAKY", 1.0, false);
  for (int i = 0; i < 3; ++i) {
    sel.record_observation("hash1", "STEADY", 50.0, true);
  }

  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "STEADY");
}

// Regression test for the iterator dereference on a cold hash key while the
// cost table is already populated (previously UB: hash_it->second before the
// hash_it != end() check).
TEST_F(PortfolioSelectorTest, SelectBrandNewHashWhenCostTableNonEmpty) {
  PortfolioPlannerSelector sel(0.5);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  for (int i = 0; i < 3; ++i) {
    sel.record_observation("hash_known", "POPF", 100.0, true);
    sel.record_observation("hash_known", "SMTP", 120.0, true);
  }
  ASSERT_FALSE(sel.needs_cold_start(3));

  std::string selected;
  auto planner = sel.select_planner("hash_brand_new", selected);
  EXPECT_TRUE(planner == planner_a_ || planner == planner_b_);
  EXPECT_FALSE(selected.empty());
}

// ---- Exploitation: picks cheapest ----

TEST_F(PortfolioSelectorTest, ExploitationPicksCheapest) {
  PortfolioPlannerSelector sel(0.0);
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 200.0, true);
  sel.record_observation("hash1", "SMTP", 50.0, true);

  std::string selected;
  sel.select_planner("hash1", selected);
  EXPECT_EQ(selected, "SMTP");
}

TEST_F(PortfolioSelectorTest, ExploitationAveragesOverMultipleCalls) {
  PortfolioPlannerSelector sel(0.0);
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

TEST_F(PortfolioSelectorTest, ExploitationFallsBackToGlobalAverage) {
  PortfolioPlannerSelector sel(0.0);
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

TEST_F(PortfolioSelectorTest, NeedsColdStartTrueWithNoData) {
  PortfolioPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  EXPECT_TRUE(sel.needs_cold_start(1));
  EXPECT_TRUE(sel.needs_cold_start(3));
}

TEST_F(PortfolioSelectorTest, NeedsColdStartFalseAfterSufficientData) {
  PortfolioPlannerSelector sel;
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

TEST_F(PortfolioSelectorTest, NeedsColdStartTrueWhenOnePlannerLags) {
  PortfolioPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);
  sel.add_planner("SMTP", planner_b_);

  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 100.0, true);
  sel.record_observation("hash1", "POPF", 100.0, true);

  EXPECT_TRUE(sel.needs_cold_start(3));
}

TEST_F(PortfolioSelectorTest, GetPlannerCostTableWithColdStart) {
  PortfolioPlannerSelector sel;
  sel.add_planner("POPF", planner_a_);

  sel.record_observation("abcdef123456", "POPF", 100.0, true);

  std::string table = sel.get_planner_cost_table();
  EXPECT_NE(table.find("Cost Table"), std::string::npos);
  EXPECT_NE(table.find("abcdef12"), std::string::npos);
  EXPECT_NE(table.find("POPF"), std::string::npos);
}

// ---- Thread safety ----

TEST_F(PortfolioSelectorTest, ConcurrentAccess) {
  PortfolioPlannerSelector sel(0.5);
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

// ---- PortfolioPlanner delegate_plan behaviour ----

class PortfolioPlannerTest : public ::testing::Test {
protected:
  void SetUp() override {
    node_ = std::make_shared<rclcpp::Node>("test_portfolio_planner_node");
    node_->declare_parameter("planner.planner_plugins",
                             std::vector<std::string>{});
    planner_ = std::make_shared<TestablePortfolioPlanner>();
    planner_->load_ros_parameters(node_);
  }

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<TestablePortfolioPlanner> planner_;
  omni_plan::pddl::Domain domain_;
  omni_plan::pddl::Problem problem_;
};

TEST_F(PortfolioPlannerTest, NoSubPlannersReturnsEmptyPlan) {
  ASSERT_NE(planner_->selector(), nullptr);
  ASSERT_EQ(planner_->selector()->get_num_planners(), 0u);

  auto plan = planner_->delegate_plan(domain_, problem_, "hash_no_planners");
  EXPECT_FALSE(plan.has_solution());
}

TEST_F(PortfolioPlannerTest, ThrowingSubPlannerDoesNotAbortSelection) {
  auto selector = std::make_shared<PortfolioPlannerSelector>(0.0);
  selector->add_planner("a_throwing", std::make_shared<ThrowingPlanner>());
  selector->add_planner("b_good", std::make_shared<SolutionPlanner>());
  planner_->selector() = selector;

  auto start = std::chrono::steady_clock::now();
  for (int i = 0; i < 4; ++i) {
    auto plan =
        planner_->delegate_plan(domain_, problem_, "hash_throwing_planner");
    EXPECT_TRUE(plan.has_solution());
  }
  auto elapsed = std::chrono::steady_clock::now() - start;

  // A throwing sub-planner must not block the ensemble, and the POIROT wait
  // must be bounded so a missing profiling result cannot stall the call.
  EXPECT_LT(std::chrono::duration<double>(elapsed).count(), 10.0);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
