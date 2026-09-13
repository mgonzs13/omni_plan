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

#include <atomic>
#include <chrono>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/cache_planner.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;

namespace {

class MockAction : public pddl::Action {
public:
  MockAction(const std::string &name,
             std::vector<std::pair<std::string, std::string>> params)
      : Action(name, params) {}
  pddl::ActionStatus run(const std::vector<std::string> &) override {
    return pddl::ActionStatus::SUCCEEDED;
  }
  void cancel() override {}
};

class SlowMockPlanner : public Planner {
public:
  mutable std::atomic<int> calls{0};
  mutable std::atomic<int> delay_ms{5};

  omni_plan::pddl::Plan generate_plan(const pddl::Domain &domain,
                                      const pddl::Problem &) const override {
    this->calls.fetch_add(1);
    std::this_thread::sleep_for(
        std::chrono::milliseconds(this->delay_ms.load()));
    auto action = domain.get_actions().at("move");
    pddl::Plan plan;
    plan.set_has_solution(true);
    plan.set_raw_output("0.000: (move robot1 loc1 loc2) [10.000]\n");
    plan.add_action(action, {"robot1", "loc1", "loc2"}, 0.0f);
    return plan;
  }
};

class GatedMockPlanner : public Planner {
public:
  mutable std::atomic<int> calls{0};
  mutable std::promise<void> entered;
  mutable std::promise<void> release_promise;
  mutable std::shared_future<void> release{
      release_promise.get_future().share()};

  omni_plan::pddl::Plan generate_plan(const pddl::Domain &domain,
                                      const pddl::Problem &) const override {
    this->calls.fetch_add(1);
    if (this->calls.load() == 1) {
      this->entered.set_value();
      this->release.wait();
    }
    auto action = domain.get_actions().at("move");
    pddl::Plan plan;
    plan.set_has_solution(true);
    plan.set_raw_output("gated\n");
    plan.add_action(action, {"robot1", "loc1", "loc2"}, 0.0f);
    return plan;
  }
};

class MockValidator : public PlanValidator {
public:
  mutable std::atomic<int> calls{0};
  bool result_ = true;
  bool validate_plan(const pddl::Domain &, const pddl::Problem &,
                     const pddl::Plan &) const override {
    this->calls.fetch_add(1);
    return this->result_;
  }

protected:
  bool validate_plan(const std::string &, const std::string &,
                     const std::string &) const override {
    return this->result_;
  }
};

class TestableCachePlanner : public CachePlanner {
public:
  void inject(std::shared_ptr<Planner> planner,
              std::shared_ptr<PlanValidator> validator) {
    wrapped_planner_ = std::move(planner);
    validator_ = std::move(validator);
    validate_on_hit_ = true;
  }
};

/// @brief CachePlanner that refuses to cache any result.
class NoCachePlanner : public TestableCachePlanner {
public:
  bool should_cache_result(const omni_plan::pddl::Plan &) const override {
    return false;
  }
};

/**
 * @brief Planner whose outer solve recursively queries the cache again.
 *
 * The recursive query uses a structurally identical problem (same structural
 * key, different exact key), so it re-enters the cache while the outer flight
 * is still active. The nested result has a different plan length, making any
 * publish into the outer flight observable.
 */
class RecursiveMockPlanner : public Planner {
public:
  mutable std::atomic<int> calls{0};
  CachePlanner *cache = nullptr;
  pddl::Problem nested_problem;
  mutable std::promise<void> outer_entered;
  mutable std::promise<void> follower_ready_promise;
  mutable std::shared_future<void> follower_ready =
      follower_ready_promise.get_future().share();

  omni_plan::pddl::Plan
  generate_plan(const pddl::Domain &domain,
                const pddl::Problem &problem) const override {
    this->calls.fetch_add(1);
    std::string robot;
    for (const auto &obj : problem.get_objects()) {
      if (obj.get_type() == "robot") {
        robot = obj.get_name();
      }
    }
    auto make_plan = [&domain, &robot](int actions) {
      const auto action = domain.get_actions().at("move");
      pddl::Plan plan;
      plan.set_has_solution(true);
      plan.set_raw_output("recursive\n");
      for (int i = 0; i < actions; ++i) {
        plan.add_action(action, {robot, "from", "to"}, 0.0f);
      }
      return plan;
    };
    if (robot == "robot1") {
      this->outer_entered.set_value();
      this->follower_ready.wait();
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
      this->cache->generate_plan(domain, this->nested_problem);
      return make_plan(2);
    }
    if (robot == "r2") {
      return make_plan(1);
    }
    return make_plan(3);
  }
};

pddl::Domain make_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("connected", {"?l1", "?l2"}));
  auto move = std::make_shared<MockAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
  move->add_condition(pddl::Type::START, "at", {"?r", "?from"});
  move->add_condition(pddl::Type::START, "connected", {"?from", "?to"});
  move->add_effect(pddl::Type::END, "at", {"?r", "?to"});
  move->add_effect(pddl::Type::END, "at", {"?r", "?from"}, true);
  domain.add_action(move);
  return domain;
}

pddl::Problem make_problem(const std::string &robot, const std::string &from,
                           const std::string &to) {
  pddl::Problem problem;
  problem.add_object(pddl::Object(robot, "robot"));
  problem.add_object(pddl::Object(from, "location"));
  problem.add_object(pddl::Object(to, "location"));
  problem.add_fact(pddl::Predicate("at", {robot, from}));
  problem.add_fact(pddl::Predicate("connected", {from, to}));
  problem.add_goal(pddl::Predicate("at", {robot, to}));
  return problem;
}

} // namespace

TEST(CachePlannerExtrasTest, AdaptedHitKeepsRawOutput) {
  auto node = std::make_shared<rclcpp::Node>("extras_raw_output");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  planner.inject(std::make_shared<SlowMockPlanner>(),
                 std::make_shared<MockValidator>());

  const auto domain = make_domain();
  planner.generate_plan(domain, make_problem("robot1", "loc1", "loc2"));
  auto hit = planner.generate_plan(domain, make_problem("r2", "lab", "office"));
  EXPECT_TRUE(hit.has_solution());
  EXPECT_FALSE(hit.get_raw_output().empty());
}

TEST(CachePlannerExtrasTest, SingleFlightDeduplicatesConcurrentMisses) {
  auto node = std::make_shared<rclcpp::Node>("extras_single_flight");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<SlowMockPlanner>();
  mock->delay_ms.store(50);
  planner.inject(mock, nullptr);

  const auto domain = make_domain();
  const auto problem = make_problem("robot1", "loc1", "loc2");

  std::vector<std::thread> threads;
  for (int i = 0; i < 4; ++i) {
    threads.emplace_back([&]() { planner.generate_plan(domain, problem); });
  }
  for (auto &thread : threads) {
    thread.join();
  }

  EXPECT_EQ(mock->calls.load(), 1);
  // Only the real leader counts as a full miss; followers must not.
  EXPECT_EQ(planner.get_cache_stats().full_misses, 1u);
}

TEST(CachePlannerExtrasTest, BoundedCachesEvict) {
  auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("planner.max_exact_cache_entries", 1),
       rclcpp::Parameter("planner.max_structural_cache_entries", 1)});
  auto node = std::make_shared<rclcpp::Node>("extras_bounded", options);
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  planner.inject(std::make_shared<SlowMockPlanner>(),
                 std::make_shared<MockValidator>());

  const auto domain = make_domain();
  planner.generate_plan(domain, make_problem("robot1", "loc1", "loc2"));
  planner.generate_plan(domain, make_problem("robot2", "loc3", "loc4"));

  pddl::Problem chain;
  chain.add_object(pddl::Object("robot3", "robot"));
  chain.add_object(pddl::Object("loc5", "location"));
  chain.add_object(pddl::Object("loc6", "location"));
  chain.add_object(pddl::Object("loc7", "location"));
  chain.add_fact(pddl::Predicate("at", {"robot3", "loc5"}));
  chain.add_fact(pddl::Predicate("connected", {"loc5", "loc6"}));
  chain.add_fact(pddl::Predicate("connected", {"loc6", "loc7"}));
  chain.add_goal(pddl::Predicate("at", {"robot3", "loc7"}));
  planner.generate_plan(domain, chain);

  const auto stats = planner.get_cache_stats();
  EXPECT_LE(stats.exact_entries, 1u);
  EXPECT_LE(stats.structural_entries, 1u);
  EXPECT_GE(stats.exact_evictions + stats.structural_evictions, 1u);
}

TEST(CachePlannerExtrasTest, MetricsCountHitsAndMisses) {
  auto node = std::make_shared<rclcpp::Node>("extras_metrics");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  planner.inject(std::make_shared<SlowMockPlanner>(),
                 std::make_shared<MockValidator>());

  const auto domain = make_domain();
  const auto problem = make_problem("robot1", "loc1", "loc2");
  planner.generate_plan(domain, problem);
  planner.generate_plan(domain, problem);

  const auto stats = planner.get_cache_stats();
  EXPECT_EQ(stats.exact_hits, 1u);
  EXPECT_EQ(stats.full_misses, 1u);
}

TEST(CachePlannerExtrasTest, FollowerExactHitKeepsOwnNames) {
  auto node = std::make_shared<rclcpp::Node>("extras_follower_names");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<GatedMockPlanner>();
  planner.inject(mock, nullptr);

  const auto domain = make_domain();
  const auto prob_a = make_problem("robot1", "loc1", "loc2");
  const auto prob_b = make_problem("r2", "lab", "office");

  std::thread leader([&]() { planner.generate_plan(domain, prob_a); });
  mock->entered.get_future().wait();
  std::thread follower([&]() { planner.generate_plan(domain, prob_b); });
  std::this_thread::sleep_for(std::chrono::milliseconds(200));
  mock->release_promise.set_value();
  leader.join();
  follower.join();

  EXPECT_EQ(mock->calls.load(), 1);
  auto again = planner.generate_plan(domain, prob_b);
  ASSERT_EQ(again.size(), 1u);
  const auto params = again.get_action_params(0);
  EXPECT_EQ(params[0], "r2");
  EXPECT_EQ(params[1], "lab");
  EXPECT_EQ(params[2], "office");
}

TEST(CachePlannerExtrasTest, ComposedPlanKeepsRawOutput) {
  auto node = std::make_shared<rclcpp::Node>("extras_composed");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<SlowMockPlanner>();
  mock->delay_ms.store(0);
  planner.inject(mock, std::make_shared<MockValidator>());

  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("robot2", "robot"));
  problem.add_object(pddl::Object("loc1", "location"));
  problem.add_object(pddl::Object("loc2", "location"));
  problem.add_object(pddl::Object("loc3", "location"));
  problem.add_object(pddl::Object("loc4", "location"));
  problem.add_fact(pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(pddl::Predicate("at", {"robot2", "loc3"}));
  problem.add_fact(pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_fact(pddl::Predicate("connected", {"loc3", "loc4"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "loc2"}));
  problem.add_goal(pddl::Predicate("at", {"robot2", "loc4"}));

  auto plan = planner.generate_plan(domain, problem);
  EXPECT_TRUE(plan.has_solution());
  EXPECT_FALSE(plan.get_raw_output().empty());
}

// Regression: the component-composition path must honor should_cache_result();
// a composed plan refused by the policy must not be stored.
TEST(CachePlannerExtrasTest, CompositionHonorsShouldCacheResult) {
  auto node = std::make_shared<rclcpp::Node>("extras_composition_nocache");
  NoCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<SlowMockPlanner>();
  mock->delay_ms.store(0);
  planner.inject(mock, std::make_shared<MockValidator>());

  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("robot2", "robot"));
  problem.add_object(pddl::Object("loc1", "location"));
  problem.add_object(pddl::Object("loc2", "location"));
  problem.add_object(pddl::Object("loc3", "location"));
  problem.add_object(pddl::Object("loc4", "location"));
  problem.add_fact(pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(pddl::Predicate("at", {"robot2", "loc3"}));
  problem.add_fact(pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_fact(pddl::Predicate("connected", {"loc3", "loc4"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "loc2"}));
  problem.add_goal(pddl::Predicate("at", {"robot2", "loc4"}));

  auto plan1 = planner.generate_plan(domain, problem);
  ASSERT_TRUE(plan1.has_solution());
  const int calls_after_first = mock->calls.load();
  ASSERT_GE(calls_after_first, 1);

  // The refused composed plan must not be cached: the second call has to
  // reach the wrapped planner again.
  auto plan2 = planner.generate_plan(domain, problem);
  ASSERT_TRUE(plan2.has_solution());
  EXPECT_GT(mock->calls.load(), calls_after_first);

  const auto stats = planner.get_cache_stats();
  EXPECT_EQ(stats.exact_entries, 0u);
  EXPECT_EQ(stats.structural_entries, 0u);
}

// Regression: a re-entrant nested solve on the same structural key must not
// publish into the outer flight; followers must observe the outer plan.
TEST(CachePlannerExtrasTest, ReentrantCallDoesNotPublishOverOuterFlight) {
  auto node = std::make_shared<rclcpp::Node>("extras_reentrant");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<RecursiveMockPlanner>();
  mock->cache = &planner;
  mock->nested_problem = make_problem("r2", "lab", "office");
  planner.inject(mock, nullptr);

  const auto domain = make_domain();
  const auto outer_problem = make_problem("robot1", "loc1", "loc2");
  const auto follower_problem = make_problem("r3", "kitchen", "dining");

  pddl::Plan leader_plan;
  pddl::Plan follower_plan;

  std::thread leader(
      [&]() { leader_plan = planner.generate_plan(domain, outer_problem); });
  std::thread follower([&]() {
    mock->outer_entered.get_future().wait();
    mock->follower_ready_promise.set_value();
    follower_plan = planner.generate_plan(domain, follower_problem);
  });

  leader.join();
  follower.join();

  // Only the outer and nested solves run; the follower is served.
  EXPECT_EQ(mock->calls.load(), 2);
  ASSERT_TRUE(leader_plan.has_solution());
  ASSERT_TRUE(follower_plan.has_solution());
  EXPECT_EQ(leader_plan.size(), 2u);
  // A nested publish would hand the 1-action nested plan to the follower.
  EXPECT_EQ(follower_plan.size(), 2u);
}

// Regression: a structural hit rejected by the validator must still fall back
// to planning and return a plan without throwing.
TEST(CachePlannerExtrasTest, ValidationFailureFallsBackToPlanning) {
  auto node = std::make_shared<rclcpp::Node>("extras_validation_fallback");
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<SlowMockPlanner>();
  mock->delay_ms.store(0);
  auto validator = std::make_shared<MockValidator>();
  validator->result_ = false;
  planner.inject(mock, validator);

  const auto domain = make_domain();
  auto first =
      planner.generate_plan(domain, make_problem("robot1", "loc1", "loc2"));
  ASSERT_TRUE(first.has_solution());

  auto second =
      planner.generate_plan(domain, make_problem("r2", "lab", "office"));
  EXPECT_TRUE(second.has_solution());
  EXPECT_EQ(mock->calls.load(), 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
