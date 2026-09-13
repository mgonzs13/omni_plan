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
#include <memory>
#include <thread>

#include "omni_plan_cache/detail/plan_cache.hpp"

using namespace omni_plan_cache;
using namespace omni_plan_cache::detail;

TEST(PlanCacheTest, PutAndGet) {
  PlanCache cache;
  cache.configure(0, 0);
  auto data = std::make_shared<CachedPlanData>();
  data->plan.set_has_solution(true);

  cache.put("exact", "structural", data);
  EXPECT_EQ(cache.get_exact("exact"), data);
  EXPECT_EQ(cache.get_structural("structural"), data);
  EXPECT_EQ(cache.get_exact("missing"), nullptr);

  const auto stats = cache.stats();
  EXPECT_EQ(stats.exact_hits, 1u);
  EXPECT_EQ(stats.structural_hits, 1u);
  EXPECT_EQ(stats.exact_entries, 1u);
  EXPECT_EQ(stats.structural_entries, 1u);
}

TEST(PlanCacheTest, BoundedEviction) {
  PlanCache cache;
  cache.configure(1, 1);
  auto first = std::make_shared<CachedPlanData>();
  auto second = std::make_shared<CachedPlanData>();

  cache.put("e1", "s1", first);
  cache.put("e2", "s2", second);

  EXPECT_EQ(cache.get_exact("e1"), nullptr);
  EXPECT_EQ(cache.get_structural("s1"), nullptr);
  EXPECT_EQ(cache.get_exact("e2"), second);
  EXPECT_EQ(cache.get_structural("s2"), second);

  const auto stats = cache.stats();
  EXPECT_EQ(stats.exact_evictions, 1u);
  EXPECT_EQ(stats.structural_evictions, 1u);
}

TEST(PlanCacheTest, SingleFlightJoinsLeader) {
  PlanCache cache;
  cache.configure(0, 0);

  const auto flight = cache.begin_or_join("key");
  EXPECT_TRUE(flight.leader);

  std::shared_future<std::shared_ptr<const CachedPlanData>> follower_future;
  std::atomic<bool> follower_joined{false};
  std::thread follower([&]() {
    const auto f = cache.begin_or_join("key");
    EXPECT_FALSE(f.leader);
    follower_future = f.future;
    follower_joined.store(true);
  });
  while (!follower_joined.load()) {
    std::this_thread::yield();
  }

  auto data = std::make_shared<CachedPlanData>();
  cache.publish("key", data);
  follower.join();

  ASSERT_TRUE(follower_future.valid());
  EXPECT_EQ(follower_future.get(), data);
}

TEST(PlanCacheTest, ReentrantOwnerDoesNotJoinItself) {
  PlanCache cache;
  cache.configure(0, 0);
  const auto outer = cache.begin_or_join("key");
  EXPECT_TRUE(outer.leader);
  const auto inner = cache.begin_or_join("key");
  EXPECT_TRUE(inner.leader);
}

TEST(PlanCacheTest, LeaderGuardReleasesFollowersOnAbandon) {
  PlanCache cache;
  cache.configure(0, 0);
  std::shared_future<std::shared_ptr<const CachedPlanData>> follower;
  std::atomic<bool> follower_joined{false};
  {
    const auto leader = cache.begin_or_join("key");
    EXPECT_TRUE(leader.leader);
    std::thread follower_thread([&]() {
      const auto joined = cache.begin_or_join("key");
      EXPECT_FALSE(joined.leader);
      follower = joined.future;
      follower_joined.store(true);
    });
    while (!follower_joined.load()) {
      std::this_thread::yield();
    }
    follower_thread.join();
  } // leader guard destructor abandons the entry
  ASSERT_TRUE(follower.valid());
  EXPECT_EQ(follower.get(), nullptr);
}

TEST(PlanCacheTest, StaleLeaderGuardDoesNotAbandonNewFlight) {
  PlanCache cache;
  cache.configure(0, 0);

  auto leader = cache.begin_or_join("key");
  EXPECT_TRUE(leader.leader);
  cache.publish("key", std::make_shared<CachedPlanData>());

  const auto new_leader = cache.begin_or_join("key");
  EXPECT_TRUE(new_leader.leader);

  std::shared_future<std::shared_ptr<const CachedPlanData>> follower_future;
  std::atomic<bool> follower_joined{false};
  std::thread follower([&]() {
    const auto joined = cache.begin_or_join("key");
    EXPECT_FALSE(joined.leader);
    follower_future = joined.future;
    follower_joined.store(true);
  });
  while (!follower_joined.load()) {
    std::this_thread::yield();
  }
  follower.join();

  leader = PlanCache::Flight{}; // stale guard must not touch the new flight

  auto second = std::make_shared<CachedPlanData>();
  cache.publish("key", second);

  ASSERT_TRUE(follower_future.valid());
  EXPECT_EQ(follower_future.get(), second);
}
