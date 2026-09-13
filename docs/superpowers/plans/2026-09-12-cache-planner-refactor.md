# CachePlanner Refactor and Optimization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the CachePlanner bugs, cut per-call CPU time, simplify it into focused internal units behind a stable facade, add safe parallelism, optional cache bounds/metrics, and a benchmark — without committing and without breaking existing tests or `HomeostaticPlanner`.

**Architecture:** Keep `CachePlanner`'s public/protected API; move algorithms into `omni_plan_cache::detail` units (`StructuralKeyer`, `RelevanceAnalyzer`, `PlanAdapter`, `ComponentComposer`, `PlanCache`, `Sha256`) and make `generate_plan` a thin orchestrator. Existing public static helpers delegate to the detail units.

**Tech Stack:** C++17, ROS 2 Humble, ament/colcon, pluginlib, OpenSSL EVP (SHA-256), gtest, `std::async`/`std::shared_mutex`, no new dependencies.

**User constraints (override the skill defaults):**
- **Do NOT run `git commit` at any point.**
- Keep the existing 23 tests passing unchanged.
- `HomeostaticPlanner` must compile with no source changes.

**Build/test commands (run from `/home/miguel/ros2_ws`):**

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select omni_plan_cache --event-handlers console_direct+
colcon test-result --verbose
```

Always rebuild before running tests; a stale install tree has caused ABI-mismatch segfaults in this workspace.

**Parallelism note for the executor:** Tasks 4–9 touch disjoint files (one detail unit + one test file each) and can be executed by parallel subagents once Task 3 is done. Tasks 1–3 and 10–12 are sequential.

---

### Task 1: Baseline benchmark

**Files:**
- Create: `omni_plan_planners/omni_plan_cache/benchmark/benchmark_cache_planner.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/CMakeLists.txt`

- [ ] **Step 1: Write the benchmark harness**

```cpp
// Copyright (C) 2026 Miguel Ángel González Santamarta
// GPL-3.0-or-later

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <ctime>
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
#include "omni_plan/plan_validator.hpp"
#include "omni_plan_cache/cache_planner.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;

namespace {

class BenchAction : public pddl::Action {
public:
  BenchAction(const std::string &name,
              std::vector<std::pair<std::string, std::string>> params)
      : Action(name, params) {}
  pddl::ActionStatus run(const std::vector<std::string> &) override {
    return pddl::ActionStatus::SUCCEEDED;
  }
  void cancel() override {}
};

class BenchPlanner : public Planner {
public:
  mutable std::atomic<int> calls{0};
  omni_plan::pddl::Plan generate_plan(const pddl::Domain &,
                                      const pddl::Problem &) const override {
    this->calls.fetch_add(1);
    pddl::Plan plan;
    plan.set_has_solution(true);
    plan.set_raw_output("bench\n");
    return plan;
  }
};

class BenchValidator : public PlanValidator {
public:
  bool validate_plan(const pddl::Domain &, const pddl::Problem &,
                     const pddl::Plan &) const override {
    return true;
  }

protected:
  bool validate_plan(const std::string &, const std::string &,
                     const std::string &) const override {
    return true;
  }
};

class BenchCachePlanner : public CachePlanner {
public:
  void inject(std::shared_ptr<Planner> planner,
              std::shared_ptr<PlanValidator> validator) {
    wrapped_planner_ = std::move(planner);
    validator_ = std::move(validator);
    validate_on_hit_ = true;
  }
};

double thread_cpu_us() {
  timespec ts;
  clock_gettime(CLOCK_THREAD_CPUTIME_ID, &ts);
  return static_cast<double>(ts.tv_sec) * 1e6 +
         static_cast<double>(ts.tv_nsec) / 1e3;
}

pddl::Domain make_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("connected", {"?l1", "?l2"}));
  domain.add_action(std::make_shared<BenchAction>(
      "move",
      std::vector<std::pair<std::string, std::string>>{
          {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}}));
  return domain;
}

pddl::Problem make_problem(int n, const std::string &suffix) {
  pddl::Problem problem;
  const std::string robot = "robot" + suffix;
  problem.add_object(pddl::Object(robot, "robot"));
  for (int i = 0; i < n; ++i) {
    problem.add_object(pddl::Object("loc" + suffix + "_" + std::to_string(i),
                                    "location"));
  }
  problem.add_fact(pddl::Predicate("at", {robot, "loc" + suffix + "_0"}));
  for (int i = 0; i + 1 < n; ++i) {
    problem.add_fact(pddl::Predicate(
        "connected", {"loc" + suffix + "_" + std::to_string(i),
                      "loc" + suffix + "_" + std::to_string(i + 1)}));
  }
  problem.add_goal(pddl::Predicate(
      "at", {robot, "loc" + suffix + "_" + std::to_string(n - 1)}));
  return problem;
}

void report(const char *name, double total_us, int iterations) {
  std::printf("%-28s total=%10.2f us  per-call=%10.2f us  n=%d\n", name,
              total_us, total_us / static_cast<double>(iterations), iterations);
}

} // namespace

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("bench_cache");
  BenchCachePlanner planner;
  planner.load_ros_parameters(node);
  auto mock = std::make_shared<BenchPlanner>();
  planner.inject(mock, std::make_shared<BenchValidator>());

  const auto domain = make_domain();
  const int n = 40;

  {
    const auto problem = make_problem(n, "_exact");
    planner.generate_plan(domain, problem);
    const int iters = 2000;
    double start = thread_cpu_us();
    for (int i = 0; i < iters; ++i) {
      planner.generate_plan(domain, problem);
    }
    report("exact hit", thread_cpu_us() - start, iters);
  }

  {
    const auto problem = make_problem(n, "_s0");
    planner.generate_plan(domain, problem);
    const int iters = 2000;
    double start = thread_cpu_us();
    for (int i = 1; i <= iters; ++i) {
      planner.generate_plan(domain, make_problem(n, "_s" + std::to_string(i)));
    }
    report("structural hit", thread_cpu_us() - start, iters);
  }

  {
    const int iters = 200;
    double start = thread_cpu_us();
    for (int i = 0; i < iters; ++i) {
      planner.generate_plan(
          domain, make_problem(20 + i, "_miss" + std::to_string(i)));
    }
    report("miss", thread_cpu_us() - start, iters);
  }

  {
    const int threads = 4;
    const int per_thread = 500;
    std::vector<std::thread> workers;
    auto start = std::chrono::steady_clock::now();
    for (int t = 0; t < threads; ++t) {
      workers.emplace_back([&, t]() {
        for (int i = 0; i < per_thread; ++i) {
          planner.generate_plan(
              domain, make_problem(n, "_t" + std::to_string(t) + "_" +
                                             std::to_string(i)));
        }
      });
    }
    for (auto &w : workers) {
      w.join();
    }
    auto wall_us = std::chrono::duration_cast<std::chrono::microseconds>(
                       std::chrono::steady_clock::now() - start)
                       .count();
    std::printf("%-28s wall=%10.2f us  n=%d  mock_calls=%d\n", "concurrent",
                static_cast<double>(wall_us), threads * per_thread,
                mock->calls.load());
  }

  rclcpp::shutdown();
  return 0;
}
```

- [ ] **Step 2: Add the benchmark target to CMake**

Append inside the existing `if(BUILD_TESTING)` block in `omni_plan_planners/omni_plan_cache/CMakeLists.txt`:

```cmake
  add_executable(benchmark_cache_planner
    benchmark/benchmark_cache_planner.cpp
  )
  target_include_directories(benchmark_cache_planner PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(benchmark_cache_planner
    cache_planner
    ${DEPENDENCIES}
  )
```

- [ ] **Step 3: Build and run the baseline**

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
./build/omni_plan_cache/benchmark_cache_planner | tee /tmp/opencode/cache_bench_baseline.txt
```
Expected: prints four lines (`exact hit`, `structural hit`, `miss`, `concurrent`) and exits 0. Save the output; Task 12 compares against it.

- [ ] **Step 4: Confirm existing tests still pass**

Run:
```bash
colcon test --packages-select omni_plan_cache --event-handlers console_direct+
colcon test-result --verbose
```
Expected: `test_cache_planner` 23/23 pass.

---

### Task 2: Extract shared public types

**Files:**
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/types.hpp`
- Modify: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/cache_planner.hpp`

- [ ] **Step 1: Create `types.hpp`**

```cpp
// Copyright (C) 2026 Miguel Ángel González Santamarta
// GPL-3.0-or-later

#ifndef OMNI_PLAN_CACHE__TYPES_HPP_
#define OMNI_PLAN_CACHE__TYPES_HPP_

#include <cstdint>
#include <cstddef>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/plan.hpp"

namespace omni_plan_cache {

/// @brief Groups object names by their PDDL type.
struct ObjectsByType {
  std::string type;
  std::vector<std::string> names;
};

/// @brief A cached plan plus the placeholder map needed to adapt object names.
struct CachedPlanData {
  omni_plan::pddl::Plan plan;
  std::unordered_map<std::string, std::string> placeholder_to_original;
};

/// @brief Backwards-compatible alias for the pre-refactor struct name.
using CachedPlan = CachedPlanData;

/// @brief Cache performance counters snapshot.
struct CacheStats {
  uint64_t exact_hits = 0;
  uint64_t exact_misses = 0;
  uint64_t structural_hits = 0;
  uint64_t full_misses = 0;
  uint64_t validations = 0;
  uint64_t adaptations = 0;
  uint64_t compositions = 0;
  uint64_t composition_fallbacks = 0;
  uint64_t exact_evictions = 0;
  uint64_t structural_evictions = 0;
  size_t exact_entries = 0;
  size_t structural_entries = 0;
};

} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__TYPES_HPP_
```

- [ ] **Step 2: Use `types.hpp` from `cache_planner.hpp`**

In `cache_planner.hpp`, replace the `struct CachedPlan { ... };` and `struct ObjectsByType { ... };` definitions with an include. The include block becomes:

```cpp
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/plan_validator.hpp"
#include "omni_plan/planner.hpp"
#include "omni_plan_cache/types.hpp"
#include "yasmin_ros/yasmin_node.hpp"
```

Delete the two struct definitions (keep the Doxygen comments for `CachedPlanData`/`ObjectsByType` in `types.hpp`).

- [ ] **Step 3: Build and test**

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select omni_plan_cache --event-handlers console_direct+
colcon test-result --verbose
```
Expected: build succeeds, 23/23 tests pass (no behavior change).

---

### Task 3: Scaffold detail headers, stubs, test targets

**Files:**
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/detail/sha256.hpp`
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/detail/structural_keyer.hpp`
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/detail/relevance_analyzer.hpp`
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/detail/plan_adapter.hpp`
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/detail/component_composer.hpp`
- Create: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/detail/plan_cache.hpp`
- Create stub sources in `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/` for each of the six units
- Create test files `test/test_cache_detail_sha256.cpp`, `test/test_cache_detail_relevance.cpp`, `test/test_cache_detail_structural.cpp`, `test/test_cache_detail_adapter.cpp`, `test/test_cache_detail_composer.cpp`, `test/test_cache_detail_plan_cache.cpp` each containing a single passing placeholder test
- Modify: `omni_plan_planners/omni_plan_cache/CMakeLists.txt`

- [ ] **Step 1: Create `detail/sha256.hpp`**

```cpp
#ifndef OMNI_PLAN_CACHE__DETAIL__SHA256_HPP_
#define OMNI_PLAN_CACHE__DETAIL__SHA256_HPP_

#include <string>
#include <string_view>

namespace omni_plan_cache {
namespace detail {

/// @brief Streaming SHA-256 digest built on OpenSSL EVP.
class Sha256 {
public:
  Sha256();
  ~Sha256();
  Sha256(const Sha256 &) = delete;
  Sha256 &operator=(const Sha256 &) = delete;

  void update(std::string_view data);
  std::string final_hex();

private:
  void *ctx_;
};

/// @brief One-shot SHA-256 hex digest.
std::string sha256_hex(std::string_view data);

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__SHA256_HPP_
```

- [ ] **Step 2: Create `detail/structural_keyer.hpp`**

```cpp
#ifndef OMNI_PLAN_CACHE__DETAIL__STRUCTURAL_KEYER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__STRUCTURAL_KEYER_HPP_

#include <set>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/types.hpp"

namespace omni_plan_cache {
namespace detail {

class Sha256;

/// @brief Result of preparing the role-normalized structural view.
struct PreparedStructure {
  std::vector<ObjectsByType> objects_by_type;
  std::unordered_map<std::string, std::string> name_to_alias;
  std::unordered_map<std::string, std::string> role_keys;
  std::unordered_map<std::string, std::string> placeholder_to_original;
};

/// @brief Pure functions for object grouping, role keys and structural digests.
class StructuralKeyer {
public:
  static std::vector<ObjectsByType>
  group_objects_by_type(const std::set<omni_plan::pddl::Object> &objects);

  static std::unordered_map<std::string, std::string> compute_role_keys(
      const std::vector<ObjectsByType> &objects_by_type,
      const std::set<omni_plan::pddl::Predicate> &facts,
      const std::set<omni_plan::pddl::Predicate> &goals,
      const std::unordered_map<std::string, std::string> *name_to_alias = nullptr,
      bool abstract_keys = false);

  static PreparedStructure
  prepare(const std::set<omni_plan::pddl::Object> &objects,
          const std::set<omni_plan::pddl::Predicate> &facts,
          const std::set<omni_plan::pddl::Predicate> &goals,
          bool abstract_keys);

  static std::string exact_key(const omni_plan::pddl::Domain &domain,
                               const omni_plan::pddl::Problem &problem);

  static std::string compute_key(
      const omni_plan::pddl::Domain &domain,
      const omni_plan::pddl::Problem &problem,
      const std::vector<ObjectsByType> &objects_by_type,
      const std::unordered_map<std::string, std::string> &role_keys,
      const std::set<omni_plan::pddl::Predicate> *filtered_facts);

  static std::string compute_key(
      const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
      const std::vector<ObjectsByType> &objects_by_type,
      const std::unordered_map<std::string, std::string> &role_keys,
      const std::set<omni_plan::pddl::Predicate> *filtered_facts);
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__STRUCTURAL_KEYER_HPP_
```

- [ ] **Step 3: Create `detail/relevance_analyzer.hpp`**

```cpp
#ifndef OMNI_PLAN_CACHE__DETAIL__RELEVANCE_ANALYZER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__RELEVANCE_ANALYZER_HPP_

#include <set>
#include <string>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"

namespace omni_plan_cache {
namespace detail {

/// @brief Predicate/object/fact sets relevant to the current goals.
struct RelevanceResult {
  std::set<std::string> relevant_predicates;
  std::set<std::string> static_predicates;
  std::set<std::string> full_static_predicates;
  std::set<std::string> relevant_objects;
  std::set<omni_plan::pddl::Predicate> relevant_facts;
};

/// @brief Backward-chaining relevance analysis with a per-call action index.
class RelevanceAnalyzer {
public:
  static std::set<std::string>
  relevant_predicates(const omni_plan::pddl::Domain &domain,
                      const omni_plan::pddl::Problem &problem);

  static RelevanceResult analyze(const omni_plan::pddl::Domain &domain,
                                 const omni_plan::pddl::Problem &problem,
                                 const std::string &robot_type);
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__RELEVANCE_ANALYZER_HPP_
```

- [ ] **Step 4: Create `detail/plan_adapter.hpp`**

```cpp
#ifndef OMNI_PLAN_CACHE__DETAIL__PLAN_ADAPTER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__PLAN_ADAPTER_HPP_

#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/plan.hpp"
#include "omni_plan_cache/types.hpp"

namespace omni_plan_cache {
namespace detail {

/// @brief Placeholder mapping and structural plan adaptation.
class PlanAdapter {
public:
  static std::unordered_map<std::string, std::string> build_name_mapping(
      const std::unordered_map<std::string, std::string>
          &old_placeholder_to_original,
      const std::vector<ObjectsByType> &new_objects_by_type);

  static omni_plan::pddl::Plan
  adapt(const CachedPlanData &cached,
        const std::unordered_map<std::string, std::string> &old_to_new);
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__PLAN_ADAPTER_HPP_
```

- [ ] **Step 5: Create `detail/component_composer.hpp`**

```cpp
#ifndef OMNI_PLAN_CACHE__DETAIL__COMPONENT_COMPOSER_HPP_
#define OMNI_PLAN_CACHE__DETAIL__COMPONENT_COMPOSER_HPP_

#include <functional>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <cstddef>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"

namespace omni_plan_cache {
namespace detail {

/// @brief Decomposes goals into independent components and stitches sub-plans.
class ComponentComposer {
public:
  struct Options {
    std::string robot_type = "robot";
    std::string priority_predicate;
    size_t max_goals_per_component = 4;
    bool parallel_independent = true;
  };

  using Solver = std::function<omni_plan::pddl::Plan(
      const omni_plan::pddl::Domain &, const omni_plan::pddl::Problem &)>;
  using Logger = std::function<void(const std::string &)>;

  ComponentComposer(Options options, Solver solver, Logger info_log = {},
                    Logger warn_log = {});

  bool compose(const omni_plan::pddl::Domain &domain,
               const omni_plan::pddl::Problem &problem,
               const std::set<omni_plan::pddl::Predicate> &relevant_facts,
               const std::set<std::string> &full_static_predicates,
               omni_plan::pddl::Plan &out_plan);

  static void apply_action_effects(
      std::set<omni_plan::pddl::Predicate> &facts,
      const std::shared_ptr<omni_plan::pddl::Action> &action,
      const std::vector<std::string> &params);

  /// @brief True when the mutable-object sets are pairwise disjoint.
  static bool
  can_solve_in_parallel(const std::vector<std::set<std::string>> &mutable_objects);

private:
  Options options_;
  Solver solver_;
  Logger info_;
  Logger warn_;
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__COMPONENT_COMPOSER_HPP_
```

- [ ] **Step 6: Create `detail/plan_cache.hpp`**

```cpp
#ifndef OMNI_PLAN_CACHE__DETAIL__PLAN_CACHE_HPP_
#define OMNI_PLAN_CACHE__DETAIL__PLAN_CACHE_HPP_

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <thread>
#include <unordered_map>

#include "omni_plan_cache/types.hpp"

namespace omni_plan_cache {
namespace detail {

/// @brief Exact + structural plan caches with bounds, metrics and single-flight.
///
/// Metric ownership: get_exact() records exact_hits/exact_misses;
/// get_structural() records structural_hits. record_structural_hit() exists
/// for followers that obtain a plan through the in-flight future instead of
/// get_structural(). All other counters are recorded by the CachePlanner.
class PlanCache {
public:
  struct Flight {
    bool leader = false;
    std::shared_future<std::shared_ptr<const CachedPlanData>> future;
    /// @brief Leader-only RAII handle. If the leader exits without calling
    /// publish()/publish_failure(), its destructor releases the in-flight
    /// entry with a null result so followers compute for themselves instead
    /// of blocking forever.
    std::shared_ptr<void> guard;
  };

  PlanCache() = default;

  void configure(size_t max_exact_entries, size_t max_structural_entries);

  std::shared_ptr<const CachedPlanData> get_exact(const std::string &key);
  std::shared_ptr<const CachedPlanData> get_structural(const std::string &key);

  void put(const std::string &exact_key, const std::string &structural_key,
           std::shared_ptr<const CachedPlanData> data);
  void put_exact(const std::string &exact_key,
                 std::shared_ptr<const CachedPlanData> data);

  Flight begin_or_join(const std::string &key);
  void publish(const std::string &key,
               std::shared_ptr<const CachedPlanData> data);
  void publish_failure(const std::string &key, std::exception_ptr error);

  void record_full_miss() {
    this->full_misses_.fetch_add(1, std::memory_order_relaxed);
  }
  void record_structural_hit() {
    this->structural_hits_.fetch_add(1, std::memory_order_relaxed);
  }
  void record_validation() {
    this->validations_.fetch_add(1, std::memory_order_relaxed);
  }
  void record_adaptation() {
    this->adaptations_.fetch_add(1, std::memory_order_relaxed);
  }
  void record_composition() {
    this->compositions_.fetch_add(1, std::memory_order_relaxed);
  }
  void record_composition_fallback() {
    this->composition_fallbacks_.fetch_add(1, std::memory_order_relaxed);
  }

  CacheStats stats() const;

private:
  struct Entry {
    std::shared_ptr<const CachedPlanData> data;
    std::atomic<uint64_t> last_use;

    Entry(std::shared_ptr<const CachedPlanData> d, uint64_t tick)
        : data(std::move(d)), last_use(tick) {}
  };

  struct CacheMap {
    std::unordered_map<std::string, Entry> entries;
    size_t max_entries = 0;
    std::atomic<uint64_t> evictions{0};
  };

  struct InFlight {
    std::promise<std::shared_ptr<const CachedPlanData>> promise;
    std::shared_future<std::shared_ptr<const CachedPlanData>> future;
    std::thread::id owner;
  };

  static void put_locked(CacheMap &map, const std::string &key,
                         const std::shared_ptr<const CachedPlanData> &data,
                         std::atomic<uint64_t> &tick);
  static void evict_if_needed(CacheMap &map);
  std::shared_ptr<const CachedPlanData> get_locked(CacheMap &map,
                                                   const std::string &key);
  void abandon(const std::string &key);

  mutable std::shared_mutex mutex_;
  CacheMap exact_;
  CacheMap structural_;

  mutable std::mutex inflight_mutex_;
  std::unordered_map<std::string, std::shared_ptr<InFlight>> inflight_;

  std::atomic<uint64_t> tick_{0};
  std::atomic<uint64_t> exact_hits_{0};
  std::atomic<uint64_t> exact_misses_{0};
  std::atomic<uint64_t> structural_hits_{0};
  std::atomic<uint64_t> full_misses_{0};
  std::atomic<uint64_t> validations_{0};
  std::atomic<uint64_t> adaptations_{0};
  std::atomic<uint64_t> compositions_{0};
  std::atomic<uint64_t> composition_fallbacks_{0};
};

} // namespace detail
} // namespace omni_plan_cache

#endif // OMNI_PLAN_CACHE__DETAIL__PLAN_CACHE_HPP_
```

- [ ] **Step 7: Create stub sources**

Create each stub `.cpp` in `src/omni_plan_cache/detail/` with exactly these contents.

`sha256.cpp`:

```cpp
#include "omni_plan_cache/detail/sha256.hpp"

namespace omni_plan_cache {
namespace detail {

Sha256::Sha256() : ctx_(nullptr) {}
Sha256::~Sha256() = default;
void Sha256::update(std::string_view) {}
std::string Sha256::final_hex() { return ""; }
std::string sha256_hex(std::string_view) { return ""; }

} // namespace detail
} // namespace omni_plan_cache
```

`structural_keyer.cpp`:

```cpp
#include "omni_plan_cache/detail/structural_keyer.hpp"

namespace omni_plan_cache {
namespace detail {

std::vector<ObjectsByType> StructuralKeyer::group_objects_by_type(
    const std::set<omni_plan::pddl::Object> &) {
  return {};
}

std::unordered_map<std::string, std::string> StructuralKeyer::compute_role_keys(
    const std::vector<ObjectsByType> &, const std::set<omni_plan::pddl::Predicate> &,
    const std::set<omni_plan::pddl::Predicate> &,
    const std::unordered_map<std::string, std::string> *, bool) {
  return {};
}

PreparedStructure StructuralKeyer::prepare(
    const std::set<omni_plan::pddl::Object> &,
    const std::set<omni_plan::pddl::Predicate> &,
    const std::set<omni_plan::pddl::Predicate> &, bool) {
  return {};
}

std::string StructuralKeyer::exact_key(const omni_plan::pddl::Domain &,
                                       const omni_plan::pddl::Problem &) {
  return "";
}

std::string StructuralKeyer::compute_key(
    const omni_plan::pddl::Domain &, const omni_plan::pddl::Problem &,
    const std::vector<ObjectsByType> &,
    const std::unordered_map<std::string, std::string> &,
    const std::set<omni_plan::pddl::Predicate> *) {
  return "";
}

std::string StructuralKeyer::compute_key(
    const std::string &, const omni_plan::pddl::Problem &,
    const std::vector<ObjectsByType> &,
    const std::unordered_map<std::string, std::string> &,
    const std::set<omni_plan::pddl::Predicate> *) {
  return "";
}

} // namespace detail
} // namespace omni_plan_cache
```

`relevance_analyzer.cpp`:

```cpp
#include "omni_plan_cache/detail/relevance_analyzer.hpp"

namespace omni_plan_cache {
namespace detail {

std::set<std::string> RelevanceAnalyzer::relevant_predicates(
    const omni_plan::pddl::Domain &, const omni_plan::pddl::Problem &) {
  return {};
}

RelevanceResult RelevanceAnalyzer::analyze(const omni_plan::pddl::Domain &,
                                           const omni_plan::pddl::Problem &,
                                           const std::string &) {
  return {};
}

} // namespace detail
} // namespace omni_plan_cache
```

`plan_adapter.cpp`:

```cpp
#include "omni_plan_cache/detail/plan_adapter.hpp"

namespace omni_plan_cache {
namespace detail {

std::unordered_map<std::string, std::string> PlanAdapter::build_name_mapping(
    const std::unordered_map<std::string, std::string> &,
    const std::vector<ObjectsByType> &) {
  return {};
}

omni_plan::pddl::Plan PlanAdapter::adapt(
    const CachedPlanData &,
    const std::unordered_map<std::string, std::string> &) {
  return {};
}

} // namespace detail
} // namespace omni_plan_cache
```

`component_composer.cpp`:

```cpp
#include "omni_plan_cache/detail/component_composer.hpp"

namespace omni_plan_cache {
namespace detail {

ComponentComposer::ComponentComposer(Options options, Solver solver,
                                     Logger info_log, Logger warn_log)
    : options_(std::move(options)), solver_(std::move(solver)),
      info_(std::move(info_log)), warn_(std::move(warn_log)) {}

bool ComponentComposer::can_solve_in_parallel(
    const std::vector<std::set<std::string>> &) {
  return false;
}

void ComponentComposer::apply_action_effects(
    std::set<omni_plan::pddl::Predicate> &,
    const std::shared_ptr<omni_plan::pddl::Action> &,
    const std::vector<std::string> &) {}

bool ComponentComposer::compose(const omni_plan::pddl::Domain &,
                                const omni_plan::pddl::Problem &,
                                const std::set<omni_plan::pddl::Predicate> &,
                                const std::set<std::string> &,
                                omni_plan::pddl::Plan &) {
  return false;
}

} // namespace detail
} // namespace omni_plan_cache
```

`plan_cache.cpp`:

```cpp
#include "omni_plan_cache/detail/plan_cache.hpp"

namespace omni_plan_cache {
namespace detail {

void PlanCache::configure(size_t, size_t) {}

std::shared_ptr<const CachedPlanData> PlanCache::get_exact(const std::string &) {
  return nullptr;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_structural(const std::string &) {
  return nullptr;
}

void PlanCache::put(const std::string &, const std::string &,
                    std::shared_ptr<const CachedPlanData>) {}

void PlanCache::put_exact(const std::string &,
                          std::shared_ptr<const CachedPlanData>) {}

PlanCache::Flight PlanCache::begin_or_join(const std::string &) {
  return Flight{true, {}};
}

void PlanCache::publish(const std::string &,
                        std::shared_ptr<const CachedPlanData>) {}

void PlanCache::publish_failure(const std::string &, std::exception_ptr) {}

void PlanCache::abandon(const std::string &) {}

void PlanCache::put_locked(CacheMap &, const std::string &,
                           const std::shared_ptr<const CachedPlanData> &,
                           std::atomic<uint64_t> &) {}

void PlanCache::evict_if_needed(CacheMap &) {}

std::shared_ptr<const CachedPlanData>
PlanCache::get_locked(CacheMap &, const std::string &) {
  return nullptr;
}

CacheStats PlanCache::stats() const { return {}; }

} // namespace detail
} // namespace omni_plan_cache
```

`CachePlanner` continues to use its existing implementation, so existing tests stay green.

- [ ] **Step 8: Add sources and test targets to CMake**

Change `add_library(cache_planner SHARED src/omni_plan_cache/cache_planner.cpp)` to:

```cmake
add_library(cache_planner SHARED
  src/omni_plan_cache/cache_planner.cpp
  src/omni_plan_cache/detail/sha256.cpp
  src/omni_plan_cache/detail/structural_keyer.cpp
  src/omni_plan_cache/detail/relevance_analyzer.cpp
  src/omni_plan_cache/detail/plan_adapter.cpp
  src/omni_plan_cache/detail/component_composer.cpp
  src/omni_plan_cache/detail/plan_cache.cpp
)
```

Inside `if(BUILD_TESTING)`, after the existing `test_cache_planner` block, add a small helper and the six test targets:

```cmake
  foreach(detail_test
      test_cache_detail_sha256
      test_cache_detail_relevance
      test_cache_detail_structural
      test_cache_detail_adapter
      test_cache_detail_composer
      test_cache_detail_plan_cache)
    ament_add_gtest(${detail_test} test/${detail_test}.cpp)
    target_include_directories(${detail_test} PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
      $<INSTALL_INTERFACE:include>
    )
    target_link_libraries(${detail_test}
      cache_planner
      ${DEPENDENCIES}
    )
  endforeach()
```

Each placeholder test file contains:

```cpp
#include <gtest/gtest.h>

TEST(Placeholder, Compiles) { EXPECT_TRUE(true); }
```

- [ ] **Step 9: Build and test**

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select omni_plan_cache --event-handlers console_direct+
colcon test-result --verbose
```
Expected: everything builds; 23 legacy tests + 6 placeholder tests pass.

**After this task, Tasks 4–9 can run in parallel (each owns disjoint files).**

---

### Task 4: Implement Sha256

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/sha256.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/test/test_cache_detail_sha256.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/cache_planner.cpp` (delegate `sha256`)

- [ ] **Step 1: Write the failing tests**

Replace the placeholder test file with:

```cpp
#include <gtest/gtest.h>

#include "omni_plan_cache/cache_planner.hpp"
#include "omni_plan_cache/detail/sha256.hpp"

using omni_plan_cache::detail::Sha256;
using omni_plan_cache::detail::sha256_hex;

TEST(Sha256Test, KnownVector) {
  EXPECT_EQ(sha256_hex("abc"),
            "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
}

TEST(Sha256Test, EmptyVector) {
  EXPECT_EQ(sha256_hex(""),
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
}

TEST(Sha256Test, StreamingMatchesOneShot) {
  Sha256 h;
  h.update("hello ");
  h.update("world");
  EXPECT_EQ(h.final_hex(), sha256_hex("hello world"));
}

TEST(Sha256Test, CachePlannerHelperMatches) {
  EXPECT_EQ(omni_plan_cache::CachePlanner::sha256("hello"), sha256_hex("hello"));
}
```

- [ ] **Step 2: Build and verify the tests fail**

Run `colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON` and `./build/omni_plan_cache/test_cache_detail_sha256`.
Expected: FAIL (stub returns `""`).

- [ ] **Step 3: Implement `sha256.cpp`**

```cpp
#include "omni_plan_cache/detail/sha256.hpp"

#include <openssl/evp.h>

#include <stdexcept>

namespace omni_plan_cache {
namespace detail {

namespace {
constexpr char kHex[] = "0123456789abcdef";
} // namespace

Sha256::Sha256() : ctx_(EVP_MD_CTX_new()) {
  if (this->ctx_ == nullptr) {
    throw std::runtime_error("Sha256: EVP_MD_CTX_new failed");
  }
  if (EVP_DigestInit_ex(static_cast<EVP_MD_CTX *>(this->ctx_), EVP_sha256(),
                        nullptr) != 1) {
    EVP_MD_CTX_free(static_cast<EVP_MD_CTX *>(this->ctx_));
    this->ctx_ = nullptr;
    throw std::runtime_error("Sha256: init failed");
  }
}

Sha256::~Sha256() {
  if (this->ctx_ != nullptr) {
    EVP_MD_CTX_free(static_cast<EVP_MD_CTX *>(this->ctx_));
  }
}

void Sha256::update(std::string_view data) {
  EVP_DigestUpdate(static_cast<EVP_MD_CTX *>(this->ctx_), data.data(),
                   data.size());
}

std::string Sha256::final_hex() {
  unsigned char digest[EVP_MAX_MD_SIZE];
  unsigned int len = 0;
  if (EVP_DigestFinal_ex(static_cast<EVP_MD_CTX *>(this->ctx_), digest, &len) !=
      1) {
    throw std::runtime_error("Sha256: final failed");
  }
  std::string out(len * 2, '\0');
  for (unsigned int i = 0; i < len; ++i) {
    out[2 * i] = kHex[digest[i] >> 4];
    out[2 * i + 1] = kHex[digest[i] & 0x0F];
  }
  return out;
}

std::string sha256_hex(std::string_view data) {
  Sha256 h;
  h.update(data);
  return h.final_hex();
}

} // namespace detail
} // namespace omni_plan_cache
```

- [ ] **Step 4: Delegate `CachePlanner::sha256`**

In `cache_planner.cpp`, add `#include "omni_plan_cache/detail/sha256.hpp"` and replace the body of `CachePlanner::sha256` with:

```cpp
std::string CachePlanner::sha256(const std::string &input) {
  return detail::sha256_hex(input);
}
```

Remove the now-unused `#include <openssl/sha.h>`, `<iomanip>`, `<sstream>` includes if nothing else uses them.

- [ ] **Step 5: Build and run tests**

Run the build and `./build/omni_plan_cache/test_cache_detail_sha256` then the full `colcon test`.
Expected: sha256 tests pass; 23 legacy tests pass.

---

### Task 5: Implement RelevanceAnalyzer

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/relevance_analyzer.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/test/test_cache_detail_relevance.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/cache_planner.cpp` (delegate `compute_relevant_predicates`)

- [ ] **Step 1: Write the failing tests**

```cpp
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/relevance_analyzer.hpp"

using namespace omni_plan;
using namespace omni_plan_cache::detail;

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

pddl::Domain make_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_type("item");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("connected", {"?l1", "?l2"}));
  domain.add_predicate(pddl::Predicate("item_at", {"?i", "?l"}));

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

} // namespace

TEST(RelevanceAnalyzerTest, FiltersIrrelevantPredicatesAndFacts) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("loc1", "location"));
  problem.add_object(pddl::Object("loc2", "location"));
  problem.add_object(pddl::Object("item1", "item"));
  problem.add_fact(pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_fact(pddl::Predicate("item_at", {"item1", "loc1"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "loc2"}));

  const auto result = RelevanceAnalyzer::analyze(domain, problem, "robot");

  EXPECT_TRUE(result.relevant_predicates.count("at") == 1);
  EXPECT_TRUE(result.relevant_predicates.count("connected") == 1);
  EXPECT_TRUE(result.relevant_predicates.count("item_at") == 0);

  EXPECT_TRUE(result.full_static_predicates.count("connected") == 1);
  EXPECT_TRUE(result.full_static_predicates.count("item_at") == 1);
  EXPECT_TRUE(result.full_static_predicates.count("at") == 0);

  EXPECT_TRUE(result.static_predicates.count("connected") == 1);
  EXPECT_TRUE(result.static_predicates.count("at") == 0);

  EXPECT_TRUE(result.relevant_objects.count("robot1") == 1);
  EXPECT_TRUE(result.relevant_objects.count("loc1") == 1);
  EXPECT_TRUE(result.relevant_objects.count("loc2") == 1);

  EXPECT_TRUE(result.relevant_facts.count(
                  pddl::Predicate("at", {"robot1", "loc1"})) == 1);
  EXPECT_TRUE(result.relevant_facts.count(
                  pddl::Predicate("connected", {"loc1", "loc2"})) == 1);
  EXPECT_TRUE(result.relevant_facts.count(
                  pddl::Predicate("item_at", {"item1", "loc1"})) == 0);
}

TEST(RelevanceAnalyzerTest, RobotTypeControlsRobotInclusion) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("loc1", "location"));
  problem.add_object(pddl::Object("loc2", "location"));
  problem.add_fact(pddl::Predicate("at", {"robot1", "loc1"}));
  problem.add_fact(pddl::Predicate("connected", {"loc1", "loc2"}));
  problem.add_goal(pddl::Predicate("at", {"robot1", "loc2"}));

  const auto with_robot = RelevanceAnalyzer::analyze(domain, problem, "robot");
  const auto without_robot =
      RelevanceAnalyzer::analyze(domain, problem, "nonexistent");
  EXPECT_EQ(with_robot.relevant_objects.count("robot1"), 1u);
  EXPECT_EQ(without_robot.relevant_objects.count("robot1"), 0u);
}
```

- [ ] **Step 2: Build and verify the tests fail**

Run `colcon build ...` then `./build/omni_plan_cache/test_cache_detail_relevance`.
Expected: FAIL (stub returns empty sets).

- [ ] **Step 3: Implement `relevance_analyzer.cpp`**

```cpp
#include "omni_plan_cache/detail/relevance_analyzer.hpp"

#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"

namespace omni_plan_cache {
namespace detail {

namespace {

using ActionPtr = std::shared_ptr<omni_plan::pddl::Action>;

std::set<std::string> compute_relevant(
    const std::unordered_map<std::string, std::vector<const ActionPtr *>>
        &effect_index,
    const std::unordered_map<const omni_plan::pddl::Action *,
                             std::vector<std::string>> &conditions,
    const std::set<omni_plan::pddl::Predicate> &goals) {

  std::set<std::string> relevant;
  std::vector<std::string> frontier;
  for (const auto &goal : goals) {
    frontier.push_back(goal.get_name());
  }

  while (!frontier.empty()) {
    std::string current = frontier.back();
    frontier.pop_back();
    if (!relevant.insert(current).second) {
      continue;
    }
    auto it = effect_index.find(current);
    if (it == effect_index.end()) {
      continue;
    }
    for (const auto *action : it->second) {
      auto cond_it = conditions.find(action->get());
      if (cond_it == conditions.end()) {
        continue;
      }
      for (const auto &name : cond_it->second) {
        if (!relevant.count(name)) {
          frontier.push_back(name);
        }
      }
    }
  }
  return relevant;
}

} // namespace

std::set<std::string>
RelevanceAnalyzer::relevant_predicates(const omni_plan::pddl::Domain &domain,
                                       const omni_plan::pddl::Problem &problem) {
  std::unordered_map<std::string, std::vector<const ActionPtr *>> effect_index;
  std::unordered_map<const omni_plan::pddl::Action *, std::vector<std::string>>
      conditions;

  for (const auto &[name, action] : domain.get_actions()) {
    std::vector<std::string> cond_names;
    for (const auto &cond : action->get_conditions()) {
      cond_names.push_back(cond.get_name());
    }
    conditions[action.get()] = std::move(cond_names);
    for (const auto &effect : action->get_effects()) {
      effect_index[effect.get_name()].push_back(&action);
    }
  }

  return compute_relevant(effect_index, conditions, problem.get_goals());
}

RelevanceResult RelevanceAnalyzer::analyze(
    const omni_plan::pddl::Domain &domain, const omni_plan::pddl::Problem &problem,
    const std::string &robot_type) {

  RelevanceResult result;
  result.relevant_predicates = relevant_predicates(domain, problem);

  for (const auto &pred : domain.get_predicates()) {
    result.full_static_predicates.insert(pred.get_name());
  }
  for (const auto &[name, action] : domain.get_actions()) {
    for (const auto &effect : action->get_effects()) {
      result.full_static_predicates.erase(effect.get_name());
    }
  }
  for (const auto &pred : result.relevant_predicates) {
    if (result.full_static_predicates.count(pred)) {
      result.static_predicates.insert(pred);
    }
  }

  for (const auto &goal : problem.get_goals()) {
    for (const auto &arg : goal.get_args()) {
      result.relevant_objects.insert(arg);
    }
  }
  for (const auto &obj : problem.get_objects()) {
    if (obj.get_type() == robot_type) {
      result.relevant_objects.insert(obj.get_name());
    }
  }
  for (const auto &fact : problem.get_facts()) {
    if (!result.static_predicates.count(fact.get_name())) {
      continue;
    }
    for (const auto &arg : fact.get_args()) {
      result.relevant_objects.insert(arg);
    }
  }

  for (const auto &fact : problem.get_facts()) {
    if (!result.relevant_predicates.count(fact.get_name())) {
      continue;
    }
    if (result.static_predicates.count(fact.get_name())) {
      result.relevant_facts.insert(fact);
      continue;
    }
    bool keep = false;
    for (const auto &arg : fact.get_args()) {
      if (result.relevant_objects.count(arg)) {
        keep = true;
        break;
      }
    }
    if (keep) {
      result.relevant_facts.insert(fact);
    }
  }

  return result;
}

} // namespace detail
} // namespace omni_plan_cache
```

- [ ] **Step 4: Delegate `CachePlanner::compute_relevant_predicates`**

Replace the body of `CachePlanner::compute_relevant_predicates` in `cache_planner.cpp` with:

```cpp
std::set<std::string> CachePlanner::compute_relevant_predicates(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem) {
  return detail::RelevanceAnalyzer::relevant_predicates(domain, problem);
}
```

Add `#include "omni_plan_cache/detail/relevance_analyzer.hpp"`.

- [ ] **Step 5: Build and test**

Run the build, the relevance test binary, then full `colcon test`.
Expected: relevance tests pass; all legacy tests still pass (the relevance integration tests exercise the delegate path).

---

### Task 6: Implement StructuralKeyer

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/structural_keyer.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/test/test_cache_detail_structural.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/cache_planner.cpp` (delegate `group_objects_by_type`, `compute_role_keys`, `compute_structural_key`)

- [ ] **Step 1: Write the failing tests**

```cpp
#include <gtest/gtest.h>

#include <set>
#include <string>

#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"

using namespace omni_plan;

TEST(StructuralKeyerTest, AbstractRoleKeys) {
  std::set<pddl::Object> objects = {
      pddl::Object("robot1", "robot"), pddl::Object("loc1", "location"),
      pddl::Object("loc2", "location"), pddl::Object("loc3", "location")};
  auto groups = omni_plan_cache::detail::StructuralKeyer::group_objects_by_type(
      objects);
  std::set<pddl::Predicate> facts = {
      pddl::Predicate("at", {"robot1", "loc1"}),
      pddl::Predicate("connected", {"loc1", "loc2"}),
      pddl::Predicate("connected", {"loc2", "loc3"})};
  std::set<pddl::Predicate> goals = {
      pddl::Predicate("at", {"robot1", "loc3"})};

  auto keys = omni_plan_cache::detail::StructuralKeyer::compute_role_keys(
      groups, facts, goals);
  EXPECT_EQ(keys["robot1"], "at_0_0|at_0_1|");
  EXPECT_EQ(keys["loc1"], "at_1_0|connected_0_0|");
  EXPECT_EQ(keys["loc2"], "connected_0_0|connected_1_0|");
  EXPECT_EQ(keys["loc3"], "at_1_1|connected_1_0|");
}

TEST(StructuralKeyerTest, PrepareFiltersAndSorts) {
  std::set<pddl::Object> objects = {
      pddl::Object("robot1", "robot"), pddl::Object("loc1", "location"),
      pddl::Object("loc2", "location"), pddl::Object("unused", "location")};
  std::set<pddl::Predicate> facts = {
      pddl::Predicate("at", {"robot1", "loc1"}),
      pddl::Predicate("connected", {"loc1", "loc2"})};
  std::set<pddl::Predicate> goals = {
      pddl::Predicate("at", {"robot1", "loc2"})};

  auto prepared = omni_plan_cache::detail::StructuralKeyer::prepare(
      objects, facts, goals, false);

  ASSERT_EQ(prepared.objects_by_type.size(), 2u);
  for (const auto &group : prepared.objects_by_type) {
    if (group.type == "location") {
      ASSERT_EQ(group.names.size(), 2u);
      EXPECT_EQ(group.names[0], "loc1");
      EXPECT_EQ(group.names[1], "loc2");
    }
  }
  EXPECT_EQ(prepared.placeholder_to_original.at("__obj_location_0__"), "loc1");
  EXPECT_EQ(prepared.placeholder_to_original.at("__obj_location_1__"), "loc2");
  EXPECT_EQ(prepared.name_to_alias.at("loc1"), "location_0");
  EXPECT_FALSE(prepared.role_keys.at("loc1").empty());
}

TEST(StructuralKeyerTest, ExactKeyDetectsDifferentFacts) {
  pddl::Domain domain;
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  pddl::Problem a;
  a.add_object(pddl::Object("r", "robot"));
  a.add_object(pddl::Object("l1", "location"));
  a.add_object(pddl::Object("l2", "location"));
  a.add_fact(pddl::Predicate("at", {"r", "l1"}));
  a.add_goal(pddl::Predicate("at", {"r", "l2"}));

  pddl::Problem b = a;
  b.add_fact(pddl::Predicate("at", {"r", "l2"}));

  EXPECT_EQ(omni_plan_cache::detail::StructuralKeyer::exact_key(domain, a),
            omni_plan_cache::detail::StructuralKeyer::exact_key(domain, a));
  EXPECT_NE(omni_plan_cache::detail::StructuralKeyer::exact_key(domain, a),
            omni_plan_cache::detail::StructuralKeyer::exact_key(domain, b));
}
```

- [ ] **Step 2: Build and verify the tests fail**

Run the build and `./build/omni_plan_cache/test_cache_detail_structural`.
Expected: FAIL (stubs).

- [ ] **Step 3: Implement `structural_keyer.cpp`**

```cpp
#include "omni_plan_cache/detail/structural_keyer.hpp"

#include <algorithm>
#include <memory>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/timing_predicate.hpp"
#include "omni_plan_cache/detail/sha256.hpp"

namespace omni_plan_cache {
namespace detail {

namespace {

namespace pddl = omni_plan::pddl;

constexpr const char *kSeparator = "|";

struct Contribution {
  std::string base;
  std::vector<std::string> co_args;
};

using Contributions = std::unordered_map<std::string, std::vector<Contribution>>;

Contributions collect_contributions(const std::set<pddl::Predicate> &facts,
                                    const std::set<pddl::Predicate> &goals) {
  Contributions out;
  auto add = [&out](const std::set<pddl::Predicate> &preds, bool is_goal) {
    for (const auto &pred : preds) {
      const auto args = pred.get_args();
      for (size_t i = 0; i < args.size(); ++i) {
        Contribution contribution;
        contribution.base = pred.get_name() + "_" + std::to_string(i) + "_" +
                            (is_goal ? "1" : "0");
        contribution.co_args.reserve(args.size() - 1);
        for (size_t j = 0; j < args.size(); ++j) {
          if (j != i) {
            contribution.co_args.push_back(args[j]);
          }
        }
        out[args[i]].push_back(std::move(contribution));
      }
    }
  };
  add(facts, false);
  add(goals, true);
  return out;
}

std::unordered_map<std::string, std::string> build_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const Contributions &contributions,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {

  std::unordered_map<std::string, std::string> result;
  for (const auto &group : objects_by_type) {
    for (const auto &name : group.names) {
      auto it = contributions.find(name);
      if (it == contributions.end()) {
        result[name].clear();
        continue;
      }
      std::vector<std::string> entries;
      entries.reserve(it->second.size());
      for (const auto &contribution : it->second) {
        std::string entry = contribution.base;
        if (name_to_alias != nullptr && !abstract_keys) {
          for (const auto &co_arg : contribution.co_args) {
            auto alias_it = name_to_alias->find(co_arg);
            entry += "_" + (alias_it != name_to_alias->end() ? alias_it->second
                                                             : co_arg);
          }
        }
        entries.push_back(std::move(entry));
      }
      std::sort(entries.begin(), entries.end());
      std::string key;
      for (const auto &entry : entries) {
        key += entry;
        key += "|";
      }
      result[name] = std::move(key);
    }
  }
  return result;
}

void update_predicate(Sha256 &hash, const pddl::Predicate &pred) {
  hash.update("p:" + pred.get_name());
  for (const auto &arg : pred.get_args()) {
    hash.update("?" + arg);
  }
  hash.update(pred.is_negated() ? "!|" : "|");
}

void update_timing_predicate(Sha256 &hash, const pddl::TimingPredicate &pred) {
  hash.update("t:" + std::to_string(static_cast<int>(pred.get_type())) + ":");
  update_predicate(hash, pred);
}

void update_action(Sha256 &hash, const pddl::Action &action) {
  hash.update("a:" + action.get_name() + ":" +
              std::to_string(action.get_duration()) + "|");
  for (const auto &param : action.get_parameters()) {
    hash.update("param:" + param.get_name() + ":" + param.get_type() + "|");
  }
  for (const auto &condition : action.get_conditions()) {
    update_timing_predicate(hash, condition);
  }
  for (const auto &effect : action.get_effects()) {
    update_timing_predicate(hash, effect);
  }
}

void update_domain(Sha256 &hash, const pddl::Domain &domain) {
  hash.update("D|");
  for (const auto &requirement : domain.get_requirements()) {
    hash.update("req:" + requirement + "|");
  }
  for (const auto &type : domain.get_types()) {
    hash.update("type:" + type + "|");
  }
  for (const auto &pred : domain.get_predicates()) {
    update_predicate(hash, pred);
  }
  for (const auto &[name, action] : domain.get_actions()) {
    update_action(hash, *action);
  }
}

void update_problem(Sha256 &hash, const pddl::Problem &problem) {
  hash.update("P|");
  for (const auto &obj : problem.get_objects()) {
    hash.update("obj:" + obj.get_name() + ":" + obj.get_type() + "|");
  }
  for (const auto &fact : problem.get_facts()) {
    update_predicate(hash, fact);
  }
  for (const auto &goal : problem.get_goals()) {
    update_predicate(hash, goal);
  }
}

void update_abstraction(
    Sha256 &hash, const pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<pddl::Predicate> *filtered_facts) {

  std::unordered_map<std::string, std::string> name_to_type;
  for (const auto &group : objects_by_type) {
    for (const auto &name : group.names) {
      name_to_type[name] = group.type;
    }
  }

  auto type_signature = [&name_to_type](const pddl::Predicate &pred) {
    std::string signature = pred.get_name();
    for (const auto &arg : pred.get_args()) {
      auto it = name_to_type.find(arg);
      signature += ":" + (it != name_to_type.end() ? it->second : arg);
    }
    return signature;
  };

  for (const auto &group : objects_by_type) {
    hash.update(group.type + ":" + std::to_string(group.names.size()) +
                kSeparator);
  }

  std::vector<std::string> entries;
  const auto &facts = filtered_facts != nullptr ? *filtered_facts
                                                : problem.get_facts();
  entries.reserve(facts.size());
  for (const auto &fact : facts) {
    entries.push_back(type_signature(fact));
  }
  std::sort(entries.begin(), entries.end());
  for (const auto &entry : entries) {
    hash.update("I:" + entry + kSeparator);
  }

  entries.clear();
  entries.reserve(problem.get_goals().size());
  for (const auto &goal : problem.get_goals()) {
    entries.push_back(type_signature(goal));
  }
  std::sort(entries.begin(), entries.end());
  for (const auto &entry : entries) {
    hash.update("G:" + entry + kSeparator);
  }

  std::vector<std::string> roles;
  roles.reserve(role_keys.size());
  for (const auto &[name, key] : role_keys) {
    roles.push_back(key);
  }
  std::sort(roles.begin(), roles.end());
  for (const auto &role : roles) {
    hash.update("R:" + role + kSeparator);
  }
}

} // namespace

std::vector<ObjectsByType> StructuralKeyer::group_objects_by_type(
    const std::set<omni_plan::pddl::Object> &objects) {
  std::unordered_map<std::string, std::vector<std::string>> type_to_names;
  std::vector<std::string> type_order;

  for (const auto &obj : objects) {
    const auto &type = obj.get_type();
    if (type_to_names.find(type) == type_to_names.end()) {
      type_order.push_back(type);
    }
    type_to_names[type].push_back(obj.get_name());
  }

  std::sort(type_order.begin(), type_order.end());

  std::vector<ObjectsByType> result;
  result.reserve(type_order.size());
  for (const auto &type : type_order) {
    result.push_back({type, type_to_names[type]});
  }
  return result;
}

std::unordered_map<std::string, std::string> StructuralKeyer::compute_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const std::set<omni_plan::pddl::Predicate> &facts,
    const std::set<omni_plan::pddl::Predicate> &goals,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {
  return build_role_keys(objects_by_type, collect_contributions(facts, goals),
                         name_to_alias, abstract_keys);
}

PreparedStructure StructuralKeyer::prepare(
    const std::set<omni_plan::pddl::Object> &objects,
    const std::set<omni_plan::pddl::Predicate> &facts,
    const std::set<omni_plan::pddl::Predicate> &goals, bool abstract_keys) {

  const auto objects_by_type = group_objects_by_type(objects);
  const auto contributions = collect_contributions(facts, goals);
  const auto abstract_keys_map =
      build_role_keys(objects_by_type, contributions, nullptr, true);

  PreparedStructure out;
  for (const auto &group : objects_by_type) {
    ObjectsByType filtered;
    filtered.type = group.type;
    for (const auto &name : group.names) {
      auto it = abstract_keys_map.find(name);
      if (it != abstract_keys_map.end() && !it->second.empty()) {
        filtered.names.push_back(name);
      }
    }
    if (!filtered.names.empty()) {
      out.objects_by_type.push_back(std::move(filtered));
    }
  }

  for (auto &group : out.objects_by_type) {
    std::sort(group.names.begin(), group.names.end(),
              [&abstract_keys_map](const std::string &a, const std::string &b) {
                const std::string &key_a = abstract_keys_map.at(a);
                const std::string &key_b = abstract_keys_map.at(b);
                if (key_a != key_b) {
                  return key_a < key_b;
                }
                return a < b;
              });
  }

  for (const auto &group : out.objects_by_type) {
    for (size_t i = 0; i < group.names.size(); ++i) {
      out.name_to_alias[group.names[i]] =
          group.type + "_" + std::to_string(i);
      out.placeholder_to_original["__obj_" + group.type + "_" +
                                  std::to_string(i) + "__"] = group.names[i];
    }
  }

  out.role_keys = build_role_keys(out.objects_by_type, contributions,
                                  &out.name_to_alias, abstract_keys);
  return out;
}

std::string StructuralKeyer::exact_key(const omni_plan::pddl::Domain &domain,
                                       const omni_plan::pddl::Problem &problem) {
  Sha256 hash;
  update_domain(hash, domain);
  hash.update("|EXACT|");
  update_problem(hash, problem);
  return hash.final_hex();
}

std::string StructuralKeyer::compute_key(
    const omni_plan::pddl::Domain &domain, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<omni_plan::pddl::Predicate> *filtered_facts) {
  Sha256 hash;
  update_domain(hash, domain);
  hash.update("|ABSTRACT|");
  update_abstraction(hash, problem, objects_by_type, role_keys, filtered_facts);
  return hash.final_hex();
}

std::string StructuralKeyer::compute_key(
    const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<omni_plan::pddl::Predicate> *filtered_facts) {
  Sha256 hash;
  hash.update(domain_pddl);
  hash.update("|ABSTRACT|");
  update_abstraction(hash, problem, objects_by_type, role_keys, filtered_facts);
  return hash.final_hex();
}

} // namespace detail
} // namespace omni_plan_cache
```

- [ ] **Step 4: Delegate the public statics**

In `cache_planner.cpp`, add `#include "omni_plan_cache/detail/structural_keyer.hpp"` and replace the bodies of `group_objects_by_type`, `compute_role_keys`, and `compute_structural_key` with one-line delegations:

```cpp
std::vector<ObjectsByType> CachePlanner::group_objects_by_type(
    const std::set<omni_plan::pddl::Object> &objects) {
  return detail::StructuralKeyer::group_objects_by_type(objects);
}

std::unordered_map<std::string, std::string> CachePlanner::compute_role_keys(
    const std::vector<ObjectsByType> &objects_by_type,
    const std::set<omni_plan::pddl::Predicate> &facts,
    const std::set<omni_plan::pddl::Predicate> &goals,
    const std::unordered_map<std::string, std::string> *name_to_alias,
    bool abstract_keys) {
  return detail::StructuralKeyer::compute_role_keys(
      objects_by_type, facts, goals, name_to_alias, abstract_keys);
}

std::string CachePlanner::compute_structural_key(
    const std::string &domain_pddl, const omni_plan::pddl::Problem &problem,
    const std::vector<ObjectsByType> &objects_by_type,
    const std::unordered_map<std::string, std::string> &role_keys,
    const std::set<omni_plan::pddl::Predicate> *filtered_facts) {
  return detail::StructuralKeyer::compute_key(domain_pddl, problem,
                                              objects_by_type, role_keys,
                                              filtered_facts);
}
```

- [ ] **Step 5: Build and test**

Run the build, the structural test binary, then full `colcon test`.
Expected: structural tests pass; all 23 legacy tests pass (including the exact role-key strings and structural equality/inequality assertions).

---

### Task 7: Implement PlanAdapter (fixes `raw_output` loss)

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/plan_adapter.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/test/test_cache_detail_adapter.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/cache_planner.cpp` (delegate `build_name_mapping`, use `adapt` in the structural-hit block)

- [ ] **Step 1: Write the failing tests**

```cpp
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan_cache/detail/plan_adapter.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;
using namespace omni_plan_cache::detail;

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

} // namespace

TEST(PlanAdapterTest, BuildNameMappingAlignsPlaceholders) {
  std::unordered_map<std::string, std::string> old = {
      {"__obj_robot_0__", "robot1"},
      {"__obj_location_0__", "kitchen"},
      {"__obj_location_1__", "dining_room"}};
  std::vector<ObjectsByType> new_objects = {
      {"robot", {"r2"}},
      {"location", {"lab", "office"}}};

  auto mapping = PlanAdapter::build_name_mapping(old, new_objects);
  EXPECT_EQ(mapping["robot1"], "r2");
  EXPECT_EQ(mapping["kitchen"], "lab");
  EXPECT_EQ(mapping["dining_room"], "office");
}

TEST(PlanAdapterTest, AdaptRenamesParamsAndKeepsRawOutput) {
  auto action = std::make_shared<MockAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
  CachedPlanData cached;
  cached.plan.set_has_solution(true);
  cached.plan.set_raw_output("0.000: (move robot1 kitchen dining) [10.000]\n");
  cached.plan.add_action(action, {"robot1", "kitchen", "dining"}, 0.0f);

  auto adapted = PlanAdapter::adapt(cached, {{"robot1", "r2"},
                                             {"kitchen", "lab"},
                                             {"dining", "office"}});

  EXPECT_TRUE(adapted.has_solution());
  ASSERT_EQ(adapted.size(), 1u);
  const auto params = adapted.get_action_params(0);
  EXPECT_EQ(params[0], "r2");
  EXPECT_EQ(params[1], "lab");
  EXPECT_EQ(params[2], "office");
  EXPECT_FALSE(adapted.get_raw_output().empty());
}
```

- [ ] **Step 2: Build and verify the tests fail**

Run the build and `./build/omni_plan_cache/test_cache_detail_adapter`.
Expected: `AdaptRenamesParamsAndKeepsRawOutput` FAILS (stub plan has no actions); mapping test fails too.

- [ ] **Step 3: Implement `plan_adapter.cpp`**

```cpp
#include "omni_plan_cache/detail/plan_adapter.hpp"

namespace omni_plan_cache {
namespace detail {

std::unordered_map<std::string, std::string> PlanAdapter::build_name_mapping(
    const std::unordered_map<std::string, std::string>
        &old_placeholder_to_original,
    const std::vector<ObjectsByType> &new_objects_by_type) {

  std::unordered_map<std::string, std::string> mapping;
  for (const auto &group : new_objects_by_type) {
    for (size_t i = 0; i < group.names.size(); ++i) {
      const std::string placeholder =
          "__obj_" + group.type + "_" + std::to_string(i) + "__";
      auto it = old_placeholder_to_original.find(placeholder);
      if (it != old_placeholder_to_original.end()) {
        mapping[it->second] = group.names[i];
      }
    }
  }
  return mapping;
}

omni_plan::pddl::Plan PlanAdapter::adapt(
    const CachedPlanData &cached,
    const std::unordered_map<std::string, std::string> &old_to_new) {

  omni_plan::pddl::Plan adapted;
  for (size_t i = 0; i < cached.plan.size(); ++i) {
    auto [action, params] = cached.plan.get_action_with_params(i);
    for (auto &param : params) {
      auto it = old_to_new.find(param);
      if (it != old_to_new.end()) {
        param = it->second;
      }
    }
    adapted.add_action(action, params, cached.plan.get_action_start_time(i));
  }
  adapted.set_has_solution(cached.plan.has_solution());
  adapted.set_raw_output(adapted.to_pddl());
  return adapted;
}

} // namespace detail
} // namespace omni_plan_cache
```

- [ ] **Step 4: Delegate `CachePlanner::build_name_mapping` and remove the private adapter**

In `cache_planner.cpp`, add `#include "omni_plan_cache/detail/plan_adapter.hpp"`, replace `CachePlanner::build_name_mapping`'s body with:

```cpp
std::unordered_map<std::string, std::string> CachePlanner::build_name_mapping(
    const std::unordered_map<std::string, std::string>
        &old_placeholder_to_original,
    const std::vector<ObjectsByType> &new_objects_by_type) {
  return detail::PlanAdapter::build_name_mapping(old_placeholder_to_original,
                                                 new_objects_by_type);
}
```

Delete the private `adapt_cached_plan` definition and its declaration in `cache_planner.hpp`, and change the structural-hit block in `generate_plan` (`cache_planner.cpp`, currently lines ~864-871) from:

```cpp
      pddl::Plan adapted_plan;
      if (needs_rename) {
        adapted_plan = this->adapt_cached_plan(it->second, old_to_new);
      } else {
        ...
        adapted_plan = it->second.plan;
      }
```

to:

```cpp
      pddl::Plan adapted_plan;
      if (needs_rename) {
        adapted_plan = detail::PlanAdapter::adapt(it->second, old_to_new);
      } else {
        RCLCPP_INFO(this->node_->get_logger(),
                    "CachePlanner: Structural cache hit (parsed plan reuse)");
        adapted_plan = it->second.plan;
      }
```

(The lock-scope rewrite happens in Task 10; this step only swaps the adapter.)

- [ ] **Step 5: Build and test**

Run the build, adapter test binary, then full `colcon test`.
Expected: adapter tests pass; legacy tests pass.

---

### Task 8: Implement ComponentComposer

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/component_composer.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/test/test_cache_detail_composer.cpp`
- (Wiring into `generate_plan` happens in Task 10.)

- [ ] **Step 1: Write the failing tests**

```cpp
#include <gtest/gtest.h>

#include <memory>
#include <set>
#include <string>
#include <vector>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan_cache/detail/component_composer.hpp"

using namespace omni_plan;
using namespace omni_plan_cache::detail;

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

pddl::Domain make_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_predicate(pddl::Predicate("at", {"?r", "?l"}));
  domain.add_predicate(pddl::Predicate("done", {"?l"}));
  auto move = std::make_shared<MockAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"?r", "robot"}, {"?from", "location"}, {"?to", "location"}});
  move->add_effect(pddl::Type::END, "at", {"?r", "?to"});
  move->add_effect(pddl::Type::END, "at", {"?r", "?from"}, true);
  domain.add_action(move);
  return domain;
}

} // namespace

TEST(ComponentComposerTest, ParallelIndependentDetection) {
  EXPECT_TRUE(ComponentComposer::can_solve_in_parallel(
      {std::set<std::string>{"a"}, std::set<std::string>{"b"},
       std::set<std::string>{"c"}}));
  EXPECT_FALSE(ComponentComposer::can_solve_in_parallel(
      {std::set<std::string>{"robot1", "a"},
       std::set<std::string>{"robot1", "b"}}));
}

TEST(ComponentComposerTest, ComposesDisjointComponents) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  for (const auto *name : {"a", "b", "c"}) {
    problem.add_object(pddl::Object(name, "location"));
    problem.add_goal(pddl::Predicate("done", {name}));
  }

  int solver_calls = 0;
  ComponentComposer composer(
      ComponentComposer::Options{},
      [&](const pddl::Domain &d, const pddl::Problem &p) {
        ++solver_calls;
        pddl::Plan plan;
        plan.set_has_solution(true);
        plan.add_action(d.get_actions().at("move"), {"robot1", "a", "b"}, 0.0f);
        return plan;
      });

  pddl::Plan out;
  const std::set<pddl::Predicate> relevant;
  const std::set<std::string> static_predicates;
  EXPECT_TRUE(composer.compose(domain, problem, relevant, static_predicates, out));
  EXPECT_EQ(solver_calls, 3);
  EXPECT_TRUE(out.has_solution());
  EXPECT_EQ(out.size(), 3u);
}

TEST(ComponentComposerTest, RejectsTooManyGoalsPerComponent) {
  const auto domain = make_domain();
  pddl::Problem problem;
  problem.add_object(pddl::Object("robot1", "robot"));
  problem.add_object(pddl::Object("shared", "location"));
  for (int i = 0; i < 5; ++i) {
    problem.add_goal(pddl::Predicate(
        "done", {"shared"})); // duplicates collapse in the set: use args
  }
  problem.add_object(pddl::Object("x0", "location"));
  problem.add_object(pddl::Object("x1", "location"));
  problem.add_object(pddl::Object("x2", "location"));
  problem.add_object(pddl::Object("x3", "location"));
  problem.add_object(pddl::Object("x4", "location"));
  problem.add_goal(pddl::Predicate("done", {"x0"}));
  problem.add_goal(pddl::Predicate("done", {"x1"}));
  problem.add_goal(pddl::Predicate("done", {"x2"}));
  problem.add_goal(pddl::Predicate("done", {"x3"}));
  problem.add_goal(pddl::Predicate("done", {"x4"}));

  ComponentComposer composer(ComponentComposer::Options{},
                             [](const pddl::Domain &, const pddl::Problem &) {
                               pddl::Plan plan;
                               plan.set_has_solution(true);
                               return plan;
                             });
  pddl::Plan out;
  EXPECT_FALSE(composer.compose(domain, problem, {}, {}, out));
}
```

- [ ] **Step 2: Build and verify the tests fail**

Run the build and `./build/omni_plan_cache/test_cache_detail_composer`.
Expected: FAIL (stub returns false / does nothing).

- [ ] **Step 3: Implement `component_composer.cpp`**

```cpp
#include "omni_plan_cache/detail/component_composer.hpp"

#include <algorithm>
#include <future>
#include <numeric>
#include <thread>
#include <unordered_map>
#include <utility>

namespace omni_plan_cache {
namespace detail {

namespace pddl = omni_plan::pddl;

ComponentComposer::ComponentComposer(Options options, Solver solver,
                                     Logger info_log, Logger warn_log)
    : options_(std::move(options)), solver_(std::move(solver)),
      info_(std::move(info_log)), warn_(std::move(warn_log)) {}

namespace {

std::string component_key(const std::vector<pddl::Predicate> &component) {
  std::string key;
  for (const auto &goal : component) {
    key += goal.get_name();
    for (const auto &arg : goal.get_args()) {
      key += "_" + arg;
    }
    key += ";";
  }
  return key;
}

std::string find_root(std::unordered_map<std::string, std::string> &parent,
                      const std::string &name) {
  auto it = parent.find(name);
  if (it == parent.end()) {
    parent[name] = name;
    return name;
  }
  if (it->second == name) {
    return name;
  }
  it->second = find_root(parent, it->second);
  return it->second;
}

void unite(std::unordered_map<std::string, std::string> &parent,
           const std::string &a, const std::string &b) {
  const std::string root_a = find_root(parent, a);
  const std::string root_b = find_root(parent, b);
  if (root_a != root_b) {
    parent[root_b] = root_a;
  }
}

bool fact_inside(const pddl::Predicate &fact,
                 const std::set<std::string> &objects) {
  for (const auto &arg : fact.get_args()) {
    if (!objects.count(arg)) {
      return false;
    }
  }
  return true;
}

} // namespace

bool ComponentComposer::can_solve_in_parallel(
    const std::vector<std::set<std::string>> &mutable_objects) {
  for (size_t i = 0; i < mutable_objects.size(); ++i) {
    for (size_t j = i + 1; j < mutable_objects.size(); ++j) {
      for (const auto &object : mutable_objects[i]) {
        if (mutable_objects[j].count(object)) {
          return false;
        }
      }
    }
  }
  return true;
}

void ComponentComposer::apply_action_effects(
    std::set<pddl::Predicate> &facts,
    const std::shared_ptr<pddl::Action> &action,
    const std::vector<std::string> &params) {

  const auto parameters = action->get_parameters();
  std::unordered_map<std::string, int> index;
  index.reserve(parameters.size());
  for (size_t i = 0; i < parameters.size(); ++i) {
    index[parameters[i].get_name()] = static_cast<int>(i);
  }

  const auto effects = action->get_effects();
  auto apply = [&](pddl::Type timing) {
    for (const auto &effect : effects) {
      if (effect.get_type() != timing) {
        continue;
      }
      std::vector<std::string> args;
      const auto effect_args = effect.get_args();
      args.reserve(effect_args.size());
      for (const auto &arg : effect_args) {
        auto it = index.find(arg);
        if (it != index.end() && it->second < static_cast<int>(params.size())) {
          args.push_back(params[it->second]);
        } else {
          args.push_back(arg);
        }
      }
      pddl::Predicate predicate(effect.get_name(), args);
      if (effect.is_negated()) {
        facts.erase(predicate);
      } else {
        facts.insert(std::move(predicate));
      }
    }
  };

  apply(pddl::Type::START);
  apply(pddl::Type::END);
}

bool ComponentComposer::compose(
    const pddl::Domain &domain, const pddl::Problem &problem,
    const std::set<pddl::Predicate> &relevant_facts,
    const std::set<std::string> &full_static_predicates,
    pddl::Plan &out_plan) {

  // 1. Union-find grouping: goals sharing any object are one component.
  std::unordered_map<std::string, std::string> parent;
  for (const auto &goal : problem.get_goals()) {
    const auto args = goal.get_args();
    if (args.empty()) {
      continue;
    }
    for (size_t i = 1; i < args.size(); ++i) {
      unite(parent, args[0], args[i]);
    }
  }

  std::vector<std::vector<pddl::Predicate>> components;
  std::unordered_map<std::string, size_t> root_to_index;
  for (const auto &goal : problem.get_goals()) {
    const auto args = goal.get_args();
    const std::string root =
        args.empty()
            ? "__anon_" + std::to_string(components.size())
            : find_root(parent, args[0]);
    auto it = root_to_index.find(root);
    if (it == root_to_index.end()) {
      root_to_index[root] = components.size();
      components.push_back({});
    }
    components[root_to_index[root]].push_back(goal);
  }

  if (components.size() < 2) {
    return false;
  }
  for (const auto &component : components) {
    if (component.size() > options_.max_goals_per_component) {
      return false;
    }
  }

  // 2. Deterministic ordering, optional priority predicate first.
  auto has_priority = [this](const std::vector<pddl::Predicate> &component) {
    if (options_.priority_predicate.empty()) {
      return false;
    }
    return std::any_of(component.begin(), component.end(),
                       [this](const pddl::Predicate &goal) {
                         return goal.get_name() == options_.priority_predicate;
                       });
  };

  std::vector<std::string> keys;
  keys.reserve(components.size());
  for (const auto &component : components) {
    keys.push_back(component_key(component));
  }
  std::vector<size_t> order(components.size());
  std::iota(order.begin(), order.end(), 0);
  std::stable_sort(order.begin(), order.end(), [&](size_t ia, size_t ib) {
    const auto &a = components[ia];
    const auto &b = components[ib];
    const bool a_priority = has_priority(a);
    const bool b_priority = has_priority(b);
    if (a_priority != b_priority) {
      return a_priority;
    }
    if (a.size() != b.size()) {
      return a.size() < b.size();
    }
    return keys[ia] < keys[ib];
  });
  std::vector<std::vector<pddl::Predicate>> ordered;
  ordered.reserve(components.size());
  for (const size_t index : order) {
    ordered.push_back(std::move(components[index]));
  }
  components = std::move(ordered);

  // 3. Static objects and facts; simulated state starts from relevant facts.
  std::set<std::string> static_objects;
  for (const auto &fact : problem.get_facts()) {
    if (!full_static_predicates.count(fact.get_name())) {
      continue;
    }
    for (const auto &arg : fact.get_args()) {
      static_objects.insert(arg);
    }
  }

  std::set<pddl::Predicate> state = relevant_facts;

  std::set<pddl::Predicate> static_facts;
  for (const auto &fact : problem.get_facts()) {
    if (full_static_predicates.count(fact.get_name())) {
      static_facts.insert(fact);
    }
  }

  // 4. Build one reduced sub-problem per component.
  struct PreparedComponent {
    pddl::Problem problem;
    std::set<std::string> mutable_objects;
    std::string key;
  };
  std::vector<PreparedComponent> prepared;
  prepared.reserve(components.size());

  for (const auto &component : components) {
    std::set<std::string> comp_objects = static_objects;
    for (const auto &obj : problem.get_objects()) {
      if (obj.get_type() == options_.robot_type) {
        comp_objects.insert(obj.get_name());
      }
    }
    for (const auto &goal : component) {
      for (const auto &arg : goal.get_args()) {
        comp_objects.insert(arg);
      }
    }

    std::unordered_map<std::string, std::vector<const pddl::Predicate *>>
        object_facts;
    for (const auto &fact : state) {
      for (const auto &arg : fact.get_args()) {
        object_facts[arg].push_back(&fact);
      }
    }
    std::vector<std::string> queue(comp_objects.begin(), comp_objects.end());
    for (size_t qi = 0; qi < queue.size(); ++qi) {
      auto it = object_facts.find(queue[qi]);
      if (it == object_facts.end()) {
        continue;
      }
      for (const auto *fact : it->second) {
        for (const auto &arg : fact->get_args()) {
          if (comp_objects.insert(arg).second) {
            queue.push_back(arg);
          }
        }
      }
    }

    PreparedComponent pc;
    pc.key = component_key(component);
    for (const auto &obj : problem.get_objects()) {
      if (comp_objects.count(obj.get_name())) {
        pc.problem.add_object(obj);
      }
    }
    for (const auto &fact : state) {
      if (fact_inside(fact, comp_objects)) {
        pc.problem.add_fact(fact);
      }
    }
    for (const auto &fact : static_facts) {
      if (fact_inside(fact, comp_objects)) {
        pc.problem.add_fact(fact);
      }
    }
    for (const auto &goal : component) {
      pc.problem.add_goal(goal);
    }
    for (const auto &object : comp_objects) {
      if (!static_objects.count(object)) {
        pc.mutable_objects.insert(object);
      }
    }
    prepared.push_back(std::move(pc));
  }

  // 5. Solve. Parallel only when mutable objects are pairwise disjoint.
  std::vector<std::set<std::string>> mutable_sets;
  mutable_sets.reserve(prepared.size());
  for (const auto &pc : prepared) {
    mutable_sets.push_back(pc.mutable_objects);
  }
  const bool parallel =
      options_.parallel_independent && prepared.size() > 1 &&
      can_solve_in_parallel(mutable_sets);

  pddl::Plan composed;
  float t = 0.0f;

  auto append_plan = [&](const pddl::Plan &sub_plan) {
    for (size_t i = 0; i < sub_plan.size(); ++i) {
      auto [action, params] = sub_plan.get_action_with_params(i);
      composed.add_action(action, params, t);
      t += action->get_duration();
    }
  };

  if (parallel) {
    const size_t batch = std::max<size_t>(
        1, static_cast<size_t>(std::thread::hardware_concurrency()));
    std::vector<pddl::Plan> plans(prepared.size());
    for (size_t start = 0; start < prepared.size(); start += batch) {
      const size_t end = std::min(prepared.size(), start + batch);
      std::vector<std::future<pddl::Plan>> futures;
      futures.reserve(end - start);
      for (size_t i = start; i < end; ++i) {
        futures.push_back(std::async(std::launch::async,
                                     [this, &domain, &prepared, i]() {
                                       return this->solver_(domain,
                                                            prepared[i].problem);
                                     }));
      }
      for (size_t i = start; i < end; ++i) {
        plans[i] = futures[i - start].get();
        if (!plans[i].has_solution()) {
          if (warn_) {
            warn_("CachePlanner: component " + prepared[i].key +
                  " has no solution, falling back to full problem");
          }
          return false;
        }
      }
    }
    for (const auto &plan : plans) {
      append_plan(plan);
    }
  } else {
    for (const auto &pc : prepared) {
      pddl::Plan sub_plan = this->solver_(domain, pc.problem);
      if (!sub_plan.has_solution()) {
        if (warn_) {
          warn_("CachePlanner: component " + pc.key +
                " has no solution, falling back to full problem");
        }
        return false;
      }
      append_plan(sub_plan);
      for (size_t i = 0; i < sub_plan.size(); ++i) {
        auto [action, params] = sub_plan.get_action_with_params(i);
        apply_action_effects(state, action, params);
      }
    }
  }

  composed.set_has_solution(true);
  if (info_) {
    info_("CachePlanner: composed plan from " +
          std::to_string(components.size()) + " components (" +
          std::to_string(composed.size()) + " actions)");
  }
  out_plan = std::move(composed);
  return true;
}

} // namespace detail
} // namespace omni_plan_cache
```

- [ ] **Step 4: Build and run tests**

Run the build and `./build/omni_plan_cache/test_cache_detail_composer`, then the full `colcon test`.
Expected: composer tests pass; legacy tests pass (composer is not yet wired into `generate_plan`).

---

### Task 9: Implement PlanCache

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/detail/plan_cache.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/test/test_cache_detail_plan_cache.cpp`
- (Wiring into `CachePlanner` happens in Task 10.)

- [ ] **Step 1: Write the failing tests**

```cpp
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
  {
    const auto leader = cache.begin_or_join("key");
    EXPECT_TRUE(leader.leader);
    const auto joined = cache.begin_or_join("key");
    EXPECT_FALSE(joined.leader);
    follower = joined.future;
  } // leader guard destructor abandons the entry
  ASSERT_TRUE(follower.valid());
  EXPECT_EQ(follower.get(), nullptr);
}
```

- [ ] **Step 2: Build and verify the tests fail**

Run the build and `./build/omni_plan_cache/test_cache_detail_plan_cache`.
Expected: FAIL (stub).

- [ ] **Step 3: Implement `plan_cache.cpp`**

```cpp
#include "omni_plan_cache/detail/plan_cache.hpp"

#include <limits>

namespace omni_plan_cache {
namespace detail {

void PlanCache::configure(size_t max_exact_entries,
                          size_t max_structural_entries) {
  std::unique_lock<std::shared_mutex> lock(this->mutex_);
  this->exact_.max_entries = max_exact_entries;
  this->structural_.max_entries = max_structural_entries;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_locked(CacheMap &map, const std::string &key) {
  auto it = map.entries.find(key);
  if (it == map.entries.end()) {
    return nullptr;
  }
  it->second.last_use.store(
      this->tick_.fetch_add(1, std::memory_order_relaxed),
      std::memory_order_relaxed);
  return it->second.data;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_exact(const std::string &key) {
  std::shared_lock<std::shared_mutex> lock(this->mutex_);
  auto data = this->get_locked(this->exact_, key);
  if (data) {
    this->exact_hits_.fetch_add(1, std::memory_order_relaxed);
  } else {
    this->exact_misses_.fetch_add(1, std::memory_order_relaxed);
  }
  return data;
}

std::shared_ptr<const CachedPlanData>
PlanCache::get_structural(const std::string &key) {
  std::shared_lock<std::shared_mutex> lock(this->mutex_);
  auto data = this->get_locked(this->structural_, key);
  if (data) {
    this->structural_hits_.fetch_add(1, std::memory_order_relaxed);
  }
  return data;
}

void PlanCache::evict_if_needed(CacheMap &map) {
  if (map.max_entries == 0 || map.entries.size() <= map.max_entries) {
    return;
  }
  while (map.entries.size() > map.max_entries) {
    auto victim = map.entries.end();
    uint64_t oldest = std::numeric_limits<uint64_t>::max();
    for (auto it = map.entries.begin(); it != map.entries.end(); ++it) {
      const uint64_t last_use =
          it->second.last_use.load(std::memory_order_relaxed);
      if (last_use < oldest) {
        oldest = last_use;
        victim = it;
      }
    }
    if (victim == map.entries.end()) {
      break;
    }
    map.entries.erase(victim);
    map.evictions.fetch_add(1, std::memory_order_relaxed);
  }
}

void PlanCache::put_locked(CacheMap &map, const std::string &key,
                           const std::shared_ptr<const CachedPlanData> &data,
                           std::atomic<uint64_t> &tick) {
  const uint64_t stamp = tick.fetch_add(1, std::memory_order_relaxed);
  auto it = map.entries.find(key);
  if (it != map.entries.end()) {
    it->second.data = data;
    it->second.last_use.store(stamp, std::memory_order_relaxed);
    return;
  }
  map.entries.emplace(std::piecewise_construct, std::forward_as_tuple(key),
                      std::forward_as_tuple(data, stamp));
  evict_if_needed(map);
}

void PlanCache::put(const std::string &exact_key,
                    const std::string &structural_key,
                    std::shared_ptr<const CachedPlanData> data) {
  std::unique_lock<std::shared_mutex> lock(this->mutex_);
  put_locked(this->exact_, exact_key, data, this->tick_);
  put_locked(this->structural_, structural_key, data, this->tick_);
}

void PlanCache::put_exact(const std::string &exact_key,
                          std::shared_ptr<const CachedPlanData> data) {
  std::unique_lock<std::shared_mutex> lock(this->mutex_);
  put_locked(this->exact_, exact_key, data, this->tick_);
}

PlanCache::Flight PlanCache::begin_or_join(const std::string &key) {
  std::lock_guard<std::mutex> lock(this->inflight_mutex_);
  auto it = this->inflight_.find(key);
  if (it != this->inflight_.end()) {
    if (it->second->owner == std::this_thread::get_id()) {
      return Flight{true, {}, {}};
    }
    return Flight{false, it->second->future, {}};
  }
  auto entry = std::make_shared<InFlight>();
  entry->owner = std::this_thread::get_id();
  entry->future = entry->promise.get_future().share();
  this->inflight_[key] = entry;

  Flight flight;
  flight.leader = true;
  flight.guard = std::shared_ptr<void>(
      nullptr, [this, key](void *) { this->abandon(key); });
  return flight;
}

void PlanCache::abandon(const std::string &key) {
  std::shared_ptr<InFlight> entry;
  {
    std::lock_guard<std::mutex> lock(this->inflight_mutex_);
    auto it = this->inflight_.find(key);
    if (it == this->inflight_.end()) {
      return;
    }
    entry = it->second;
    this->inflight_.erase(it);
  }
  try {
    entry->promise.set_value(nullptr);
  } catch (const std::future_error &) {
  }
}

void PlanCache::publish(const std::string &key,
                        std::shared_ptr<const CachedPlanData> data) {
  std::shared_ptr<InFlight> entry;
  {
    std::lock_guard<std::mutex> lock(this->inflight_mutex_);
    auto it = this->inflight_.find(key);
    if (it == this->inflight_.end()) {
      return;
    }
    entry = it->second;
    this->inflight_.erase(it);
  }
  try {
    entry->promise.set_value(std::move(data));
  } catch (const std::future_error &) {
  }
}

void PlanCache::publish_failure(const std::string &key,
                                std::exception_ptr error) {
  std::shared_ptr<InFlight> entry;
  {
    std::lock_guard<std::mutex> lock(this->inflight_mutex_);
    auto it = this->inflight_.find(key);
    if (it == this->inflight_.end()) {
      return;
    }
    entry = it->second;
    this->inflight_.erase(it);
  }
  try {
    entry->promise.set_exception(error);
  } catch (const std::future_error &) {
  }
}

CacheStats PlanCache::stats() const {
  std::shared_lock<std::shared_mutex> lock(this->mutex_);
  CacheStats stats;
  stats.exact_hits = this->exact_hits_.load(std::memory_order_relaxed);
  stats.exact_misses = this->exact_misses_.load(std::memory_order_relaxed);
  stats.structural_hits =
      this->structural_hits_.load(std::memory_order_relaxed);
  stats.full_misses = this->full_misses_.load(std::memory_order_relaxed);
  stats.validations = this->validations_.load(std::memory_order_relaxed);
  stats.adaptations = this->adaptations_.load(std::memory_order_relaxed);
  stats.compositions = this->compositions_.load(std::memory_order_relaxed);
  stats.composition_fallbacks =
      this->composition_fallbacks_.load(std::memory_order_relaxed);
  stats.exact_evictions =
      this->exact_.evictions.load(std::memory_order_relaxed);
  stats.structural_evictions =
      this->structural_.evictions.load(std::memory_order_relaxed);
  stats.exact_entries = this->exact_.entries.size();
  stats.structural_entries = this->structural_.entries.size();
  return stats;
}

} // namespace detail
} // namespace omni_plan_cache
```

- [ ] **Step 4: Build and run tests**

Run the build and `./build/omni_plan_cache/test_cache_detail_plan_cache`, then the full `colcon test`.
Expected: PlanCache tests pass; legacy tests pass.

---

### Task 10: Rewrite `CachePlanner` orchestration

**Files:**
- Modify: `omni_plan_planners/omni_plan_cache/include/omni_plan_cache/cache_planner.hpp`
- Modify: `omni_plan_planners/omni_plan_cache/src/omni_plan_cache/cache_planner.cpp`

- [ ] **Step 1: Update the header**

In `cache_planner.hpp`:

1. Add includes:
```cpp
#include "omni_plan_cache/detail/plan_cache.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"
```
2. Add a public method after `build_name_mapping`:
```cpp
  /**
   * @brief Returns a snapshot of cache performance counters.
   */
  omni_plan_cache::CacheStats get_cache_stats() const;
```
3. Add private members (and remove the old private cache members and method declarations):
```cpp
  /// @brief Robot object type used by component composition and relevance.
  std::string robot_type_;
  /// @brief Goal predicate whose component is ordered first ("" = none).
  std::string component_priority_predicate_;
  /// @brief Maximum goals per component eligible for composition.
  int component_goal_limit_ = 4;
  /// @brief Maximum exact cache entries (0 = unbounded).
  int max_exact_cache_entries_ = 0;
  /// @brief Maximum structural cache entries (0 = unbounded).
  int max_structural_cache_entries_ = 0;

  /// @brief Cache store with bounds, metrics and single-flight.
  mutable detail::PlanCache plan_cache_;
```
Delete these private declarations: `adapt_cached_plan`, `compose_from_components`, `apply_plan_action_effects`, `exact_cache_`, `structural_cache_`, `cache_mutex_`. Keep all protected members (`wrapped_planner_`, `node_`, `delegate_plan`, `should_cache_result`, `planner_loader_`, `validator_`, `validator_loader_`, `validate_on_hit_`, `abstract_role_keys_`) and the static helper declarations.
4. Add private helper declarations:
```cpp
  rclcpp::Logger log() const;

  std::optional<omni_plan::pddl::Plan> try_structural_hit(
      const omni_plan::pddl::Domain &domain,
      const omni_plan::pddl::Problem &problem,
      const std::string &structural_key,
      const detail::PreparedStructure &prepared, bool abstract_keys) const;

  std::optional<omni_plan::pddl::Plan> serve_cached_entry(
      const CachedPlanData &entry, const omni_plan::pddl::Domain &domain,
      const omni_plan::pddl::Problem &problem,
      const detail::PreparedStructure &prepared, bool abstract_keys) const;

  omni_plan::pddl::Plan compute_miss_plan(
      const omni_plan::pddl::Domain &domain,
      const omni_plan::pddl::Problem &problem, const std::string &exact_key,
      const std::string &structural_key,
      const detail::RelevanceResult &relevance,
      const detail::PreparedStructure &prepared) const;
```
Add `#include <optional>` and `#include "omni_plan_cache/detail/relevance_analyzer.hpp"` at the top.

5. Update the `should_cache_result` Doxygen comment to say it returns `plan.has_solution()`.

- [ ] **Step 2: Add parameters and delegation in the constructor**

In `cache_planner.cpp`'s `CachePlanner::CachePlanner`, extend `add_ros_parameters` to:

```cpp
  this->add_ros_parameters({
      {"planner_plugin", std::string(), this->wrapped_planner_name_},
      {"validator_plugin", std::string(), this->validator_plugin_name_},
      {"validate_on_hit", true, this->validate_on_hit_},
      {"robot_type", std::string("robot"), this->robot_type_},
      {"component_goal_limit", 4, this->component_goal_limit_},
      {"component_priority_predicate", std::string(),
       this->component_priority_predicate_},
      {"max_exact_cache_entries", 0, this->max_exact_cache_entries_},
      {"max_structural_cache_entries", 0, this->max_structural_cache_entries_},
  });
```

At the end of the loaded-params callback (after the validator loading block), add:

```cpp
    this->plan_cache_.configure(
        static_cast<size_t>(std::max(0, this->max_exact_cache_entries_)),
        static_cast<size_t>(std::max(0, this->max_structural_cache_entries_)));
```

- [ ] **Step 3: Replace `generate_plan` and add helpers**

Replace the entire `CachePlanner::generate_plan` body with:

```cpp
omni_plan::pddl::Plan CachePlanner::generate_plan(
    const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem) const {

  const bool abstract_keys =
      this->validate_on_hit_ && (this->validator_ != nullptr);

  const std::string exact_key =
      detail::StructuralKeyer::exact_key(domain, problem);

  if (auto cached = this->plan_cache_.get_exact(exact_key)) {
    RCLCPP_INFO(this->log(), "CachePlanner: Exact cache hit");
    return cached->plan;
  }

  const auto relevance =
      detail::RelevanceAnalyzer::analyze(domain, problem, this->robot_type_);
  const auto prepared = detail::StructuralKeyer::prepare(
      problem.get_objects(), relevance.relevant_facts, problem.get_goals(),
      abstract_keys);
  const std::string structural_key = detail::StructuralKeyer::compute_key(
      domain, problem, prepared.objects_by_type, prepared.role_keys,
      &relevance.relevant_facts);

  this->plan_cache_.record_full_miss();

  if (auto plan = this->try_structural_hit(domain, problem, structural_key,
                                           prepared, abstract_keys)) {
    return *plan;
  }

  auto flight = this->plan_cache_.begin_or_join(structural_key);

  if (flight.leader) {
    try {
      return this->compute_miss_plan(domain, problem, exact_key,
                                     structural_key, relevance, prepared);
    } catch (...) {
      this->plan_cache_.publish_failure(structural_key,
                                        std::current_exception());
      throw;
    }
  }

  std::shared_ptr<const CachedPlanData> shared;
  try {
    shared = flight.future.get();
  } catch (const std::exception &e) {
    RCLCPP_WARN(this->log(), "CachePlanner: in-flight planning failed (%s)",
                e.what());
    shared = nullptr;
  }
  if (shared) {
    if (auto plan = this->serve_cached_entry(*shared, domain, problem, prepared,
                                             abstract_keys)) {
      this->plan_cache_.record_structural_hit();
      this->plan_cache_.put_exact(exact_key, shared);
      return *plan;
    }
  }

  return this->compute_miss_plan(domain, problem, exact_key, structural_key,
                                 relevance, prepared);
}
```

Add the helpers:

```cpp
rclcpp::Logger CachePlanner::log() const {
  return this->node_ ? this->node_->get_logger()
                     : rclcpp::get_logger("CachePlanner");
}

std::optional<omni_plan::pddl::Plan> CachePlanner::try_structural_hit(
    const omni_plan::pddl::Domain &domain, const omni_plan::pddl::Problem &problem,
    const std::string &structural_key, const detail::PreparedStructure &prepared,
    bool abstract_keys) const {
  auto entry = this->plan_cache_.get_structural(structural_key);
  if (!entry) {
    return std::nullopt;
  }
  return this->serve_cached_entry(*entry, domain, problem, prepared,
                                  abstract_keys);
}

std::optional<omni_plan::pddl::Plan> CachePlanner::serve_cached_entry(
    const CachedPlanData &entry, const omni_plan::pddl::Domain &domain,
    const omni_plan::pddl::Problem &problem,
    const detail::PreparedStructure &prepared, bool abstract_keys) const {

  const auto old_to_new = detail::PlanAdapter::build_name_mapping(
      entry.placeholder_to_original, prepared.objects_by_type);
  bool needs_rename = false;
  for (const auto &[old_name, new_name] : old_to_new) {
    if (old_name != new_name) {
      needs_rename = true;
      break;
    }
  }

  omni_plan::pddl::Plan plan =
      needs_rename ? detail::PlanAdapter::adapt(entry, old_to_new) : entry.plan;
  if (needs_rename) {
    this->plan_cache_.record_adaptation();
  }

  const bool must_validate = this->validator_ && this->validate_on_hit_ &&
                             (needs_rename || abstract_keys);
  if (must_validate) {
    this->plan_cache_.record_validation();
    if (!this->validator_->validate_plan(domain, problem, plan)) {
      RCLCPP_WARN(this->log(),
                  "CachePlanner: structural cache hit but adapted plan "
                  "invalid; falling through to cache miss");
      return std::nullopt;
    }
  }

  RCLCPP_INFO(this->log(), "CachePlanner: Structural cache hit%s",
              must_validate ? " (validated)" : "");
  return plan;
}

omni_plan::pddl::Plan CachePlanner::compute_miss_plan(
    const omni_plan::pddl::Domain &domain, const omni_plan::pddl::Problem &problem,
    const std::string &exact_key, const std::string &structural_key,
    const detail::RelevanceResult &relevance,
    const detail::PreparedStructure &prepared) const {

  if (this->validator_) {
    detail::ComponentComposer composer(
        detail::ComponentComposer::Options{
            this->robot_type_, this->component_priority_predicate_,
            static_cast<size_t>(std::max(0, this->component_goal_limit_)), true},
        [this](const omni_plan::pddl::Domain &d,
               const omni_plan::pddl::Problem &p) {
          return this->generate_plan(d, p);
        },
        [this](const std::string &message) {
          RCLCPP_INFO(this->log(), "%s", message.c_str());
        },
        [this](const std::string &message) {
          RCLCPP_WARN(this->log(), "%s", message.c_str());
        });

    omni_plan::pddl::Plan composed;
    if (composer.compose(domain, problem, relevance.relevant_facts,
                         relevance.full_static_predicates, composed)) {
      if (this->validator_->validate_plan(domain, problem, composed)) {
        auto data = std::make_shared<CachedPlanData>(
            CachedPlanData{composed, prepared.placeholder_to_original});
        this->plan_cache_.record_composition();
        this->plan_cache_.put(exact_key, structural_key, data);
        this->plan_cache_.publish(structural_key, data);
        return composed;
      }
      this->plan_cache_.record_composition_fallback();
      RCLCPP_WARN(this->log(),
                  "CachePlanner: composed plan invalid, falling back to full "
                  "problem");
    }
  }

  auto plan = this->delegate_plan(domain, problem, structural_key);
  RCLCPP_INFO(this->log(), "CachePlanner: Cache miss, delegating to sub-planner");

  if (this->should_cache_result(plan)) {
    auto data = std::make_shared<CachedPlanData>(
        CachedPlanData{plan, prepared.placeholder_to_original});
    this->plan_cache_.put(exact_key, structural_key, data);
    this->plan_cache_.publish(structural_key, data);
  } else {
    this->plan_cache_.publish(structural_key, nullptr);
  }
  return plan;
}

CacheStats CachePlanner::get_cache_stats() const {
  return this->plan_cache_.stats();
}
```

Delete the old private methods `adapt_cached_plan`, `compose_from_components`, `apply_plan_action_effects` and the old cache membership code from `generate_plan`. Keep the include block; add:

```cpp
#include <optional>
#include <utility>

#include "omni_plan_cache/detail/component_composer.hpp"
#include "omni_plan_cache/detail/plan_adapter.hpp"
#include "omni_plan_cache/detail/relevance_analyzer.hpp"
#include "omni_plan_cache/detail/structural_keyer.hpp"
```

Also delete the now-unused static `MAX_COMPONENT_GOALS` constant and the `"battery_ok"`/`"robot"` hardcodes (they moved to parameters).

- [ ] **Step 4: Build and fix compile errors**

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
```
Expected: builds cleanly. If `std::optional<pddl::Plan>` fails to compile because `pddl::Plan` is fine as a value type, no action needed.

- [ ] **Step 5: Run all legacy tests**

Run:
```bash
colcon test --packages-select omni_plan_cache --event-handlers console_direct+
colcon test-result --verbose
```
Expected: all legacy 23 tests pass. Pay attention to:
- `ValidateOnHitTrueValidates` (exactly 1 validation on a renamed hit),
- `ValidateOnHitFalseSkipsValidator` (0 validations),
- `NoWrappedPlannerThrows` (throws),
- all relevance/structural tests.

- [ ] **Step 6: Build and run `HomeostaticPlanner`**

Run:
```bash
colcon build --packages-select omni_plan_homeostatic --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select omni_plan_homeostatic --event-handlers console_direct+
colcon test-result --verbose
```
Expected: builds with no source changes; its tests pass.

---

### Task 11: New integration tests

**Files:**
- Create: `omni_plan_planners/omni_plan_cache/test/test_cache_planner_extras.cpp`
- Modify: `omni_plan_planners/omni_plan_cache/CMakeLists.txt`

- [ ] **Step 1: Create the test helper + failing tests**

```cpp
#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
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

class MockValidator : public PlanValidator {
public:
  mutable std::atomic<int> calls{0};
  bool validate_plan(const pddl::Domain &, const pddl::Problem &,
                     const pddl::Plan &) const override {
    this->calls.fetch_add(1);
    return true;
  }

protected:
  bool validate_plan(const std::string &, const std::string &,
                     const std::string &) const override {
    return true;
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
}

TEST(CachePlannerExtrasTest, BoundedCachesEvict) {
  auto options = rclcpp::NodeOptions().parameter_overrides(
      {rclcpp::Parameter("max_exact_cache_entries", 1),
       rclcpp::Parameter("max_structural_cache_entries", 1)});
  auto node = std::make_shared<rclcpp::Node>("extras_bounded", options);
  TestableCachePlanner planner;
  planner.load_ros_parameters(node);
  planner.inject(std::make_shared<SlowMockPlanner>(),
                 std::make_shared<MockValidator>());

  const auto domain = make_domain();
  planner.generate_plan(domain, make_problem("robot1", "loc1", "loc2"));
  planner.generate_plan(domain, make_problem("robot2", "loc3", "loc4"));

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

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  auto result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
```

- [ ] **Step 2: Add the test target**

Inside `if(BUILD_TESTING)` in `CMakeLists.txt`, after the detail test loop:

```cmake
  ament_add_gtest(test_cache_planner_extras test/test_cache_planner_extras.cpp)
  target_include_directories(test_cache_planner_extras PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(test_cache_planner_extras
    cache_planner
    ${DEPENDENCIES}
  )
```

- [ ] **Step 3: Build and run**

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select omni_plan_cache --event-handlers console_direct+
colcon test-result --verbose
```
Expected: `test_cache_planner_extras` 4/4 pass, everything else still passes. If `SingleFlightDeduplicatesConcurrentMisses` is flaky because the planner finishes before the other threads join, increase `delay_ms` to 100 and retry — do not weaken the assertion.

---

### Task 12: Re-run the benchmark and compare

**Files:** none modified (measurement only).

- [ ] **Step 1: Rebuild and run the benchmark**

Run:
```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select omni_plan_cache --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
./build/omni_plan_cache/benchmark_cache_planner | tee /tmp/opencode/cache_bench_after.txt
```

- [ ] **Step 2: Compare against the baseline**

Run:
```bash
paste /tmp/opencode/cache_bench_baseline.txt /tmp/opencode/cache_bench_after.txt
```
Expected: per-call `exact hit` and `structural hit` times are lower after the refactor; `miss` should be no worse. If any scenario regresses by more than ~10%, investigate with `perf` before declaring success.

- [ ] **Step 3: Full test sweep**

Run:
```bash
colcon test --packages-select omni_plan_cache omni_plan_homeostatic --event-handlers console_direct+
colcon test-result --verbose
```
Expected: all green.

- [ ] **Step 4: Ensure no commits were made**

Run:
```bash
git -C /home/miguel/ros2_ws/src/omni_plan status --short
git -C /home/miguel/ros2_ws/src/omni_plan diff --stat
```
Expected: only intended working-tree changes; no new commits (`git log -1` still shows `0820635`). Do **not** commit.

---

## Self-review notes

- **Spec coverage:** bug fixes map to Tasks 7 (raw_output), 10 (lock scope, logs, stale flag, doc), 8/10 (hardcoded constants → params), 9 (double storage, bounds, metrics), 10 (single-flight). CPU optimizations map to Task 6 (streamed hashing, relevance index, single-pass roles), 8 (composer), 9 (lock scope). Parallelism maps to Task 8 (disjoint components) and 9 (single-flight). Benchmark maps to Tasks 1 and 12. Tests map to Tasks 4–9 and 11. API stability is enforced by Tasks 2, 3, and the unchanged legacy test run in every task.
- **`exact_key` change:** exact cache keys now hash canonical fields rather than `to_pddl()` text. This is internal-only (caches are process-local) and every legacy exact-hit test still passes.
- **No commit steps** anywhere, per the user instruction; each task ends with build/test verification instead.
