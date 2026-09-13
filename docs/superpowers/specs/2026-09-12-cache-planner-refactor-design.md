# CachePlanner Refactor and Optimization — Design

Date: 2026-09-12
Package: `omni_plan_planners/omni_plan_cache`
Status: Approved design (implementation plan pending)

## 1. Goals and non-goals

### Goals

- Fix the correctness and concurrency bugs found in `CachePlanner`.
- Reduce CPU time per `generate_plan` call on exact hits, structural hits, and misses.
- Reduce algorithmic complexity of the structural-key pipeline and component composer.
- Simplify the code by decomposing the 949-line translation unit into small, focused,
  testable units behind a stable `CachePlanner` facade.
- Add safe, targeted parallelism: single-flight deduplication and parallel solving of
  provably independent goal components.
- Add optional bounded caches and lightweight metrics, off by default so behavior and
  memory usage are unchanged unless configured.
- Add a standalone benchmark target to produce before/after CPU numbers.
- Keep the existing 23 tests passing unchanged and keep `HomeostaticPlanner` compiling
  without edits.

### Non-goals

- No changes to planning semantics or plan contents beyond the bug fixes listed below.
- No changes to `omni_plan` base-class PDDL types or other planner packages.
- No commits (explicit user instruction).
- No new third-party dependencies (C++17 standard library + already linked OpenSSL only).

## 2. Current state

- `cache_planner.cpp`: 949 lines; `cache_planner.hpp`: 313 lines; tests: 945 lines.
- Caches: unbounded `exact_cache_` (`unordered_map<string, Plan>`) and
  `structural_cache_` (`unordered_map<string, CachedPlan>`) guarded by one
  `std::shared_mutex`.
- Consumers: `HomeostaticPlanner` (subclass, production) uses protected `node_`,
  `planner_loader_`, overrides `delegate_plan`/`should_cache_result`;
  `TestableCachePlanner` (test) injects `wrapped_planner_`, `validator_`,
  `validate_on_hit_`; tests call the public static helpers.
- Baseline after a clean rebuild: 23/23 tests pass. (An initial segfault was caused by a
  stale install tree with mismatched ABI, not by current code.)

## 3. Bugs to fix

1. **Validation under the cache lock** (`cache_planner.cpp:847-893`). `build_name_mapping`,
   `adapt_cached_plan`, and `validator_->validate_plan()` run inside a `shared_lock`,
   blocking all writers for the duration of validation.
   Fix: copy the cached entry (as `shared_ptr<const CachedPlanData>`) under the lock,
   release it, then adapt and validate.
2. **`raw_output` lost on adapted plans** (`cache_planner.cpp:368-388`). Structural hits
   with renames (and composed plans) return a plan whose `get_raw_output()` is empty;
   `plan_state.cpp:63-66` logs it. Fix: set `raw_output` on adapted/composed plans.
3. **Duplicate and premature log** (`cache_planner.cpp:868-879`). Log once, only after the
   hit is accepted.
4. **Hardcoded domain constants**: `"battery_ok"` (`:504`), type `"robot"` (`:555`),
   `MAX_COMPONENT_GOALS = 4` (`:434`). Fix: new parameters
   `component_priority_predicate` (default `""`, since `battery_ok` appears nowhere else
   in the repo), `robot_type` (default `"robot"`), `component_goal_limit` (default 4).
5. **Documentation mismatch** for `should_cache_result` (`hpp:232-241` vs
   `cpp:944-947`). Fix the comments.
6. **No single-flight**: concurrent identical misses each call the sub-planner. Fix with
   an in-flight registry (see section 7).
7. **Unbounded caches and double storage**: the same `Plan` is copied into both maps.
   Fix: store `std::shared_ptr<const CachedPlanData>` in both maps, and add optional
   per-map bounds (default 0 = unlimited).
8. **`abstract_role_keys_` goes stale** if `validate_on_hit` changes after parameter load
   (`cache_planner.cpp:114-115`). Fix: derive it at use time; keep the protected member
   for API stability but stop relying on a cached copy.

`validate_on_hit=false` continues to allow unvalidated renamed hits. This is documented
as best-effort in the header.

## 4. Architecture

New internal units under `include/omni_plan_cache/detail/` and
`src/omni_plan_cache/detail/`:

| Unit | Responsibility |
| --- | --- |
| `StructuralKeyer` | Object grouping; single-pass role keys; structural key digest; placeholder map |
| `RelevanceAnalyzer` | Per-call predicate→actions index; backward-chaining relevance; relevant/static fact and object sets |
| `PlanAdapter` | Placeholder-to-object name mapping; plan adaptation; `raw_output` preservation |
| `ComponentComposer` | Goal decomposition (union-find); ordering; closure; sub-problem build; effects simulation; sequential and disjoint-parallel solving via a solver callback |
| `PlanCache` | Exact + structural maps, `shared_ptr<const CachedPlanData>` storage, lock, optional approximate-LRU bounds, metrics, single-flight registry |
| `Sha256` (small util) | RAII EVP digest wrapper and hex formatting |

`CachePlanner` keeps its exact public/protected API and implements orchestration only.
The public static helpers (`sha256`, `group_objects_by_type`, `compute_role_keys`,
`compute_structural_key`, `compute_relevant_predicates`, `build_name_mapping`) keep
their signatures and delegate to the detail units. `CachedPlan` and `ObjectsByType`
remain in the public header.

Expected file sizes: orchestrator ~350-450 lines, keying ~250, relevance ~120,
adapter ~80, composer ~250, cache ~200.

### Data flow

```
generate_plan(domain, problem)
  ├─ exact key digest ──► PlanCache::get_exact ──► hit: return
  ├─ RelevanceAnalyzer
  ├─ StructuralKeyer (roles → key → placeholders)
  ├─ PlanCache::get_structural ──► hit: copy entry, unlock, adapt, validate, return
  ├─ ComponentComposer (optional, requires validator; disjoint components may solve in parallel)
  ├─ single-flight on structural key
  │    ├─ leader: delegate_plan (or composition) → PlanCache::put_exact + put_structural
  │    └─ follower: wait, then retry the structural-hit path
  └─ return plan
```

## 5. Structural-key pipeline optimizations

- **Incremental hashing**: compute exact and structural keys by streaming canonical fields
  (requirements, types, predicates, actions; objects, facts, goals; abstraction parts)
  into an OpenSSL EVP SHA-256 context. No `domain.to_pddl()`/`problem.to_pddl()` string
  construction and no concatenation of the abstraction string for keying.
  The `sha256(const std::string&)` helper stays and uses a hex lookup table instead of
  `std::stringstream`.
- **Single-pass role keys**: collect each object's contributions once while computing the
  abstract keys; after sorting and alias assignment, expand to concrete keys from the
  collected data instead of re-traversing all facts and goals.
- **Relevance analysis**: build a per-call index (`predicate name → actions whose effects
  mention it`, plus each action's condition names) once, then BFS/backward-chain over the
  index. Complexity drops from O(relevant × actions × effects) to
  O(actions × effects + reachable fan-out).
- **Composer**: union-find goal grouping; precomputed component key strings (not rebuilt
  inside the sort comparator); worklist-based object closure; a single copy of
  `action->get_effects()` with two filtered passes (start then end) instead of two
  `get_on_start_effects()`/`get_on_end_effects()` copies; precomputed
  parameter-name→index map per action instead of linear `get_parameter_index` scans.
- Reserve strings and avoid repeated `Predicate::get_args()` copies in hot loops.

## 6. Component composition

Behavior preserved, with these changes:

- `component_priority_predicate` replaces the hardcoded `battery_ok` ordering (empty
  default = no priority component).
- `robot_type` replaces the hardcoded `"robot"` seed.
- `component_goal_limit` replaces `MAX_COMPONENT_GOALS`.
- The composer receives a solver callback so it can be tested without `CachePlanner` and
  so composition results still flow through the same caching path.

Parallel composition rule (safe subset):

- Compute each component's sub-problem object set as today.
- Two components are independent iff their sets of **mutable** objects are disjoint.
  Static-fact objects (rooms, chargers, ...) may be shared.
- If all components are pairwise independent, solve them concurrently with a bounded
  number of `std::async` tasks (≤ `std::thread::hardware_concurrency()`), then stitch
  them in deterministic order, simulate effects, and validate.
- Otherwise fall back to the current sequential solve/simulate loop.
- If the validator is absent, composition is disabled as today.

## 7. Concurrency design

- **Single-flight**: `PlanCache` maintains an in-flight registry keyed by structural key.
  The first thread becomes the owner and computes; followers wait on a `shared_future`.
  Owner completion stores the entry in both caches and publishes the result; owner failure
  publishes an exception or empty result (followers then compute for themselves).
  An RAII guard removes the entry on every exit path. If a thread encounters a key it
  already owns (recursive component solve edge case), it bypasses waiting to avoid
  self-deadlock.
- **Lock discipline**: all cache reads copy `shared_ptr`s under a `shared_lock`; all
  mutation under a `unique_lock`; adaptation and validation happen with no lock held.
- **Metrics** are atomics; sizes are read under a shared lock.
- **Parallel components** as in section 6.

## 8. Cache and metrics design

- `struct CachedPlanData { Plan plan; std::unordered_map<std::string, std::string>
  placeholder_to_original; }` stored as `std::shared_ptr<const CachedPlanData>` in both
  maps, so a plan is stored once and shared.
- Parameters: `max_exact_cache_entries` and `max_structural_cache_entries` (int,
  default 0 = unlimited).
- Bounded eviction: approximate LRU. Hits update a per-entry atomic last-use timestamp
  (relaxed); when inserting over the limit, evict the oldest of a small fixed-size random
  sample. This keeps reads on the shared lock and eviction O(1)-ish.
- `CacheStats` counters: `exact_hits`, `structural_hits`, `misses`, `validations`,
  `adaptations`, `compositions`, `composition_fallbacks`, `exact_evictions`,
  `structural_evictions`, plus current sizes. Exposed via a new public
  `get_cache_stats()` method (additive, does not break consumers).

## 9. Parameters

Existing: `planner_plugin`, `validator_plugin`, `validate_on_hit`.
New:

| Name | Type | Default | Purpose |
| --- | --- | --- | --- |
| `robot_type` | string | `"robot"` | Object type always included in component sub-problems |
| `component_goal_limit` | int | `4` | Max goals per component eligible for composition |
| `component_priority_predicate` | string | `""` | Goal predicate whose components are ordered first |
| `max_exact_cache_entries` | int | `0` | Exact cache bound; 0 = unlimited |
| `max_structural_cache_entries` | int | `0` | Structural cache bound; 0 = unlimited |

## 10. Error handling

- `delegate_plan` still throws `std::runtime_error` when no planner plugin is set; the
  `NoWrappedPlannerThrows` test is preserved.
- Validator rejection of adapted/composed plans falls through/returns false exactly as
  today.
- Single-flight publishes exceptions to waiting followers; the entry is always cleaned up.
- No swallowed exceptions are introduced.

## 11. Testing

Existing 23 tests must pass unchanged. New tests:

1. Adapted structural hit preserves a non-empty `raw_output`.
2. Single-flight: a slow counting mock planner called from two threads for the same
   problem is invoked once.
3. Bounded cache: with `max_*_entries=1`, inserting a second problem evicts the first and
   causes a later miss; metrics reflect evictions.
4. Parallel composition: independent components are solved and the composed plan passes
   the (mock) validator; a shared-robot case stays sequential.
5. Slow-validator concurrency: validation outside the lock lets a writer insert without
   deadlock; validate count is correct.
6. Metrics counters for exact hit, structural hit, miss, adaptation, composition.

## 12. Benchmark

- The original micro-benchmark target `benchmark_cache_planner` was removed after the
  refactor at the user's request. It used a stub delegate to isolate cache overhead and
  its results are retained below as the historical baseline for the acceptance claim.
  The single benchmark now shipped is `benchmark` (§12.1).
- Measures per-thread CPU time with `CLOCK_THREAD_CPUTIME_ID`.
- Scenarios: exact hit, structural hit with renames, miss with a stub delegate,
  composition path, and concurrent multi-thread throughput, over small/medium/large
  synthetic problems.
- Run before and after the refactor; report the numbers with the implementation summary.
- Methodology: both sides were built with `-DCMAKE_BUILD_TYPE=Release` and the benchmark
  disables `RCLCPP_INFO` formatting during the run (`rcutils_logging_set_default_logger_level`)
  so the measured time reflects the cache hot path rather than the logger. An earlier
  comparison against an unoptimized (`-O0`) cache library and with logging enabled made
  the refactor look slower; normalizing those two factors is required for a valid
  comparison.

### Results (2026-09-12, per call, n=2000/200, median of 3 runs)

| Scenario | Pre-refactor | Post-refactor | Delta |
| --- | --- | --- | --- |
| exact hit | 26.8 us | 24.2 us | -10% |
| structural hit | 320.6 us | 241.5 us | -25% |
| miss (stub delegate) | 1221.2 us | 890.8 us | -27% |
| concurrent (4 threads, 2000 calls) | 164.1 ms | 146.7 ms | -11% |

Single runs vary by roughly ±10% on the shared benchmark machine; the medians above are
the basis for the acceptance claim.

### 12.1 Comparison against bare POPF

`benchmark` (same CMake block) compares three real configurations on
identical problem streams, loaded at runtime through pluginlib:

1. `bare-popf` — `omni_plan_popf/PopfPlanner` alone.
2. `cache` — `CachePlanner` wrapping POPF, no validator (`validate_on_hit` irrelevant).
3. `cache+val` — same with `omni_plan_val/ValValidator` (production-like).

Because POPF and VAL run as child processes spawned with `popen`, thread CPU excludes
them; the benchmark reports wall time and child CPU from `getrusage(RUSAGE_CHILDREN)`
deltas. Phases: synthetic exact repeats, structural (isomorphic rename) repeats, unique
structures, a size sweep, a two-component composition problem, and the `cache_demo` room
goal stream. Problems use the valid PDDL convention (predicate arguments are types,
action parameter names have no `?`).

Results (2026-09-12, Release, defaults: 30/30/15/10/10 calls, wall speedup vs bare POPF):

| Phase | cache | cache+val |
| --- | --- | --- |
| exact repeats | 26x | 28x |
| structural repeats | 24x | 0.09x |
| unique structures | ~1.0x | ~1.0x |
| size sweep (25/75/120) | ~1.0x | ~1.0x |
| composition | 9.6x | 0.22x |
| demo goal stream | 17x | 0.34x |

Findings:

- Misses add roughly 1% overhead over bare POPF; composition reuses recurring
  sub-problems and is not a hit-rate cost.
- `cache+val` structural/composed hits re-validate through VAL, which by default enables
  VAL robustness analysis (`-rm m -rd u` is always passed because the default
  `robustness_metric`/`robustness_distribution` strings are non-empty), costing
  ~300 ms per hit on the synthetic plans. For small problems this dominates the ~26 ms
  POPF call and makes validator-backed hits slower than re-planning; larger problems
  shift the crossover. Production can avoid it by clearing those two validator
  parameters or by disabling `validate_on_hit`.
- While investigating, plain VAL reported `Plan failed to execute` for the POPF demo
  plan, whereas VAL with the plugin's default robustness flags reported `1000 plans are
  valid`. This discrepancy is an `omni_plan_val` behaviour outside the cache planner and
  should be investigated separately.

## 13. Risks and mitigations

- **Larger diff** than an in-place patch: mitigated by the stable facade and unchanged
  tests.
- **Parallel path correctness**: only enabled when mutable objects are disjoint and the
  validator confirms the composed plan; otherwise sequential/fallback.
- **Approximate LRU** is not exact LRU: acceptable for an optional, default-off bound;
  documented in the header.
- **Added per-call overhead**: new parameter handling is load-time only; metrics are
  relaxed atomics; no extra locks on the hit path beyond today's.

## 14. Acceptance criteria

1. All pre-existing tests pass unchanged; new tests pass.
2. `HomeostaticPlanner` compiles and its tests pass without source changes.
3. Benchmark shows a measurable CPU reduction for exact hits, structural hits, and
   misses versus the pre-refactor baseline; numbers reported.
4. No behavior change with default parameters other than the documented bug fixes.
5. No commits made.
