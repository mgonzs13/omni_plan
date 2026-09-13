# OmniPlan Full-Repo Review Fix Plan — COMPLETED

> **Status (2026-09-13):** all workstreams executed. Final verification: 21 packages build clean; `colcon test` 554 tests / 0 failures (the single `yolo_ros pytest.missing_result` error is pre-existing and unrelated). No commits were made.

**Baseline:** build OK (21 packages), `colcon test` 467 tests / 0 failures.

**Final:** build OK, 554 tests / 0 failures (87 new regression tests), plus the pre-existing `yolo_ros` error.

**Verification commands (from `/home/miguel/ros2_ws`):**
```bash
source /opt/ros/humble/setup.bash
flock /tmp/opencode/omni_build.lock colcon build --packages-select <pkgs> --event-handlers console_direct+
flock /tmp/opencode/omni_build.lock colcon test --executor sequential --packages-select <pkgs>
colcon test-result --verbose
```

---

## Workstream 0 — Shared core package `omni_plan`

### Task 0.1: Planning graph correctness + memory — DONE
- [x] `get_roots()` no longer stops at the first non-executable action (all initially-executable parallelizable actions become roots).
- [x] Reachability guarantee: every non-root node gets an in-arc (causal, mutex, or conservative serialization fallback); a node with no parents and no processed nodes is promoted to a root.
- [x] Negated conditions linked to the node whose delete effect establishes them.
- [x] Contradiction detection is symmetric and covers start+end effects vs conditions and effect mutex (`actions_conflict`).
- [x] Reference cycles broken in `~PlanningGraph()` (graph destruction frees all nodes).
- [x] Tests: orphan action not dropped, every non-root has an in-arc, negated-condition ordering, end-effect mutex, node release on graph destruction.

### Task 0.2: Planner/validator robustness — DONE
- [x] Unknown action line ⇒ `has_solution=false`; success marker with zero parsed actions ⇒ invalid.
- [x] Predictable temp files replaced by private per-call temp dirs (`mkdtemp`, 0600, `TempFileGuard`/`TempDirGuard`); planner/validator cannot collide.
- [x] Tests: unknown action, zero-action solution, private+removed files, concurrent calls.

### Task 0.3: `pddl_manager.cpp` effect handling — DONE
- [x] `predicate_exists` evaluated once per effect.
- [x] Already-true positive effects still invoke `apply_effect` when the predicate is a goal (KB goal consumption), without recording a no-op for undo.
- [x] Undo is the exact inverse of applied operations.
- [x] Tests: evaluate-once, goal consumption, undo correctness.

### Task 0.4: Header API fixes — DONE
- [x] `action.hpp`: single-argument constructor ambiguity removed.
- [x] `object.hpp`: `operator==` consistent with `operator<` (name+type).
- [x] `parameter_loader.hpp`: copy disabled (setters bind to owner members); concurrent declare race caught.
- [x] `timing_predicate.cpp`, `action.cpp`, `predicate.cpp`: no `??x` for names that already carry `?`.
- [x] `package.xml`/`CMakeLists.txt`: `builtin_interfaces` declared/found/linked.

### Task 0.5: PlanDispatcher base — DONE
- [x] Monitor thread stopped/joined on every path, including exceptions.
- [x] Cancel flag set before snapshot; membership re-checked before `cancel()` so reused cached instances are never cancelled.
- [x] `pluginlib` create/param exceptions caught; null instance handled by dispatchers.
- [x] `SKIPPED` handled explicitly (start effects rolled back, status reported), mapped to node SKIPPED in both dispatchers.
- [x] Node numbering validated (unique, in range) before use; final FAILED status published on every early return.

### Task 0.6: States + FSM — DONE
- [x] `clear_goals_state`: propagates failure.
- [x] `load_plugins_state`: ABORT on empty required plugin params/nulls; duplicate action names rejected.
- [x] `idle_state`: null manager ⇒ aborted.
- [x] `plan_state`: null planner guard.
- [x] `publish_pddl_state`: blackboard errors produce ABORT instead of a partial publish.
- [x] `dispatch_plan_state`: reads/graph build inside try/catch; `dispatcher_` mutex-protected.
- [x] `validate_plan_state`: infrastructure failures aborted, `invalid` reserved for real validation failures.
- [x] XMLs: `canceled` transitions added; cancel no longer immediately replans; unsupported `frequency` attribute removed; outcome/transition cross-check + strict validation pass.

### Task 0.7: Core tests — DONE

---

## Workstream 1 — `omni_plan_cache` — DONE
- [x] Structural key includes predicate polarity.
- [x] Composition honors `should_cache_result()`.
- [x] Re-entrant flight: `Flight::owns_flight`; only the real leader publishes.
- [x] Leader re-checks the cache after winning the flight (check-then-act closed).
- [x] Follower fallback re-enters `begin_or_join` instead of hijacking a live flight.
- [x] `plan_cache_.configure()` before the validator early-return.
- [x] Dead/uninitialized `abstract_role_keys_` removed.
- [x] Exact key hashes float bits (sub-microsecond durations distinguished).
- [x] `full_misses` counts real leaders; benchmark counts wrapped-planner calls explicitly.
- [x] `compute_structural_key(string)` and Domain overload produce identical keys.
- [x] Unmapped plan parameters force validation.
- [x] `component_priority_predicate` default restored to `battery_ok` and documented.
- [~] LRU eviction kept as documented O(n) scan (only runs with a configured bound); explanatory comment added.

## Workstream 2 — `omni_plan_mrta` + `omni_plan_homeostatic` — DONE
- [x] Selector UB (`end()` deref before guard) fixed; empty-planner guard added.
- [x] Selector score incorporates success rate (never-succeeded planners treated as worst).
- [x] Homeostatic: per-planner try/catch; bounded POIROT wait with wall-clock fallback; unique profiler names; selector lazy-init/guard; RAII profiler stop; `std::move` warning removed.
- [x] CBBA/Coalition constructor values survive parameter loading.
- [x] MrtaPlanner: uncovered goals ⇒ `has_solution()==false`; exactly-once coverage; stable merge sort.
- [x] Greedy allocator unreachable sentinel consistent; bid arithmetic computed in `long long` with saturation.
- [x] Homeostatic package declares/links/exports `poirot` and `omni_plan_cache`.

## Workstream 3 — Knowledge — DONE
- [x] `has_goals()` non-blocking fast path with bounded wait; no KG/service calls under `goal_mutex_` (deadlock removed).
- [x] KG callback lifetime token prevents use-after-free.
- [x] KB destruction order fixed (callback removed, client joined while mutex/cv alive).
- [x] `knowledge_base_node_lib` installed/exported.
- [x] KG predicates deduplicated by signature with conflict warning.
- [x] `apply_effect` preserves existing edge properties; `std::get_if` guard.
- [x] Fact/goal arity+type validation; goal erased only on newly inserted fact.
- [x] `clear_goals` propagates failures; bounded `wait_for_service`; callback snapshot outside lock + unregister API.
- [x] Timing enum converted explicitly by name (send path).
- [x] Packaging deps (`rclcpp`, `omni_plan_msgs`, `pluginlib` export).
- [~] Timing on the return path cannot be preserved because `KnowledgeBase` stores plain `Predicate` (no time field); documented as an API limitation.

## Workstream 4 — Planner wrappers — DONE
- [x] VHPOP accepts classical (bracket-less) plan lines.
- [x] Exit status checked (`pclose`/`system`) in all wrappers.
- [x] All shell arguments quoted; enum parameters allow-listed.
- [x] VAL robustness opt-in; singular "1 plan is valid" parsed; `using PlanValidator::validate_plan`.
- [x] LPG: `mkdtemp` temp dir, all `*_N.SOL` cleaned, `tolower` UB fixed, params no longer lowercased.
- [x] SMTP/VAL stderr separated from parsed stdout.
- [x] POPF dead options removed.
- [x] Child `timeout` parameter added (system `timeout -k 5`, 0 = disabled).
- [x] COLIN/OPTIC tests assert real solutions (fixtures corrected to valid PDDL naming).

## Workstream 5 — Actions + Dispatcher package — DONE
- [x] `BtAction` atomic initialized and cancellation reset per run (no poisoned cached instances).
- [x] `YasminFactoryAction` null state-machine viewer guard.
- [x] `YasminAction` viewer reference cycle removed.
- [x] Parallel dispatcher: worker exception boundary, promise resolution on all paths, recursive SKIPPED subtree resolution, cancel aggregation, pool capped by node count, node-number validation.
- [x] Sequential dispatcher: remaining nodes marked SKIPPED, final status published.

## Workstream 6 — TUI / demos / bringup / tests / CI — DONE
- [x] TUI: async-safe signal handler, empty-`states` guard, `current_state >= 0`, scroll clamp, FSM depth via parent walk, negative column guard, log before ncurses init.
- [~] TUI UTF-8-safe clipping/width-relative plan columns: documented as a known limitation (comment), not restructured.
- [x] Demos: direct deps declared, Groot ports made unique.
- [x] Bringup: `IfCondition` (no `PythonExpression`), runtime dependencies added.
- [x] `test_full_system`: tracked PIDs, process-group kill, unique ROS domain, no service calls after kill.
- [x] COLIN/OPTIC integration assertions now fail when the solver cannot run.
- [x] CI: TUI/demos/bringup/msgs built; libCbc installed; Docker distro-aware CBC + used `CMAKE_BUILD_TYPE`; release workflow injection fixed; Doxyfile paths corrected.
- [~] Foxy/Galactic EOL workflows left as-is (out of scope behaviour change; noted in review).

## Final verification — PASSED
- [x] `colcon build` all 21 packages clean.
- [x] `colcon test` all omni packages: 554 tests, 0 failures (baseline 467, +87 new tests).
- [x] Benchmark and graph spot-checks run.
