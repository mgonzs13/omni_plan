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
//
// Compares three real planner configurations on identical problem streams:
//   1. bare-popf   - omni_plan_popf/PopfPlanner alone
//   2. cache       - CachePlanner wrapping PopfPlanner (no validator)
//   3. cache+val   - CachePlanner wrapping PopfPlanner with
//                    omni_plan_val/ValValidator
//
// Because POPF (and VAL) run as child processes spawned with popen(), thread
// CPU time does not include them. This benchmark therefore reports wall time
// and child CPU time obtained from getrusage(RUSAGE_CHILDREN) deltas, which
// capture the planner/validator subprocesses (~0 for cache hits).
//
// Build the package with -DCMAKE_BUILD_TYPE=Release for meaningful numbers.
// Requires the installed omni_plan_popf and omni_plan_val packages, which are
// loaded at runtime through pluginlib.

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <sys/resource.h>
#include <vector>

#include <pluginlib/class_loader.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>

#include "omni_plan/pddl/action.hpp"
#include "omni_plan/pddl/domain.hpp"
#include "omni_plan/pddl/object.hpp"
#include "omni_plan/pddl/plan.hpp"
#include "omni_plan/pddl/predicate.hpp"
#include "omni_plan/pddl/problem.hpp"
#include "omni_plan/planner.hpp"
#include "omni_plan_cache/cache_planner.hpp"

using namespace omni_plan;
using namespace omni_plan_cache;

namespace {

/// @brief Action used only to build PDDL domains; never executed.
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

// ---------------------------------------------------------------------------
// Timing helpers
// ---------------------------------------------------------------------------

/// @brief Wall-clock time in microseconds.
double wall_us() {
  return static_cast<double>(
             std::chrono::duration_cast<std::chrono::microseconds>(
                 std::chrono::steady_clock::now().time_since_epoch())
                 .count());
}

/// @brief Cumulative CPU time of all reaped child processes, in microseconds.
double child_cpu_us() {
  struct rusage usage;
  getrusage(RUSAGE_CHILDREN, &usage);
  return (static_cast<double>(usage.ru_utime.tv_sec) +
          static_cast<double>(usage.ru_stime.tv_sec)) *
             1e6 +
         static_cast<double>(usage.ru_utime.tv_usec) +
         static_cast<double>(usage.ru_stime.tv_usec);
}

/// @brief Nearest-rank percentile of a copy of @p values.
double percentile(std::vector<double> values, double pct) {
  if (values.empty()) {
    return 0.0;
  }
  std::sort(values.begin(), values.end());
  const size_t rank = static_cast<size_t>(pct / 100.0 * values.size());
  return values[std::min(rank, values.size() - 1)];
}

std::string speedup(double base, double value) {
  if (value <= 0.0) {
    return "inf";
  }
  char buffer[32];
  std::snprintf(buffer, sizeof(buffer), "%.2fx", base / value);
  return buffer;
}

// ---------------------------------------------------------------------------
// Domains and problems (valid PDDL convention: predicate arguments are types,
// action parameter names carry no leading '?')
// ---------------------------------------------------------------------------

pddl::Domain make_synthetic_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_requirement("durative-actions");
  domain.add_type("location");
  domain.add_type("robot");
  domain.add_predicate(pddl::Predicate("at", {"robot", "location"}));
  domain.add_predicate(pddl::Predicate("connected", {"location", "location"}));
  auto move = std::make_shared<BenchAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"r", "robot"}, {"from", "location"}, {"to", "location"}});
  move->add_condition(pddl::Type::START, "at", {"r", "from"});
  move->add_condition(pddl::Type::START, "connected", {"from", "to"});
  move->add_effect(pddl::Type::START, "at", {"r", "from"}, true);
  move->add_effect(pddl::Type::END, "at", {"r", "to"});
  domain.add_action(move);
  return domain;
}

/// @brief One robot and a chain of @p n locations; goal at the far end.
pddl::Problem make_chain_problem(int n, const std::string &suffix) {
  pddl::Problem problem;
  const std::string robot = "r" + suffix;
  problem.add_object(pddl::Object(robot, "robot"));
  for (int i = 0; i < n; ++i) {
    problem.add_object(
        pddl::Object("l" + suffix + "_" + std::to_string(i), "location"));
  }
  problem.add_fact(pddl::Predicate("at", {robot, "l" + suffix + "_0"}));
  for (int i = 0; i + 1 < n; ++i) {
    problem.add_fact(pddl::Predicate(
        "connected", {"l" + suffix + "_" + std::to_string(i),
                      "l" + suffix + "_" + std::to_string(i + 1)}));
  }
  problem.add_goal(pddl::Predicate(
      "at", {robot, "l" + suffix + "_" + std::to_string(n - 1)}));
  return problem;
}

/// @brief Two robots with independent goals, i.e. two composable components.
pddl::Problem make_composition_problem(const std::string &suffix) {
  pddl::Problem problem;
  const std::string r1 = "ra" + suffix;
  const std::string r2 = "rb" + suffix;
  problem.add_object(pddl::Object(r1, "robot"));
  problem.add_object(pddl::Object(r2, "robot"));
  for (const auto *name : {"a0", "a1", "b0", "b1"}) {
    problem.add_object(pddl::Object(name + suffix, "location"));
  }
  problem.add_fact(pddl::Predicate("at", {r1, "a0" + suffix}));
  problem.add_fact(pddl::Predicate("at", {r2, "b0" + suffix}));
  problem.add_fact(
      pddl::Predicate("connected", {"a0" + suffix, "a1" + suffix}));
  problem.add_fact(
      pddl::Predicate("connected", {"b0" + suffix, "b1" + suffix}));
  problem.add_goal(pddl::Predicate("at", {r1, "a1" + suffix}));
  problem.add_goal(pddl::Predicate("at", {r2, "b1" + suffix}));
  return problem;
}

/// @brief Demo-like domain: rooms graph with a battery condition on move.
pddl::Domain make_demo_domain() {
  pddl::Domain domain;
  domain.add_requirement("strips");
  domain.add_requirement("typing");
  domain.add_requirement("negative-preconditions");
  domain.add_requirement("durative-actions");
  domain.add_type("robot");
  domain.add_type("room");
  domain.add_predicate(pddl::Predicate("connected", {"room", "room"}));
  domain.add_predicate(pddl::Predicate("charging_point_at", {"room"}));
  domain.add_predicate(pddl::Predicate("battery_low", {"robot"}));
  domain.add_predicate(pddl::Predicate("battery_full", {"robot"}));
  domain.add_predicate(pddl::Predicate("robot_at", {"robot", "room"}));
  auto move = std::make_shared<BenchAction>(
      "move", std::vector<std::pair<std::string, std::string>>{
                  {"robot", "robot"}, {"r1", "room"}, {"r2", "room"}});
  move->add_condition(pddl::Type::START, "robot_at", {"robot", "r1"});
  move->add_condition(pddl::Type::OVER_ALL, "battery_full", {"robot"});
  move->add_condition(pddl::Type::START, "connected", {"r1", "r2"});
  move->add_effect(pddl::Type::START, "robot_at", {"robot", "r1"}, true);
  move->add_effect(pddl::Type::END, "robot_at", {"robot", "r2"});
  domain.add_action(move);
  return domain;
}

/// @brief The cache_demo room graph, robot at @p origin, goal @p goal_room.
pddl::Problem make_demo_problem(const std::string &goal_room,
                                const std::string &origin) {
  pddl::Problem problem;
  problem.add_object(pddl::Object("leia", "robot"));
  for (const auto *room : {"entrance", "kitchen", "bedroom", "dinning",
                           "bathroom", "chargingroom"}) {
    problem.add_object(pddl::Object(room, "room"));
  }
  const std::vector<std::pair<std::string, std::string>> edges = {
      {"entrance", "dinning"},     {"dinning", "entrance"},
      {"dinning", "kitchen"},      {"kitchen", "dinning"},
      {"dinning", "bedroom"},      {"bedroom", "dinning"},
      {"chargingroom", "bedroom"}, {"bedroom", "chargingroom"},
      {"chargingroom", "kitchen"}, {"kitchen", "chargingroom"},
      {"bathroom", "chargingroom"}, {"chargingroom", "bathroom"}};
  for (const auto &[from, to] : edges) {
    problem.add_fact(pddl::Predicate("connected", {from, to}));
  }
  problem.add_fact(pddl::Predicate("charging_point_at", {"chargingroom"}));
  problem.add_fact(pddl::Predicate("battery_full", {"leia"}));
  problem.add_fact(pddl::Predicate("robot_at", {"leia", origin}));
  problem.add_goal(pddl::Predicate("robot_at", {"leia", goal_room}));
  return problem;
}

// ---------------------------------------------------------------------------
// Measurement
// ---------------------------------------------------------------------------

struct Sample {
  double wall_us = 0.0;
  double cpu_us = 0.0;
  bool solved = false;
};

struct PhaseResult {
  std::string phase;
  std::string config;
  size_t calls = 0;
  size_t solved = 0;
  uint64_t popf_calls = 0;
  double wall_total_ms = 0.0;
  double wall_p50_ms = 0.0;
  double wall_p95_ms = 0.0;
  double cpu_total_ms = 0.0;
  double cpu_p50_ms = 0.0;
  double cpu_p95_ms = 0.0;
};

struct Config {
  std::string name;
  std::shared_ptr<Planner> planner;
  std::shared_ptr<CachePlanner> cache;
};

struct Args {
  int exact = 30;
  int structural = 30;
  int unique = 15;
  int composition = 10;
  int rounds = 10;
};

Args parse_args(int argc, char **argv) {
  Args args;
  auto read_int = [&](int &slot, int index) {
    slot = std::atoi(argv[index]);
    if (slot < 1) {
      slot = 1;
    }
  };
  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--exact" && i + 1 < argc) {
      read_int(args.exact, ++i);
    } else if (arg == "--structural" && i + 1 < argc) {
      read_int(args.structural, ++i);
    } else if (arg == "--unique" && i + 1 < argc) {
      read_int(args.unique, ++i);
    } else if (arg == "--composition" && i + 1 < argc) {
      read_int(args.composition, ++i);
    } else if (arg == "--rounds" && i + 1 < argc) {
      read_int(args.rounds, ++i);
    } else if (arg == "--help") {
      std::printf(
          "usage: benchmark [options]\n"
          "  --exact N        identical-problem calls (default 30)\n"
          "  --structural N   isomorphic renamed calls (default 30)\n"
          "  --unique N       distinct-structure calls (default 15)\n"
          "  --composition N  multi-component calls (default 10)\n"
          "  --rounds N       demo goal-stream rounds (default 10)\n");
      std::exit(0);
    } else {
      std::fprintf(stderr, "Unknown option: %s (try --help)\n", arg.c_str());
      std::exit(1);
    }
  }
  return args;
}

PhaseResult run_phase(const Config &config, const std::string &phase,
                      const pddl::Domain &domain,
                      const std::vector<pddl::Problem> &problems) {
  CacheStats before;
  if (config.cache) {
    before = config.cache->get_cache_stats();
  }

  std::vector<Sample> samples;
  samples.reserve(problems.size());
  for (const auto &problem : problems) {
    const double cpu_0 = child_cpu_us();
    const double wall_0 = wall_us();
    const pddl::Plan plan = config.planner->generate_plan(domain, problem);
    const double wall_1 = wall_us();
    const double cpu_1 = child_cpu_us();
    samples.push_back({wall_1 - wall_0, cpu_1 - cpu_0, plan.has_solution()});
  }

  PhaseResult result;
  result.phase = phase;
  result.config = config.name;
  result.calls = samples.size();
  result.popf_calls =
      config.cache
          ? config.cache->get_cache_stats().full_misses - before.full_misses
          : samples.size();

  std::vector<double> walls;
  std::vector<double> cpus;
  walls.reserve(samples.size());
  cpus.reserve(samples.size());
  for (const auto &sample : samples) {
    walls.push_back(sample.wall_us);
    cpus.push_back(sample.cpu_us);
    result.wall_total_ms += sample.wall_us / 1000.0;
    result.cpu_total_ms += sample.cpu_us / 1000.0;
    if (sample.solved) {
      result.solved++;
    }
  }
  result.wall_p50_ms = percentile(walls, 50.0) / 1000.0;
  result.wall_p95_ms = percentile(walls, 95.0) / 1000.0;
  result.cpu_p50_ms = percentile(cpus, 50.0) / 1000.0;
  result.cpu_p95_ms = percentile(cpus, 95.0) / 1000.0;
  return result;
}

void print_results(const std::vector<PhaseResult> &results) {
  std::printf("\n%-32s %-10s %6s %6s %6s %12s %10s %10s %12s %10s %9s %9s\n",
              "phase", "config", "calls", "popf", "solved", "wall_tot_ms",
              "wall_p50", "wall_p95", "cpu_tot_ms", "cpu_p50", "wall_x",
              "cpu_x");
  std::printf("%s\n", std::string(150, '-').c_str());

  std::string current_phase;
  for (const auto &result : results) {
    if (!current_phase.empty() && result.phase != current_phase) {
      std::printf("\n");
    }
    current_phase = result.phase;

    const PhaseResult *bare = nullptr;
    for (const auto &candidate : results) {
      if (candidate.phase == result.phase &&
          candidate.config == "bare-popf") {
        bare = &candidate;
        break;
      }
    }

    std::string wall_x = "-";
    std::string cpu_x = "-";
    if (bare != nullptr && result.config != "bare-popf") {
      wall_x = speedup(bare->wall_total_ms, result.wall_total_ms);
      cpu_x = speedup(bare->cpu_total_ms, result.cpu_total_ms);
    }

    std::printf("%-32s %-10s %6zu %6llu %6zu %12.2f %10.2f %10.2f %12.2f "
                "%10.2f %9s %9s\n",
                result.phase.c_str(), result.config.c_str(), result.calls,
                static_cast<unsigned long long>(result.popf_calls),
                result.solved, result.wall_total_ms, result.wall_p50_ms,
                result.wall_p95_ms, result.cpu_total_ms, result.cpu_p50_ms,
                wall_x.c_str(), cpu_x.c_str());
  }
}

void print_cache_stats(const std::vector<Config> &configs) {
  std::printf("\ncache statistics (cumulative over the whole run)\n");
  for (const auto &config : configs) {
    if (!config.cache) {
      continue;
    }
    const CacheStats stats = config.cache->get_cache_stats();
    std::printf(
        "%-10s exact_hits=%llu exact_misses=%llu structural_hits=%llu "
        "full_misses=%llu adaptations=%llu validations=%llu "
        "compositions=%llu composition_fallbacks=%llu entries=%zu/%zu\n",
        config.name.c_str(),
        static_cast<unsigned long long>(stats.exact_hits),
        static_cast<unsigned long long>(stats.exact_misses),
        static_cast<unsigned long long>(stats.structural_hits),
        static_cast<unsigned long long>(stats.full_misses),
        static_cast<unsigned long long>(stats.adaptations),
        static_cast<unsigned long long>(stats.validations),
        static_cast<unsigned long long>(stats.compositions),
        static_cast<unsigned long long>(stats.composition_fallbacks),
        stats.exact_entries, stats.structural_entries);
  }
  std::printf(
      "\nnotes:\n"
      "  popf = planner subprocess invocations (cache: full_misses delta)\n"
      "  wall/cpu totals and percentiles are per phase; p50/p95 are per call\n"
      "  child CPU comes from getrusage(RUSAGE_CHILDREN) and includes POPF\n"
      "  and, for cache+val, the omni_plan_val/ValValidator process; cache\n"
      "  hits without validation have ~0 child CPU\n"
      "  cache+val re-validates structural/composed hits (validate_on_hit),\n"
      "  so its hit cost includes VAL runtime\n");
}

std::vector<pddl::Problem> make_demo_stream(int rounds) {
  const std::vector<std::string> goals = {"kitchen", "bedroom", "kitchen",
                                          "bedroom"};
  std::vector<pddl::Problem> problems;
  problems.reserve(static_cast<size_t>(rounds) * goals.size());
  std::string origin = "entrance";
  for (int round = 0; round < rounds; ++round) {
    for (const auto &goal : goals) {
      problems.push_back(make_demo_problem(goal, origin));
      origin = goal;
    }
  }
  return problems;
}

} // namespace

int main(int argc, char **argv) {
  const Args args = parse_args(argc, argv);

  rclcpp::init(argc, argv);
  rcutils_logging_set_default_logger_level(RCUTILS_LOG_SEVERITY_WARN);

  const std::vector<rclcpp::Parameter> overrides = {
      rclcpp::Parameter("cache_no_val.planner_plugin",
                        "omni_plan_popf/PopfPlanner"),
      rclcpp::Parameter("cache_no_val.validator_plugin", ""),
      rclcpp::Parameter("cache_with_val.planner_plugin",
                        "omni_plan_popf/PopfPlanner"),
      rclcpp::Parameter("cache_with_val.validator_plugin",
                        "omni_plan_val/ValValidator")};
  auto node = std::make_shared<rclcpp::Node>(
      "bench_cache_vs_popf",
      rclcpp::NodeOptions().parameter_overrides(overrides));

  pluginlib::ClassLoader<omni_plan::Planner> loader("omni_plan",
                                                    "omni_plan::Planner");

  std::vector<Config> configs;

  {
    Config config;
    config.name = "bare-popf";
    config.planner = loader.createSharedInstance("omni_plan_popf/PopfPlanner");
    config.planner->set_namespace("bare_popf");
    config.planner->load_ros_parameters(node);
    configs.push_back(std::move(config));
  }
  {
    Config config;
    config.name = "cache";
    config.cache = std::make_shared<CachePlanner>();
    config.cache->set_namespace("cache_no_val");
    config.cache->load_ros_parameters(node);
    config.planner = config.cache;
    configs.push_back(std::move(config));
  }
  {
    Config config;
    config.name = "cache+val";
    config.cache = std::make_shared<CachePlanner>();
    config.cache->set_namespace("cache_with_val");
    config.cache->load_ros_parameters(node);
    config.planner = config.cache;
    configs.push_back(std::move(config));
  }

  const auto synthetic = make_synthetic_domain();
  const auto demo = make_demo_domain();

  struct Phase {
    std::string name;
    const pddl::Domain *domain;
    std::vector<pddl::Problem> problems;
  };
  std::vector<Phase> phases;

  {
    const pddl::Problem problem = make_chain_problem(40, "_ex");
    phases.push_back({"synthetic exact repeats", &synthetic,
                      std::vector<pddl::Problem>(args.exact, problem)});
  }
  {
    std::vector<pddl::Problem> problems;
    for (int i = 0; i < args.structural; ++i) {
      problems.push_back(make_chain_problem(41, "_st" + std::to_string(i)));
    }
    phases.push_back(
        {"synthetic structural repeats", &synthetic, std::move(problems)});
  }
  {
    std::vector<pddl::Problem> problems;
    for (int i = 0; i < args.unique; ++i) {
      problems.push_back(make_chain_problem(50 + i, "_uq" + std::to_string(i)));
    }
    phases.push_back(
        {"synthetic unique structures", &synthetic, std::move(problems)});
  }
  {
    std::vector<pddl::Problem> problems;
    for (const int n : {25, 75, 120}) {
      problems.push_back(make_chain_problem(n, "_sz" + std::to_string(n)));
    }
    phases.push_back({"synthetic size sweep", &synthetic, std::move(problems)});
  }
  {
    std::vector<pddl::Problem> problems;
    for (int i = 0; i < args.composition; ++i) {
      problems.push_back(make_composition_problem("_cp" + std::to_string(i)));
    }
    phases.push_back(
        {"synthetic composition", &synthetic, std::move(problems)});
  }
  phases.push_back({"demo goal stream", &demo, make_demo_stream(args.rounds)});

  // Warm-up: pay plugin loading / first-call costs outside the measurements.
  const pddl::Problem warmup = make_chain_problem(3, "_warm");
  for (const auto &config : configs) {
    config.planner->generate_plan(synthetic, warmup);
  }

  std::vector<PhaseResult> results;
  results.reserve(phases.size() * configs.size());
  for (const auto &phase : phases) {
    for (const auto &config : configs) {
      results.push_back(
          run_phase(config, phase.name, *phase.domain, phase.problems));
    }
  }

  print_results(results);
  print_cache_stats(configs);

  rclcpp::shutdown();
  return 0;
}
