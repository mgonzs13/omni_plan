# OmniPlan

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

---

OmniPlan is a ROS 2 framework for automated task planning and execution. It integrates multiple classical planners with flexible execution mechanisms including direct action implementation, state machines and behavior trees. The framework supports both knowledge base and knowledge graph approaches for state management, enabling planning solutions for robotic applications. Finally, OmniPlan can be extended through the creation of new plugins to integrate new planners and new knowledge sources.

---

<div align="center">

[![License: GPL-v3.0](https://img.shields.io/badge/GitHub-GPL--3.0-informational)](https://opensource.org/license/gpl-3-0)
[![GitHub release](https://img.shields.io/github/release/mgonzs13/omni_plan.svg)](https://github.com/mgonzs13/omni_plan/releases)
[![Code Size](https://img.shields.io/github/languages/code-size/mgonzs13/omni_plan.svg?branch=main)](https://github.com/mgonzs13/omni_plan?branch=main)
[![Last Commit](https://img.shields.io/github/last-commit/mgonzs13/omni_plan.svg)](https://github.com/mgonzs13/omni_plan/commits/main)

[![GitHub issues](https://img.shields.io/github/issues/mgonzs13/omni_plan)](https://github.com/mgonzs13/omni_plan/issues)
[![GitHub pull requests](https://img.shields.io/github/issues-pr/mgonzs13/omni_plan)](https://github.com/mgonzs13/omni_plan/pulls)
[![Contributors](https://img.shields.io/github/contributors/mgonzs13/omni_plan.svg)](https://github.com/mgonzs13/omni_plan/graphs/contributors)

[![Python Formatter Check](https://github.com/mgonzs13/omni_plan/actions/workflows/python-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/python-formatter.yml?branch=main)
[![C++ Formatter Check](https://github.com/mgonzs13/omni_plan/actions/workflows/cpp-formatter.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/cpp-formatter.yml?branch=main)

| ROS 2 Distro |                                                                                                       Build and Test                                                                                                       |
| :----------: | :------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------: |
|   **Foxy**   |        [![Foxy Build](https://github.com/mgonzs13/omni_plan/actions/workflows/foxy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/foxy-build-test.yml?branch=main)         |
| **Galactic** |  [![Galactic Build](https://github.com/mgonzs13/omni_plan/actions/workflows/galactic-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/galactic-build-test.yml?branch=main)   |
|  **Humble**  | [![Humble Build and Test](https://github.com/mgonzs13/omni_plan/actions/workflows/humble-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/humble-build-test.yml?branch=main) |
|   **Iron**   |        [![Iron Build](https://github.com/mgonzs13/omni_plan/actions/workflows/iron-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/iron-build-test.yml?branch=main)         |
|  **Jazzy**   |       [![Jazzy Build](https://github.com/mgonzs13/omni_plan/actions/workflows/jazzy-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/jazzy-build-test.yml?branch=main)       |
|  **Kilted**  |     [![Kilted Build](https://github.com/mgonzs13/omni_plan/actions/workflows/kilted-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/kilted-build-test.yml?branch=main)      |
| **Rolling**  |    [![Rolling Build](https://github.com/mgonzs13/omni_plan/actions/workflows/rolling-build-test.yml/badge.svg?branch=main)](https://github.com/mgonzs13/omni_plan/actions/workflows/rolling-build-test.yml?branch=main)    |

</div align="center">

## Table of Contents

- [Key Features](#key-features)
- [Installation](#installation)
- [Demos](#demos)
  - [Knowledge Graph with POPF Planner](#knowledge-graph-with-popf-planner)
  - [Knowledge Base with POPF Planner](#knowledge-base-with-popf-planner)
  - [Parallel Assembly Demo](#parallel-assembly-demo)
  - [Other Demos](#other-demos)
  - [Adding Knowledge](#adding-knowledge)
- [OmniPlan TUI Monitor](#omniplan-tui-monitor)
- [API Development](#api-development)
  - [Creating New PDDL Managers](#creating-new-pddl-managers)
  - [Creating New Planners](#creating-new-planners)
  - [Creating New MRTA Task Allocators](#creating-new-mrta-task-allocators)
    - [Spatial Proximity Model](#spatial-proximity-co-occurrence-graph-and-bfs-distance)
    - [Round-Robin Allocator](#1-round-robin-allocator)
    - [SSI Affinity Allocator](#2-ssi-affinity-allocator)
    - [Greedy Auction Allocator](#3-greedy-auction-allocator)
    - [CBBA Allocator](#4-cbba-allocator)
    - [Coalition Formation Allocator](#5-coalition-formation-allocator)
  - [Creating New Plan Validators](#creating-new-plan-validators)
  - [Creating New Plan Dispatchers](#creating-new-plan-dispatchers)
  - [Creating New Actions](#creating-new-actions)
    - [Regular Omni Plan Actions](#regular-omni-plan-actions)
    - [YASMIN Actions](#yasmin-actions)
    - [YASMIN Factory Actions](#yasmin-factory-actions)
    - [Behavior Tree Actions](#behavior-tree-actions)

## Key Features

- **Plugin Architecture**: Extensible design using ROS 2 pluginlib for omni customization.
- **Multiple PDDL Planners**: Support for POPF, SMTP, VHPOP, Colin, LPG, and OPTIC planners, plus the VAL plan validator.
- **Flexible Execution**: Execute plans using direct actions, YASMIN state machines or Behavior Trees.
- **Knowledge Management**: Choose between knowledge base or knowledge graph approaches or integrate your own implementation.
- **Multi-Robot Task Allocation (MRTA)**: Built-in allocator plugins (Round-Robin, SSI Affinity, Greedy Auction, CBBA, and Coalition Formation) to distribute goals across robot fleets.
- **ROS 2 Native**: Built on ROS 2 with proper message interfaces.

## Installation

```shell
# Clone this repo
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/omni_plan
# Install dependecies
cd ~/ros2_ws
vcs import src < src/omni_plan/dependencies.repos
rosdep install --from-paths src --ignore-src -r -y
# SMTPlan+ dependency
sudo apt install libz3-dev -y
# Colin and OPTIC dependecy
sudo apt install coinor-libcbc3 -y
# Optional: You may need to create symbolic link
sudo ln -s /usr/lib/x86_64-linux-gnu/libCbc.so.3.10.11 /usr/lib/x86_64-linux-gnu/libCbc.so.3
# Build the workspace
colcon build --symlink-install
```

To run the tests:

```shell
colcon test --executor sequential --packages-select omni_plan omni_plan_knowledge_base omni_plan_knowledge_graph omni_plan_popf omni_plan_vhpop omni_plan_smtp omni_plan_optic omni_plan_lpg omni_plan_colin omni_plan_mrta omni_plan_val omni_plan_dispatcher omni_plan_yasmin omni_plan_bt omni_plan_tests
colcon test-result --verbose
```

## Demos

https://github.com/user-attachments/assets/ded56c3f-1d74-451e-b317-48be318b2f2b

The framework includes several demo packages showcasing different planning and execution approaches:

### Knowledge Graph with POPF Planner

```shell
ros2 launch omni_plan_demos popf_kg_demo.launch.py
```

### Knowledge Base with POPF Planner

```shell
ros2 launch omni_plan_demos popf_kb_demo.launch.py
```

### Other Demos

- `smtp_kb_demo.launch.py` / `smtp_kg_demo.launch.py`: SMTP planner demos
- `vhpop_kb_demo.launch.py` / `vhpop_kg_demo.launch.py`: VHPOP planner demos
- `lpg_kb_demo.launch.py` / `lpg_kg_demo.launch.py`: LPG planner demos
- `optic_kb_demo.launch.py` / `optic_kg_demo.launch.py`: OPTIC planner demos

### Adding Knowledge

```shell
ros2 run omni_plan_demos knowledge_graph_demo
```

### Parallel Assembly Demo

Demonstrates concurrent action execution using the knowledge graph. Three
robots pick up parts in parallel, then a convergence step assembles them once
all predecessors have completed.

```shell
ros2 launch omni_plan_demos popf_assembly_demo.launch.py
```

The PDDL domain models `pick-up` (per-robot) and `assemble` (requires all
parts) as durative actions with OVER_ALL battery conditions so the planner
schedules them as a true parallel graph.

```shell
ros2 run omni_plan_demos assembly_demo
```

## OmniPlan TUI Monitor

The `omni_plan_tui` package provides a terminal-based monitoring interface for
active plan execution. It subscribes to three topics and renders a live,
colour-coded view in the terminal using ncurses.

### Launch

```shell
ros2 run omni_plan_tui omni_plan_tui_node
```

### Tabs

| Tab                | Key | Description                                                                           |
| ------------------ | --- | ------------------------------------------------------------------------------------- |
| **Plan Execution** | `1` | Level-grouped graph view of all actions with real-time status icons and elapsed times |
| **FSM State**      | `2` | Current YASMIN state machine state and full hierarchy                                 |
| **Action Catalog** | `3` | Complete list of loaded plugin actions with parameters, conditions and effects        |

### Key Bindings

| Key                   | Action                           |
| --------------------- | -------------------------------- |
| `q` / `Q`             | Quit                             |
| `1` / `2` / `3`       | Jump to Plan / FSM / Actions tab |
| `Tab` / `]`           | Next tab                         |
| `[`                   | Previous tab                     |
| `↑` / `↓`             | Scroll one row                   |
| `PgUp` / `PgDn`       | Scroll ten rows                  |
| `Home` / `End`        | Jump to top / bottom             |
| Mouse click (tab bar) | Switch to clicked tab            |

### Topics Subscribed

| Topic                       | Message type                         | QoS                       |
| --------------------------- | ------------------------------------ | ------------------------- |
| `/omni_plan/actions_info`   | `omni_plan_msgs/ActionInfoArray`     | Transient-local (latched) |
| `/omni_plan/plan_execution` | `omni_plan_msgs/PlanExecutionStatus` | Default                   |
| `/fsm_viewer`               | `yasmin_msgs/StateMachine`           | Default                   |

## API Development

OmniPlan uses a plugin-based architecture that allows developers to extend the framework by creating new planners, plan validators, and actions. All plugins are loaded using ROS2's pluginlib system.

### Creating New PDDL Managers

PDDL managers handle domain and problem generation from action definitions and manage the current world state. They support different state representation approaches (e.g., knowledge base vs. knowledge graph). Inherit from `omni_plan::PddlManager`:

```cpp
#include "omni_plan/pddl_manager.hpp"

class MyPddlManager : public omni_plan::PddlManager {
public:
  MyPddlManager() : PddlManager() {}

protected:
  // Generate PDDL domain and problem from current state
  std::pair<omni_plan::pddl::Domain, omni_plan::pddl::Problem>
  get_pddl() const override {
    // Implement your state representation logic here
    // Return a pair of Domain and Problem objects
  }

  // Check if there are any goals to achieve
  bool has_goals() const override {
    // Query your state representation for pending goals
  }

  // Clear all current goals
  bool clear_goals() const override {
    // Clear goals from your state representation
  }

  // Check if a predicate exists in the current state
  bool predicate_exists(const omni_plan::pddl::Predicate &predicate) const override {
    // Query your state representation for the predicate
  }

  // Check if a predicate is part of the goal conditions
  bool predicate_is_goal(const omni_plan::pddl::Predicate &predicate) const override {
    // Check if the predicate is in the goals
  }

  // Apply a single effect to the current state
  void apply_effect(const omni_plan::pddl::Effect &exp) override {
    // Update your state representation with the effect
    // May add or delete predicates depending on the effect type
  }
};
```

### Creating New Planners

To create a new planner, inherit from the `omni_plan::Planner` base class and implement the required virtual methods:

```cpp
#include "omni_plan/planner.hpp"

class MyPlanner : public omni_plan::Planner {
public:
  MyPlanner() : Planner() {}

  // Generate plan from PDDL
  virtual pddl::Plan generate_plan(const pddl::Domain &domain,
                                   const pddl::Problem &problem) const override {
    // Implement your planning algorithm here
    // Return the plan object
  }

protected:
  // Generate plan from PDDL files
  std::string generate_plan(const std::string domain_path,
                            const std::string problem_path) const override {
    // Implement your planning algorithm here using PDDL files
    // Return the plan as a string in PDDL format
  }

  // Check if the plan output indicates a valid solution
  bool has_solution(const std::string &plan_str) const override {
    // Analyze the planner output to determine if a solution was found
  }

  // Optional: Parse action lines from the plan output
  std::pair<std::string, std::vector<std::string>>
  parse_action_line(std::string line) const override {
    // Extract action name and parameters from a line of planner output
  }

  // Optional: Extract lines containing actions from the complete plan output
  std::vector<std::string>
  get_lines_with_actions(const std::string &plan_str) const override {
    // Filter and return only the lines that represent actions
  }

  // Optional: Parse the start time from an action line
  float parse_start_time(const std::string &line) const override {
    // Extract the start time (in seconds) from a line of planner output
  }
};
```

### Creating New MRTA Task Allocators

The `omni_plan_mrta` package adds multi-robot task allocation on top of the standard planning pipeline. An `MrtaPlanner` decomposes the PDDL problem into per-team sub-problems, solves them in parallel with the configured sub-planner, and merges the resulting plans by sorting all actions by start time.

#### Notation

| Symbol                   | Meaning                                              |
| ------------------------ | ---------------------------------------------------- |
| $N$                      | Number of robots                                     |
| $M$                      | Number of goals                                      |
| $i \in \{0,\ldots,N-1\}$ | Robot index                                          |
| $j \in \{0,\ldots,M-1\}$ | Goal index                                           |
| $S_0$                    | Set of initial-state ground predicates               |
| $g_j$                    | Goal predicate with argument list $\text{args}(g_j)$ |

#### Spatial proximity: co-occurrence graph and BFS distance

All proximity-aware allocators share a common spatial model built from $S_0$ without requiring any geometric map.

**Co-occurrence adjacency.** Two objects $u, v$ are adjacent if they appear together as arguments in any single fact $f \in S_0$:

$$u \sim v \iff \exists\, f \in S_0 \text{ s.t. } u \in \text{args}(f) \wedge v \in \text{args}(f)$$

The adjacency list is deduplicated so each edge appears once regardless of how many facts contain the same pair.

**Robot-position arguments.** Let $\mathcal{R}$ be the set of robot names. The set of currently-occupied locations is

$$P = \{ a \mid \exists\, f \in S_0,\; \exists\, r \in \mathcal{R} : r \in \text{args}(f) \wedge a \in \text{args}(f) \wedge a \notin \mathcal{R} \}$$

**Sum-BFS distance.** A single BFS from robot $i$ in the co-occurrence graph records the hop-count $d(i, t)$ to every node $t$. The distance from robot $i$ to goal $g_j$ is the sum over the _unoccupied_ target arguments:

$$T_{ij} = \text{args}(g_j) \setminus (\{r_i\} \cup P) \qquad \text{(fallback to } \text{args}(g_j)\setminus\{r_i\} \text{ if } T_{ij} = \emptyset\text{)}$$

$$
D(i,j) = \sum_{t \in T_{ij}} d(i,t)
\qquad \bigl(d(i,t) = \tfrac{\mathrm{INT\_MAX}}{2} \text{ if } t \text{ unreachable}\bigr)
$$

Excluding occupied positions prevents a robot that happens to share one argument of a multi-argument goal from gaining a trivial 1-hop advantage over a robot genuinely closer to the goal's primary unoccupied locations.

---

#### 1. Round-Robin Allocator

The simplest allocator. It makes no use of the initial state or action schemas.

$$\text{robot}(j) = j \bmod N$$

Goal $j$ is assigned to robot $r_{j \bmod N}$. The result is a perfectly balanced assignment with $\lceil M/N \rceil$ or $\lfloor M/N \rfloor$ goals per robot. **Complexity:** $O(M)$.

---

#### 2. SSI Affinity Allocator

A Sequential Single-Item auction with 1-hop co-occurrence affinity bidding (Gerkey & Matarić, 2004).

**Affinity score.** For robot $i$ and goal $g_j$, the affinity counts initial-state facts that simultaneously mention robot $r_i$ and at least one non-robot argument of $g_j$:

$$\text{aff}(i,j) = \bigl|\{f \in S_0 \mid r_i \in \text{args}(f) \wedge \text{args}(g_j) \cap \text{args}(f) \neq \emptyset\}\bigr|$$

**Bid.** The bid accounts for the affinity minus a load-balancing penalty:

$$\text{score}(i,j) = \text{aff}(i,j) - \text{load}(i)$$

At each auction round the $(i^\ast, j^\ast)$ pair with the highest score wins and goal $j^\ast$ is removed from the pool. **Complexity:** $O(N \times M^2)$.

> **Limitation.** Only direct (1-hop) co-occurrence is captured. Indirect spatial relationships (robot → location → object → goal) are invisible to this allocator. Prefer the Greedy Auction Allocator when indirect proximity matters.

---

#### 3. Greedy Auction Allocator

SSI auction with full BFS-distance bidding in the co-occurrence graph.

**Bid.** Let $D(i,j)$ be the sum-BFS distance defined above, and $\mathrm{load\_coeff} = D_{\max} + 1$ where $D_{\max}$ is the largest finite distance across all robot–goal pairs:

$$\mathrm{score}(i,j) = -D(i,j) - \mathrm{load\_coeff} \cdot \mathrm{load}(i)$$

The load coefficient is chosen so that load balancing only resolves ties _within_ a distance tier — a robot one hop closer always beats a more lightly loaded but more distant robot. At each auction round the $(i^\ast, j^\ast)$ pair with the highest score wins. **Complexity:** $O(N \times M^2)$.

---

#### 4. CBBA Allocator

Consensus-Based Bundle Algorithm (Choi, Brunet & How, 2009) with delete-relaxed heuristic bidding and a post-convergence load-balancing pass.

##### Delete-relaxed heuristic costs

For each robot $i$, the allocator grounds all PDDL action templates that involve $r_i$ and runs delete-relaxed Bellman-Ford (h_add or h_max) against the initial facts filtered to exclude other robots:

$$S_0^{(i)} = \{ f \in S_0 \mid \forall r \in \mathcal{R} \setminus \{r_i\}: r \notin \text{args}(f) \}$$

**h_add** — additive relaxation: the cost of an action is $1 + \sum_{\text{pre}} h(p)$.

**h_max** — max relaxation: the cost of an action is $1 + \max_{\text{pre}} h(p)$.

The resulting cost map gives $h(i, j)$: the relaxed cost for robot $i$ to achieve goal $g_j$ ($\infty$ if unreachable).

##### Bid matrix

Let $D_{\max}^\text{BFS}$ be the largest finite sum-BFS distance. Define:

$$\mathrm{dist\_scale} = D_{\max}^{\mathrm{BFS}} + 1$$

This ensures h-cost differences always dominate BFS differences in the bid value. For reachable goals the bid is:

$$c(i,j) = -\bigl(h(i,j) \cdot \mathrm{dist\_scale} + D_{\mathrm{capped}}(i,j)\bigr)$$

where $D_{\mathrm{capped}}(i,j) = \min\bigl(D(i,j),\, \mathrm{dist\_scale}\bigr)$. For goals with $h(i,j) = \infty$:

$$c(i,j) = c_{\mathrm{unreach}} = -\bigl(\mathrm{bid\_max} + \alpha \cdot M + 1\bigr)$$

where $$\mathrm{bid\_max} = h_{\max} \cdot \mathrm{dist\_scale} + \mathrm{dist\_scale} \quad \text{and}$$ and $h_{\max}$ is the largest finite h-cost across all robot–goal pairs.

The load-balancing penalty $\alpha$ is chosen so that adding a task to the bundle can never outbid the best single-task assignment:

$$\alpha = \left\lfloor \frac{\mathrm{bid\_max}}{M+1} \right\rfloor + 1$$

##### CBBA main loop

Let $y_i[j]$ be the highest bid robot $i$ has seen for goal $j$, $z_i[j]$ the corresponding winner, and $b_i$ the ordered bundle of goals robot $i$ currently holds.

**Bundle phase** — each robot greedily extends its bundle:

$$\text{bid}(i, j, k) = c(i,j) - \alpha \cdot k \qquad (k = |b_i|)$$

Robot $i$ adds goal $j^\ast = \arg\max_j \{\text{bid}(i,j,k) \mid j \notin b_i,\; \text{bid}(i,j,k) > y_i[j]\}$ and updates $y_i[j^\ast] \leftarrow \text{bid}(i,j^\ast,k)$, $z_i[j^\ast] \leftarrow i$.

**Consensus phase** — global winner for goal $j$:

$$i^\ast = \arg\max_i\, y_i[j] \qquad \text{(current holder wins ties)}$$

Every robot that holds $j$ but is not $i^\ast$ removes $j$ from its bundle and updates its local tables to $y_i[j] \leftarrow y_{i^\ast}[j]$, $z_i[j] \leftarrow i^\ast$.

The loop terminates when no bundle changes occur in a full round (convergence), or after $3NM+1$ iterations (safety bound).

##### Post-convergence rebalancing

After CBBA converges, a load-rebalancing pass drives every robot's goal count to within 1 of the optimal $\lfloor M/N \rfloor$ or $\lceil M/N \rceil$.

Each iteration identifies the receiver (robot with minimum load) and the donor (most-loaded robot with $\text{load}[\text{donor}] > \text{load}[\text{recv}] + 1$). From the donor's goals the one transferred is:

$$j^\ast = \arg\min_{j \in b_\text{donor},\, c(\text{recv},j) > c_\text{unreach}}\; \bigl(c(\text{donor},j) - c(\text{recv},j)\bigr)$$

The steal is only attempted when the receiver has a finite h-cost for the goal. The loop repeats until $\max_i \text{load}(i) - \min_i \text{load}(i) \le 1$, or no reachable goal can be transferred.

**ROS parameter:** `allocator.use_h_max` (bool, default `false`).

---

#### 5. Coalition Formation Allocator

A three-phase PDDL-aware allocator that identifies goals requiring multi-robot cooperation, forms the smallest feasible coalition for each, and then assigns remaining single-robot goals via a greedy BFS auction.

##### Phase 1 — Goal classification

For each robot $i$ the allocator computes $h_\text{add}(i, j)$ using the grounded actions for $r_i$ and the filtered initial facts $S_0^{(i)}$. A goal is classified as **single-robot (SR)** if at least one robot can achieve it alone:

$$\text{SR}(j) \iff \exists\, i : h_\text{add}(i,j) < \infty$$

Otherwise it is classified as **multi-robot (MR)**: no single robot's action pool can reach $g_j$.

##### Phase 2 — Coalition formation (MR goals)

For each MR goal $g_j$, the allocator enumerates all $K$-subsets of available robots for $K = 2, 3, \ldots, K_{\max}$ and checks whether the _pooled_ action set of the coalition can achieve $g_j$ via h_add on the coalition-filtered facts:

$$S_0^{(C)} = \{ f \in S_0 \mid \forall r \in \mathcal{R} \setminus C: r \notin \text{args}(f) \}$$

The smallest $K$ for which any feasible coalition exists is chosen. Among all feasible $K$-subsets the one with minimum combined BFS distance is selected:

$$C^\ast = \arg\min_{C : h_\text{add}(C,j) < \infty,\, |C|=K^\ast} \sum_{i \in C} D(i,j)$$

All robots in $C^\ast$ are committed and removed from the solo pool.

**ROS parameter:** `allocator.max_coalition_size` (int, default `3`).

##### Phase 3 — Greedy BFS auction (SR goals and MR fallbacks)

Remaining goals are assigned to still-available solo robots using the same SSI BFS-distance auction as the Greedy Auction Allocator, augmented with a capability bonus to prefer robots that can actually achieve the goal over those that cannot:

$$\mathrm{score}(i,j) = \underbrace{\mathbb{1}[h_{\mathrm{add}}(i,j) < \infty] \cdot B}_{\text{capability}} - D(i,j) - \mathrm{load\_coeff} \cdot \mathrm{load}(i)$$

where $B = D_{\max} + \mathrm{load\_coeff} \cdot M + 1$ is large enough to ensure any capable robot outbids any incapable robot regardless of BFS distance or load.

---

#### Summary table

| Plugin                        | Key formula                                                                  | Complexity                                     |
| ----------------------------- | ---------------------------------------------------------------------------- | ---------------------------------------------- |
| `RoundRobinAllocator`         | $\text{robot}(j) = j \bmod N$                                                | $O(M)$                                         |
| `SsiAffinityAllocator`        | $\text{score}(i,j) = \text{aff}(i,j) - \text{load}(i)$                       | $O(NM^2)$                                      |
| `GreedyAuctionAllocator`      | $\mathrm{score}(i,j) = -D(i,j) - \mathrm{load\_coeff}\cdot \mathrm{load}(i)$ | $O(NM^2)$                                      |
| `CbbaAllocator`               | $c(i,j) = -(h(i,j)\cdot\sigma + D_\text{cap}(i,j))$, bundle+consensus        | $O(NM \cdot \text{iter})$                      |
| `CoalitionFormationAllocator` | h_add classification + subset search + BFS auction                           | $O\!\left(\binom{N}{K_{\max}} \cdot NM\right)$ |

---

To implement a custom allocator, inherit from `omni_plan_mrta::TaskAllocator`:

```cpp
#include "omni_plan_mrta/task_allocator.hpp"

class MyAllocator : public omni_plan_mrta::TaskAllocator {
public:
  MyAllocator() : TaskAllocator() {}

  std::vector<omni_plan_mrta::TeamAllocation>
  allocate(const std::vector<std::string> &robots,
           const std::vector<omni_plan::pddl::Predicate> &goals,
           const omni_plan::pddl::Problem &problem,
           const std::unordered_map<std::string,
                                    std::shared_ptr<omni_plan::pddl::Action>>
               &actions) const override {
    // Assign each goal to one or more robots.
    // Return a vector of TeamAllocation, one entry per robot group.
    // Each TeamAllocation contains:
    //   robots      — list of robot names in the team (≥ 1)
    //   goal_indices — indices into `goals` assigned to this team
    std::vector<omni_plan_mrta::TeamAllocation> result;
    // ... your allocation logic ...
    return result;
  }
};
```

Register the allocator in an `allocator_plugins.xml` file:

```xml
<class_libraries>
  <library path="my_allocator">
    <class name="my_pkg/MyAllocator"
           type="my_pkg::MyAllocator"
           base_class_type="omni_plan_mrta::TaskAllocator" />
  </library>
</class_libraries>
```

#### CBBA parameters

| Parameter             | Default | Description                                                   |
| --------------------- | ------- | ------------------------------------------------------------- |
| `allocator.use_h_max` | `false` | Use the h_max deletion-relaxation heuristic instead of h_add. |

#### Coalition Formation parameters

| Parameter                      | Default | Description                                                |
| ------------------------------ | ------- | ---------------------------------------------------------- |
| `allocator.max_coalition_size` | `3`     | Maximum number of robots that may form a single coalition. |

Register your planner in a `plugins.xml` file and export it using `PLUGINLIB_EXPORT_CLASS`.

### Creating New Plan Validators

Plan validators verify that generated plans are correct. Inherit from `omni_plan::PlanValidator`:

```cpp
#include "omni_plan/plan_validator.hpp"

class MyValidator : public omni_plan::PlanValidator {
public:
  MyValidator() : PlanValidator() {}

  // Validate plan against domain, problem and plan
  bool validate_plan(const pddl::Domain &domain,
                     const pddl::Problem &problem,
                     const pddl::Plan &plan) const override {
    // Implement validation logic using your preferred validator
  }

protected:
  // Validate plan against domain, problem and plan files
  bool validate_plan(const std::string &domain_path,
                     const std::string &problem_path,
                     const std::string &plan_path) const override {
    // Implement validation logic using your preferred validator
  }
};
```

### Creating New Plan Dispatchers

A `PlanDispatcher` plugin controls how the actions of a plan are executed once the planning graph has been built. Two built-in strategies are provided (`SequentialPlanDispatcher` and `ParallelPlanDispatcher`). You can create your own by:

1. Inheriting from `omni_plan::PlanDispatcher` and implementing `dispatch_actions()`.
2. Registering it as a pluginlib plugin with base class `omni_plan::PlanDispatcher`.

```cpp
// my_dispatcher/include/my_dispatcher/my_plan_dispatcher.hpp
#include "omni_plan/plan_dispatcher.hpp"

namespace my_dispatcher {

class MyPlanDispatcher : public omni_plan::PlanDispatcher {
public:
  MyPlanDispatcher() : omni_plan::PlanDispatcher() {}

protected:
  omni_plan::pddl::ActionStatus dispatch_actions(
      const std::vector<omni_plan::pddl::GraphNode::Ptr> &all_nodes) override {

    for (const auto &node : all_nodes) {
      if (this->is_canceled()) {
        return omni_plan::pddl::ActionStatus::CANCELED;
      }

      this->set_node_status(node->node_num,
                            omni_plan_msgs::msg::PlanActionStatus::RUNNING);
      this->publish_exec_status(
          omni_plan_msgs::msg::PlanExecutionStatus::RUNNING);

      auto action = node->action.action;
      this->push_current_action(action);
      omni_plan::pddl::ActionStatus result =
          this->run_node_action(node, action);
      this->remove_current_action(action);

      if (result != omni_plan::pddl::ActionStatus::SUCCEEDED) {
        if (this->cancel_on_abort_) {
          this->cancel_plan();
        }
        return result;
      }
    }

    this->clear_current_actions();
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }
};

} // namespace my_dispatcher
```

#### Protected helpers available in `dispatch_actions()`

| Helper                                   | Description                                                                                                                                                                                                    |
| ---------------------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| `is_canceled()`                          | Returns `true` if `cancel_plan()` has been called.                                                                                                                                                             |
| `cancel_plan()`                          | Cancels all running actions and sets the cancellation flag.                                                                                                                                                    |
| `run_node_action(node, action)`          | Applies PDDL effects, runs the action, and rolls back on failure. Returns `ActionStatus`.                                                                                                                      |
| `push_current_action(action, use_cache)` | Registers an action as currently running (enables cancellation). Returns the action instance to use — if `use_cache=true` a cached copy is returned when the action is already in use (for parallel branches). |
| `remove_current_action(action)`          | Un-registers a completed action.                                                                                                                                                                               |
| `clear_current_actions()`                | Clears all tracked actions (call at the end of dispatch).                                                                                                                                                      |
| `set_node_status(node_num, status)`      | Updates the per-node execution status for publishing.                                                                                                                                                          |
| `publish_exec_status(overall)`           | Publishes the current status snapshot on `/omni_plan/plan_execution`.                                                                                                                                          |
| `acquire_cached_action(action)`          | Returns an idle clone from the action pool (or creates one).                                                                                                                                                   |
| `release_cached_action(action)`          | Returns a clone to the pool for future reuse.                                                                                                                                                                  |

#### Protected configuration flags

| Flag                   | Default | Description                                                                             |
| ---------------------- | ------- | --------------------------------------------------------------------------------------- |
| `cancel_on_abort_`     | `false` | When `true`, call `cancel_plan()` automatically whenever an action aborts.              |
| `cancel_on_new_goals_` | `false` | When `true`, monitor the PDDL manager for new goals and cancel execution if any appear. |

Both flags are exposed as ROS parameters: `plan_dispatcher.cancel_on_abort` and `plan_dispatcher.cancel_on_new_goals`.

The built-in `ParallelPlanDispatcher` also exposes `plan_dispatcher.execution_threads` (integer, defaults to `std::hardware_concurrency()`) to control the size of its thread pool.

### Creating New Actions

Actions define the executable behaviors in your planning domain. All actions inherit from `omni_plan::pddl::Action` and must implement the `run` and `cancel` methods. All action types must be registered in a `plugins.xml` file and exported using the appropriate `PLUGINLIB_EXPORT_CLASS` macro.

#### Regular Omni Plan Actions

For simple actions implemented directly in C++:

```cpp
#include "omni_plan/pddl/action.hpp"

class MyAction : public omni_plan::pddl::Action {
public:
  MyAction()
      : Action("my_action", {
          {"param1", "type1"},
          {"param2", "type2"}
      }) {
    // Add preconditions
    this->add_condition(omni_plan::pddl::START, "predicate_name",
                       {"param1", "param2"});

    // Add effects
    this->add_effect(omni_plan::pddl::END, "predicate_name",
                    {"param1"}, true);  // true for negated effect
  }

  omni_plan::pddl::ActionStatus run(const std::vector<std::string> &params) override {
    // Implement your action execution logic
    // Return SUCCEEDED, CANCELED, or ABORTED
    return omni_plan::pddl::ActionStatus::SUCCEEDED;
  }

  void cancel() override {
    // Handle action cancellation
  }
};
```

#### YASMIN Actions

For actions that use YASMIN state machines defined programmatically:

```cpp
#include "omni_plan_yasmin/yasmin_action.hpp"

class MyYasminAction : public omni_plan_yasmin::YasminAction {
public:
  MyYasminAction()
      : YasminAction("my_action", {
          {"param1", "type1"},
          {"param2", "type2"}
      }) {
    // Define PDDL conditions and effects as in regular actions

    // Build your YASMIN state machine
    this->add_state("STATE1", std::make_shared<MyState1>());
    this->add_state("STATE2", std::make_shared<MyState2>());
    this->add_transition("STATE1", "STATE2", "outcome1");
    // ... configure state machine
  }
};
```

#### YASMIN Factory Actions

For actions using YASMIN state machines defined in XML files:

```cpp
#include "omni_plan_yasmin/yasmin_factory_action.hpp"

class MyYasminFactoryAction : public omni_plan_yasmin::YasminFactoryAction {
public:
  MyYasminFactoryAction()
      : YasminFactoryAction("my_action",
                           {{"param1", "type1"}, {"param2", "type2"}},
                           "/path/to/state_machine.xml") {
    // Define PDDL conditions and effects
    // The state machine is loaded from the XML file
  }

protected:
  // Optional: populate the blackboard before the state machine runs
  yasmin::Blackboard::SharedPtr create_blackboard() override {
    auto bb = YasminFactoryAction::create_blackboard();
    // Add custom entries to the blackboard here
    return bb;
  }
};
```

#### Behavior Tree Actions

For actions implemented as Behavior Trees:

```cpp
#include "omni_plan_bt/bt_action.hpp"

class MyBtAction : public omni_plan_bt::BtAction {
public:
  MyBtAction()
      : BtAction("my_action",
                {{"param1", "type1"}, {"param2", "type2"}},
                "/path/to/behavior_tree.xml") {
    // Define PDDL conditions and effects
    // The behavior tree is loaded from the XML file
  }

protected:
  // Optional: write action parameters into the BT blackboard before execution
  void load_data_in_blackboard() override {
    // Use this->set_input<T>("key", value) to populate blackboard entries
  }
};
```
