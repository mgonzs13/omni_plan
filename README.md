# EasyPlan

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

---

EasyPlan is a ROS 2 framework for automated task planning and execution. It integrates multiple classical planners with flexible execution mechanisms including direct action implementation, state machines and behavior trees. The framework supports both knowledge base and knowledge graph approaches for state management, enabling planning solutions for robotic applications. Finally, EasyPlan can be extended through the creation of new plugins to integrate new planners and new knowledge sources.

## Table of Contents

- [Key Features](#key-features)
- [Installation](#installation)
- [Demos](#demos)
  - [Knowledge Graph with POPF Planner](#knowledge-graph-with-popf-planner)
  - [Knowledge Base with POPF Planner](#knowledge-base-with-popf-planner)
  - [Other Demos](#other-demos)
  - [Adding Knowledge](#adding-knowledge)
- [API Development](#api-development)
  - [Creating New Planners](#creating-new-planners)
  - [Creating New Plan Validators](#creating-new-plan-validators)
  - [Creating New Actions](#creating-new-actions)
    - [Regular Easy Plan Actions](#regular-easy-plan-actions)
    - [YASMIN Actions](#yasmin-actions)
    - [YASMIN Factory Actions](#yasmin-factory-actions)
    - [Behavior Tree Actions](#behavior-tree-actions)

## Key Features

- **Plugin Architecture**: Extensible design using ROS 2 pluginlib for easy customization.
- **Multiple PDDL Planners**: Support for POPF, SMTP and VHPOP planners and VAL plan validator.
- **Flexible Execution**: Execute plans using direct actions, YASMIN state machines or Behavior Trees.
- **Knowledge Management**: Choose between knowledge base or knowledge graph approaches or integrate your own implementation.
- **ROS 2 Native**: Built on ROS 2 with proper message interfaces.

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/easy_plan
cd ~/ros2_ws
vcs import src < src/easy_plan/dependencies.rosinstall
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

To run the tests:

```shell
colcon test --executor sequential --packages-select easy_plan easy_plan_knowledge_base easy_plan_knowledge_graph easy_plan_val easy_plan_popf easy_plan_vhpop easy_plan_smtp easy_plan_yasmin easy_plan_bt easy_plan_tests
colcon test-result --verbose
```

## Demos

https://github.com/user-attachments/assets/ded56c3f-1d74-451e-b317-48be318b2f2b

The framework includes several demo packages showcasing different planning and execution approaches:

### Knowledge Graph with POPF Planner

```shell
ros2 launch easy_plan_demos popf_kg_demo.launch.py
```

### Knowledge Base with POPF Planner

```shell
ros2 launch easy_plan_demos popf_kb_demo.launch.py
```

### Other Demos

- `smtp_kb_demo.launch.py` / `smtp_kg_demo.launch.py`: SMTP planner demos
- `vhpop_kb_demo.launch.py` / `vhpop_kg_demo.launch.py`: VHPOP planner demos

### Adding Knowledge

```shell
ros2 run easy_plan_demos knowledge_graph_demo
```

## API Development

EasyPlan uses a plugin-based architecture that allows developers to extend the framework by creating new planners, plan validators, and actions. All plugins are loaded using ROS2's pluginlib system.

### Creating New Planners

To create a new planner, inherit from the `easy_plan::Planner` base class and implement the required virtual methods:

```cpp
#include "easy_plan/planner.hpp"

class MyPlanner : public easy_plan::Planner {
public:
  MyPlanner() : Planner() {}

protected:
  // Generate plan from PDDL files
  std::string generate_plan(const std::string domain_path,
                           const std::string problem_path) const override {
    // Implement your planning algorithm here
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
};
```

Register your planner in a `plugins.xml` file and export it using `PLUGINLIB_EXPORT_CLASS`.

### Creating New Plan Validators

Plan validators verify that generated plans are correct. Inherit from `easy_plan::PlanValidator`:

```cpp
#include "easy_plan/plan_validator.hpp"

class MyValidator : public easy_plan::PlanValidator {
public:
  MyValidator() : PlanValidator() {}

protected:
  // Validate plan against domain and problem
  bool validate_plan(const std::string &domain_path,
                    const std::string &problem_path,
                    const std::string &plan_path) const override {
    // Implement validation logic using your preferred validator
  }

  // Convert Plan object to PDDL string format
  std::string parse_pddl(const easy_plan::pddl::Plan &plan) const override {
    // Convert the internal Plan representation to PDDL format
  }
};
```

### Creating New Actions

Actions define the executable behaviors in your planning domain. All actions inherit from `easy_plan::pddl::Action` and must implement the `run` and `cancel` methods. All action types must be registered in a `plugins.xml` file and exported using the appropriate `PLUGINLIB_EXPORT_CLASS` macro.

#### Regular Easy Plan Actions

For simple actions implemented directly in C++:

```cpp
#include "easy_plan/pddl/action.hpp"

class MyAction : public easy_plan::pddl::Action {
public:
  MyAction()
      : Action("my_action", {
          {"param1", "type1"},
          {"param2", "type2"}
      }) {
    // Add preconditions
    this->add_condition(easy_plan::pddl::START, "predicate_name",
                       {"param1", "param2"});

    // Add effects
    this->add_effect(easy_plan::pddl::END, "predicate_name",
                    {"param1"}, true);  // true for negated effect
  }

  easy_plan::pddl::ActionStatus run(const std::vector<std::string> &params) override {
    // Implement your action execution logic
    // Return SUCCEED, CANCEL, or ABORT
    return easy_plan::pddl::ActionStatus::SUCCEED;
  }

  void cancel() override {
    // Handle action cancellation
  }
};
```

#### YASMIN Actions

For actions that use YASMIN state machines defined programmatically:

```cpp
#include "easy_plan_yasmin/yasmin_action.hpp"

class MyYasminAction : public easy_plan_yasmin::YasminAction {
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
#include "easy_plan_yasmin/yasmin_factory_action.hpp"

class MyYasminFactoryAction : public easy_plan_yasmin::YasminFactoryAction {
public:
  MyYasminFactoryAction()
      : YasminFactoryAction("my_action",
                           {{"param1", "type1"}, {"param2", "type2"}},
                           "/path/to/state_machine.xml") {
    // Define PDDL conditions and effects
    // The state machine is loaded from the XML file
  }
};
```

#### Behavior Tree Actions

For actions implemented as Behavior Trees:

```cpp
#include "easy_plan_bt/bt_action.hpp"

class MyBtAction : public easy_plan_bt::BtAction {
public:
  MyBtAction()
      : BtAction("my_action",
                {{"param1", "type1"}, {"param2", "type2"}},
                "/path/to/behavior_tree.xml") {
    // Define PDDL conditions and effects
    // The behavior tree is loaded from the XML file
  }
};
```
