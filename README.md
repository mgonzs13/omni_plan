# EasyPlan

## Installation

```shell
cd ~/ros2_ws/src
git clone https://github.com/mgonzs13/easy_plan
cd ~/ros2_ws
vcs import src < src/easy_plan/dependencies.rosinstall
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

```shell
colcon test --executor sequential --packages-select easy_plan_knowledge_graph easy_plan_popf easy_plan_vhpop easy_plan_smtp
colcon test-result --verbose
```

## Demo

```shell
ros2 launch easy_plan_demos easy_plan_demos.launch.py
```

```shell
ros2 run yasmin_viewer yasmin_viewer_node
```

```shell
ros2 run knowledge_graph_viewer rqt_knowledge_graph
```

```shell
ros2 run easy_plan_demos knowledge_graph_demo
```
