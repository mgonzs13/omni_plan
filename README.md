# EasyPlan

<p align="center">
  <img src="./docs/logo.png" width="50%" />
</p>

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
colcon test --executor sequential --packages-select easy_plan easy_plan_knowledge_base easy_plan_knowledge_graph easy_plan_val easy_plan_popf easy_plan_vhpop easy_plan_smtp easy_plan_yasmin easy_plan_bt easy_plan_tests
colcon test-result --verbose
```

## Demo

```shell
ros2 launch easy_plan_demos popf_kg_demo.launch.py
```

```shell
ros2 run easy_plan_demos knowledge_graph_demo
```

<!-- https://github.com/user-attachments/assets/3cb4d0e4-9118-40c6-9a4c-834608307166 -->

https://github.com/user-attachments/assets/ded56c3f-1d74-451e-b317-48be318b2f2b
