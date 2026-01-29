# Copyright (C) 2025 Miguel Ángel González Santamarta
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

import os
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from ament_index_python import get_package_share_directory


def generate_launch_description():

    def run_omni_plan(context: LaunchContext, state_machine_file, config_file):

        state_machine_file = str(context.perform_substitution(state_machine_file))
        config_file = str(context.perform_substitution(config_file))

        return [
            Node(
                package="yasmin_factory",
                executable="yasmin_factory_node",
                output="both",
                name="omni_plan_node",
                parameters=[config_file, {"state_machine_file": state_machine_file}],
            )
        ]

    state_machine_file = LaunchConfiguration("state_machine_file")
    state_machine_file_cmd = DeclareLaunchArgument(
        "state_machine_file",
        default_value=os.path.join(
            get_package_share_directory("omni_plan"),
            "state_machines",
            "planning_sm.xml",
        ),
        description="State machine file to use",
    )

    config_file = LaunchConfiguration("config_file")
    config_file_cmd = DeclareLaunchArgument(
        "config_file",
        default_value=os.path.join(
            get_package_share_directory("omni_plan_bringup"),
            "params",
            "omni_plan.yaml",
        ),
        description="Config file to use",
    )

    run_knowledge_base = LaunchConfiguration("run_knowledge_base")
    run_knowledge_base_cmd = DeclareLaunchArgument(
        "run_knowledge_base",
        default_value="False",
        description="Whether to run the knowledge base",
    )

    run_yasmin_viewer = LaunchConfiguration("run_yasmin_viewer")
    run_yasmin_viewer_cmd = DeclareLaunchArgument(
        "run_yasmin_viewer",
        default_value="True",
        description="Whether to run the Yasmin viewer",
    )

    run_knowledge_graph_viewer = LaunchConfiguration("run_knowledge_graph_viewer")
    run_knowledge_graph_viewer_cmd = DeclareLaunchArgument(
        "run_knowledge_graph_viewer",
        default_value="False",
        description="Whether to run the knowledge graph viewer",
    )

    knowledge_base_node = Node(
        package="omni_plan_knowledge_base",
        executable="knowledge_base_node",
        name="knowledge_base_node",
        output="both",
        condition=IfCondition(PythonExpression([run_knowledge_base])),
    )

    yasmin_viewer_node = Node(
        package="yasmin_viewer",
        executable="yasmin_viewer_node",
        name="yasmin_viewer_node",
        output="both",
        condition=IfCondition(PythonExpression([run_yasmin_viewer])),
    )

    knowledge_graph_viewer_cmd = Node(
        package="knowledge_graph_viewer",
        executable="rqt_knowledge_graph",
        condition=IfCondition(PythonExpression([run_knowledge_graph_viewer])),
    )

    omni_plan_cmd = OpaqueFunction(
        function=run_omni_plan,
        args=[
            state_machine_file,
            config_file,
        ],
    )

    ld = LaunchDescription()
    ld.add_action(state_machine_file_cmd)
    ld.add_action(config_file_cmd)
    ld.add_action(run_knowledge_base_cmd)
    ld.add_action(run_yasmin_viewer_cmd)
    ld.add_action(run_knowledge_graph_viewer_cmd)
    ld.add_action(knowledge_base_node)
    ld.add_action(yasmin_viewer_node)
    ld.add_action(knowledge_graph_viewer_cmd)
    ld.add_action(omni_plan_cmd)
    return ld
