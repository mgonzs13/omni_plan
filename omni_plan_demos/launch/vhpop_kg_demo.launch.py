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
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory("omni_plan_demos"),
        "params",
        "vhpop_kg_demo.yaml",
    )

    ld = LaunchDescription()
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("omni_plan_bringup"),
                    "launch",
                    "omni_plan.launch.py",
                ),
            ),
            launch_arguments={
                "config_file": config_file,
                "run_knowledge_base": "True",
                "run_knowledge_graph_viewer": "True",
            }.items(),
        )
    )
    return ld
