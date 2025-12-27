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
from launch import LaunchDescription
from ament_index_python import get_package_share_directory


def generate_launch_description():

    config_file = os.path.join(
        get_package_share_directory("easy_plan_demos"),
        "params",
        "smtp_kg_demo.yaml",
    )

    sm_file = os.path.join(
        get_package_share_directory("easy_plan"),
        "state_machines",
        "planning_sm.xml",
    )

    ld = LaunchDescription()
    ld.add_action(
        Node(
            package="yasmin_factory",
            executable="yasmin_factory_node",
            output="both",
            parameters=[config_file, {"state_machine_file": sm_file}],
        )
    )
    return ld
