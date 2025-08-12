# Copyright 2025 SCHUNK SE & Co. KG
#
# This program is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the Free
# Software Foundation, either version 3 of the License, or (at your option)
# any later version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
# FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for
# more details.
#
# You should have received a copy of the GNU General Public License along with
# this program. If not, see <https://www.gnu.org/licenses/>.
# --------------------------------------------------------------------------------

from launch import LaunchDescription  # type: ignore [attr-defined]
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

host = DeclareLaunchArgument(
    "host",
    default_value="192.168.0.100",
    description="The sensor's TCP/IP host address",
)
port = DeclareLaunchArgument(
    "port",
    default_value="82",
    description="The sensor's TCP/IP port",
)
streaming_port = DeclareLaunchArgument(
    "streaming_port",
    default_value="54843",
    description="The sensor's UDP/IP streaming port",
)

args = [host, port, streaming_port]


def generate_launch_description():
    return LaunchDescription(
        args
        + [
            Node(
                package="schunk_fts_driver",
                namespace="schunk",
                executable="driver.py",
                name="driver",
                parameters=[
                    {"host": LaunchConfiguration("host")},
                    {"port": LaunchConfiguration("port")},
                    {"streaming_port": LaunchConfiguration("streaming_port")},
                ],
                respawn=False,
                output="both",
            )
        ]
    )
