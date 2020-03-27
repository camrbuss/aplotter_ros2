
import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('aplotter_ros2').find('aplotter_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'aplotter.urdf')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  node_executable='robot_state_publisher',
                                  output='both',
                                  arguments=[urdf_file])
    asp = launch_ros.actions.Node(package='aplotter_ros2',
                                  node_executable='aplotter_state_publisher',
                                  output='both')

    return launch.LaunchDescription([rsp, asp])