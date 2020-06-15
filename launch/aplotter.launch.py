
import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('aplotter_ros2').find('aplotter_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'aplotter.urdf')

    rsp = launch_ros.actions.Node(package='robot_state_publisher',
                                  executable='robot_state_publisher',
                                  output='both',
                                  arguments=[urdf_file])
    asp = launch_ros.actions.Node(package='aplotter_ros2',
                                  executable='aplotter',
                                  output='both',
                                  parameters=[
                                      {"control_loop_frequency": 50}, # In hz
                                      {"left_arm_length": 361.0}, # mm
                                      {"right_arm_length": 379.62}, # mm
                                      {"right_arm_extension_length": 187.0}, # mm
                                      {"left_arm_extension_length": 35.0}, # mm
                                      {"homed_joint_offset": 68.0}, # mm
                                      {"encoder_counts_per_mm": 4030817} # counts/mm, 16384*(89/20)*pi*17.598
                                  ])

    return launch.LaunchDescription([rsp, asp])