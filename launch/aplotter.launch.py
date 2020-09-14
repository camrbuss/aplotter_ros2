
import os

import launch
import launch_ros.actions
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('aplotter_ros2').find('aplotter_ros2')
    urdf_file = os.path.join(pkg_share, 'urdf', 'aplotter.urdf')

    joy = launch_ros.actions.Node(package='joy_linux',
                                  executable='joy_linux_node',
                                  output='both')
    roc = launch_ros.actions.Node(package='ros2_odrive_can',
                                  executable='odrive_can',
                                  output='both')                                  
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
                                      {"right_arm_pivot_length": 379.62}, # mm
                                      {"right_arm_full_length": 565.0}, # mm
                                      {"right_arm_offset_angle": 0.0923}, # rad
                                      {"homed_joint_offset": 95.0}, # mm
                                      {"mm_per_rev": 12.417} # rev/mm, (20/89)*pi*17.598 Measured approximately 12.8 mm per rev
                                  ])

    return launch.LaunchDescription([joy, roc, rsp, asp])