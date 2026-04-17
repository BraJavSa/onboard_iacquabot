import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('onboard_iacquabot')

    mavros_node = Node(
        package='mavros',
        executable='mavros_node',
        output='screen',
        parameters=[
            {
                'fcu_url': '/dev/ttyACM0:57600',
                'gcs_url': 'udp://@brayan-Victus.local:14550',
            },
            os.path.join(pkg_share, 'config', 'px4_pluginlists.yaml'),
            os.path.join(pkg_share, 'config', 'px4_config.yaml')
        ]
    )

    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '-b', '115200'],
        output='screen'
    )

    odomtotf_node = Node(
        package='onboard_iacquabot',
        executable='odomtotf.py',
        output='screen'
    )

    rc_control_node = Node(
        package='onboard_iacquabot',
        executable='rc_control_px4.py',
        output='screen'
    )

    return LaunchDescription([
        mavros_node,
        micro_ros_agent,
        odomtotf_node,
        rc_control_node
    ])
