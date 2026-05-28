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
                'fcu_url': '/dev/ttyACM1:57600',
                'gcs_url': 'udp://@brayan-Victus.local:14550',
            },
            os.path.join(pkg_share, 'config', 'px4_pluginlists.yaml'),
            os.path.join(pkg_share, 'config', 'px4_config.yaml')
        ]
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
        odomtotf_node,
        rc_control_node
    ])