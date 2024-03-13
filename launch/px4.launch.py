import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    px4_config_path = os.path.join(
        get_package_share_directory('px4_isaac_ros_bridge'),
        'config', 'px4_config.yaml'
    )
    px4_pluginlists_path = os.path.join(
        get_package_share_directory('px4_isaac_ros_bridge'),
        'config', 'px4_pluginlists.yaml'
    )

    return LaunchDescription([

        Node(
            package='mavros',
            executable='mavros_node',
            parameters=[
                px4_pluginlists_path,
                px4_config_path,
            {    
                'fcu_url': '/dev/ttyACM0:921600',
                'gcs_url': 'udp://@10.42.0.1',
                'tgt_system': 1,
                'tgt_component': 1,
                'fcu_protocol': "v2.0",
                'respawn_mavros': "false",
                'namespace': "mavros",
            }],
        ),
    ])