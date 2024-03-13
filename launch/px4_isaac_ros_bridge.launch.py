import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import TextSubstitution
from launch.substitutions import Command
from launch_ros.actions import Node


def generate_launch_description():

    # Paths to packages 
    package_path = get_package_share_directory('px4_isaac_ros_bridge')
    urdf_xacro_path = os.path.join(package_path, 'urdf', 'urdf', 's500.urdf.xacro')
    rviz_config_path = os.path.join(package_path,'config','rviz_config.rviz')

    # run the px4 bridge node 
    px4_bridge_node = Node(
        package='px4_isaac_ros_bridge',
        executable='px4_isaac_ros_bridge',
        output='screen'
    )

    # include the mavros px4.launch file
    px4_launch_include = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
        os.path.join(
            package_path,
            'launch', 'px4.launch.py'
            )
        )
    )

    # load the robot model from urdf/xacro
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name ='s500_state_publisher',
        output='screen',
        parameters=[
            {'robot_description': Command([ 'xacro ', urdf_xacro_path])},
            {'package_path': package_path}
        ]
    )

    # joint_state_publisher for robot joints
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen'
    )
    
    # rviz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        px4_bridge_node,
        px4_launch_include,
        robot_state_publisher_node,
        joint_state_publisher_node,
        rviz_node,
    ])