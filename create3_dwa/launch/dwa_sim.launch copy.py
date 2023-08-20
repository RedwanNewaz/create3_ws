import argparse
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

ARGUMENTS = [
    DeclareLaunchArgument('namespace', default_value='',
                          description='Robot namespace'),
]

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    namespace = LaunchConfiguration('namespace')

    create3_dummy_state_publisher = ComposableNode(
        namespace=namespace,
        package='create3_dummy_state_publisher', plugin='airlab::create3_dummy_state_publisher',
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    create3_dwa = ComposableNode(
        namespace=namespace,
        package='create3_dwa', plugin='airlab::create3_dwa',
        extra_arguments=[{'use_intra_process_comms': True}],
    )



    container = ComposableNodeContainer(
        name='tag_container',
        namespace='apriltag',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[create3_dummy_state_publisher, create3_dwa],
        output='screen'
    )

    ld.add_action(container)






    return ld