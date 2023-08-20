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

def getNode(namespace, tagId, start):
    create3_dummy_state_publisher = ComposableNode(
        namespace=namespace,
        package='create3_dummy_state_publisher', plugin='airlab::create3_dummy_state_publisher',
        parameters=[{"tag_id":tagId, "start_loc":start}],
        extra_arguments=[{'use_intra_process_comms': True}]
    )

    create3_dwa = ComposableNode(
        namespace=namespace,
        package='create3_dwa', plugin='airlab::create3_dwa',
        parameters=[{"tag_id":tagId}],
        extra_arguments=[{'use_intra_process_comms': True}],
    )



    container = ComposableNodeContainer(
        name='tag_container_%s' % namespace,
        namespace='apriltag_%s' % namespace,
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[create3_dummy_state_publisher, create3_dwa],
        output='screen'
    )
    return container


def generate_launch_description():
    ld = LaunchDescription()

    container1 = getNode('ac31', 7, [2.0, 2.0])
    container2 = getNode('ac32', 32, [-2.0, -2.0])
    ld.add_action(container1)
    ld.add_action(container2)

    return ld