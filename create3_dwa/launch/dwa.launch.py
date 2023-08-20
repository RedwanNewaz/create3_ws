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
conf_sensor_fusion = {
    "ctrv_mtx": [0.9725,  0.0, 0.0, 0.9725], #[0.725,  0.0, 0.0, 0.725],
    "lidar_mtx": [0.0275,0.0, 0.0, 0.0275], 
    "radar_mtx": [0.9725,  0.000,  0.00, 
                  0.000,  0.9725,  0.00, 
                  0.000,  0.000,  0.025]
}

def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    namespace = LaunchConfiguration('namespace')
    sensor_fusion_dir = current_pkg_dir = get_package_share_directory('sensor_fusion')
    robotName = 'ac32'
    if len(sys.argv) > 4:
        robotName = sys.argv[4].split("=")[-1]

    conf_sensor_fusion['tag_id'] = 32 if robotName == 'ac32' else 7

    
    multicam_tag_state_estimator = ComposableNode(
        package='multicam_tag_state_estimator', plugin='airlab::multicam_tag_state_estimator',
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    sensor_fusion_node = ComposableNode(
        namespace=namespace,
        package='sensor_fusion', plugin='airlab::apriltag_fusion',
        parameters=[conf_sensor_fusion],
        extra_arguments=[{'use_intra_process_comms': True}],
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
        composable_node_descriptions=[multicam_tag_state_estimator, sensor_fusion_node, create3_dwa],
        output='screen'
    )

    ld.add_action(container)






    return ld