# cartographer.launch.py
# 맵핑 계층. TF/EKF는 직접 띄우지 않고 control의 bringup을 포함한다.
# (예전에는 여기서도 robot_state_publisher/ekf_node를 자체 실행해, 터미널 1과 함께
#  켜면 ekf_filter_node가 2개 떠서 odom->base_link TF가 충돌했다)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg = get_package_share_directory('wheelchair_robot_control')
    carto_pkg = get_package_share_directory('wheelchair_robot_cartographer')

    bringup_launch = os.path.join(control_pkg, 'launch', 'bringup.launch.py')
    carto_config_dir = os.path.join(carto_pkg, 'config')
    rviz_config = os.path.join(carto_pkg, 'rviz', 'wheelchair_robot_cartographer.rviz')
    occupancy_grid_launch = os.path.join(carto_pkg, 'launch', 'occupancy_grid.launch.py')

    return LaunchDescription([

        # [1] 로봇 기반 계층 (TF + EKF)
        IncludeLaunchDescription(PythonLaunchDescriptionSource(bringup_launch)),

        # [2] 지도 작성 (Cartographer)
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            parameters=[{'use_sim_time': False}],
            arguments=[
                '-configuration_directory', carto_config_dir,
                '-configuration_basename', 'wheelchair_robot.lua'
            ]
        ),

        # [3] 2D 맵 변환 (Occupancy Grid)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([occupancy_grid_launch]),
            launch_arguments={
                'use_sim_time': 'false',
                'resolution': '0.05',
                'publish_period_sec': '1.0'
            }.items()
        ),

        # [4] 시각화 (RViz2)
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': False}]
        )
    ])
