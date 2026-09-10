# bringup.launch.py
# 로봇 기반 계층 — TF(URDF) + EKF. 맵핑이든 네비게이션이든 항상 이것 위에 얹는다.
# 이 조합이 여러 launch에 복붙돼 ekf_filter_node가 이중 실행되던 것을 여기로 모았다.
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('wheelchair_robot_description')
    control_pkg = get_package_share_directory('wheelchair_robot_control')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'wheelchair_robot.urdf')
    ekf_config = os.path.join(control_pkg, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    return LaunchDescription([
        # [1] 로봇 뼈대(TF) 발행
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
        ),

        # [2] 위치 추정(EKF) — 센서 융합. odom -> base_link TF를 낸다.
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_config, {'use_sim_time': False}],
        ),
    ])
