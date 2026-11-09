# safety.launch.py
# 휠체어 로봇 안전 제어 계층 런치 파일
# - dynamic_scan_filter: 라이다 기반 동적 장애물 감지 및 CPA 위협 선별
# - safety_stop_node: 긴급 정지 / 감속 및 속도 명령 안전 중재 (/cmd_vel_nav, /cmd_vel_teleop -> /cmd_vel)
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    control_pkg_dir = get_package_share_directory('wheelchair_control')
    safety_config = os.path.join(control_pkg_dir, 'config', 'safety.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'
        ),

        # [1] Dynamic Scan Filter (CPA 기반 동적 위협 선별)
        Node(
            package='wheelchair_control',
            executable='dynamic_scan_filter',
            name='dynamic_scan_filter',
            output='screen',
            parameters=[safety_config, {'use_sim_time': use_sim_time}],
        ),

        # [2] Safety Stop Node (안전 정지 및 감속 제어)
        Node(
            package='wheelchair_control',
            executable='safety_stop_node',
            name='safety_stop_node',
            output='screen',
            parameters=[safety_config, {'use_sim_time': use_sim_time}],
        ),
    ])
