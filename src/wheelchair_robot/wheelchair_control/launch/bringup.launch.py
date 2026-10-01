# bringup.launch.py
# 로봇 기본 드라이버 및 상태 추정 계층: TF(URDF), IMU Relay, EKF 노드 관리
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node


def generate_launch_description():
    desc_pkg = get_package_share_directory('wheelchair_description')
    control_pkg = get_package_share_directory('wheelchair_control')

    urdf_file = os.path.join(desc_pkg, 'urdf', 'robot.urdf')
    ekf_config = os.path.join(control_pkg, 'config', 'ekf.yaml')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # [1] 로봇 뼈대(TF) 발행 노드
    rsp_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # [2] IMU frame_id 릴레이 노드 (stella_ahrs_node의 빈 frame_id를 'imu_link'로 보정하여 /imu/data_fixed 발행)
    imu_relay_node = Node(
        package='wheelchair_control',
        executable='imu_relay_node',
        name='imu_relay_node',
        output='screen',
        parameters=[{
            'input_topic': '/imu/data',
            'output_topic': '/imu/data_fixed',
            'target_frame_id': 'imu_link',
        }],
    )

    # [3] 위치 추정(EKF) — TF 및 IMU 릴레이 초기화 안정화 후 기동 (지연 0.5초)
    ekf_node = TimerAction(
        period=0.5,
        actions=[
            Node(
                package='robot_localization',
                executable='ekf_node',
                name='ekf_filter_node',
                output='screen',
                parameters=[ekf_config, {'use_sim_time': False}],
            )
        ]
    )

    return LaunchDescription([
        rsp_node,
        imu_relay_node,
        ekf_node,
    ])
