# gazebo_bridge.launch.py
# 실물 R카 대신 Gazebo(turtlebot3 waffle_pi)를 붙일 때 쓰는 연결 계층.
# 기존 노드/설정은 손대지 않고, 이미 떠 있는 gazebo(빈 월드)에 로봇을 스폰하고
# 실기에만 있는 초음파 센서를 흉내내서 기존 launch들이 그대로 동작하게 한다.
import os
import tempfile
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    stock_sdf = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'models', 'turtlebot3_waffle_pi', 'model.sdf')

    with open(stock_sdf, 'r') as f:
        sdf = f.read()

    # ekf_filter_node(odom0: /odom, imu0: /imu/data)가 odom->base_footprint TF를
    # 직접 낸다. gazebo 플러그인도 같은 TF를 내면 두 발행자가 충돌하므로 끈다.
    sdf = sdf.replace(
        '<publish_odom_tf>true</publish_odom_tf>',
        '<publish_odom_tf>false</publish_odom_tf>')
    # ekf.yaml이 imu0로 /imu/data를 구독하므로 기본 /imu 대신 그쪽으로 낸다.
    sdf = sdf.replace('~/out:=imu', '~/out:=imu/data')

    patched_sdf = os.path.join(tempfile.gettempdir(), 'wheelchair_gazebo_waffle_pi.sdf')
    with open(patched_sdf, 'w') as f:
        f.write(sdf)

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'waffle_pi', '-file', patched_sdf,
                   '-x', '0.0', '-y', '0.0', '-z', '0.01'],
        output='screen',
        condition=IfCondition(LaunchConfiguration('spawn', default='false')),
    )

    def fake_ultrasonic(topic):
        # 장애물 없음(먼 거리)으로 계속 채워서 safety_stop_node가
        # '센서 끊김'으로 판단해 영구 비상정지 거는 걸 막는다.
        msg = ('{radiation_type: 0, field_of_view: 0.3, '
               'min_range: 0.02, max_range: 4.0, range: 4.0}')
        return ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '10', topic,
                 'sensor_msgs/msg/Range', msg],
            output='screen',
        )

    rsp_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch', 'robot_state_publisher.launch.py'
            )
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument('spawn', default_value='false', description='Gazebo에 로봇 자동 스폰 여부'),
        spawn,
        rsp_launch,
        fake_ultrasonic('/ultrasonic/range'),
        fake_ultrasonic('/ultrasonic/left'),
        fake_ultrasonic('/ultrasonic/right'),
    ])
