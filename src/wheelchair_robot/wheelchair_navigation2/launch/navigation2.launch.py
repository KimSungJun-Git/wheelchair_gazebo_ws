#navigation2.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, SetEnvironmentVariable
from launch_ros.actions import Node, SetRemap
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # ===== [RViz2 튕김 방지 및 클린 스타트] 이전 RViz 강제 종료 및 캐시/히스토리 초기화 =====
    os.system('killall -9 rviz2 2>/dev/null || true')
    os.system('rm -rf ~/.rviz2/ ~/.config/ros.org/rviz2.conf 2>/dev/null || true')

    # ===== 경로 설정 =====
    pkg_dir = get_package_share_directory('wheelchair_navigation2')
    nav2_launch_dir = os.path.join(
        get_package_share_directory('nav2_bringup'), 'launch')

    rviz_config = os.path.join(pkg_dir, 'rviz', 'navigation2.rviz')
    default_map = os.path.join(pkg_dir, 'map', 'world.yaml')
    # BT XML 경로를 이 워크스페이스의 실제 설치 위치로 주입 (yaml에 절대경로를 박아두면
    # 다른 워크스페이스/실기에 배포했을 때 파일을 못 찾으므로 여기서 매번 재계산한다)
    bt_xml_path = os.path.join(pkg_dir, 'behavior_trees', 'navigate_w_replanning_and_recovery.xml')
    default_param = RewrittenYaml(
        source_file=os.path.join(pkg_dir, 'param', 'navigation2.yaml'),
        root_key='',
        param_rewrites={
            'default_bt_xml_filename': bt_xml_path,
            'default_nav_to_pose_bt_xml': bt_xml_path,
            'default_nav_through_poses_bt_xml': bt_xml_path,
        },
        convert_types=True,
    )
    keepout_mask_yaml = os.path.join(pkg_dir, 'map', 'keepout_mask.yaml')
    speed_mask_yaml = os.path.join(pkg_dir, 'map', 'speed_mask.yaml') 

    # ===== 실행 인자 =====
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    map_path = LaunchConfiguration('map', default=default_map)
    param_path = LaunchConfiguration('params_file', default=default_param)

    return LaunchDescription([
        # ===== 환경 변수 설정 (RViz2 Wayland 2D Pose 충돌 방지) =====
        SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb'),

        # ===== 인자 선언 =====
        DeclareLaunchArgument('map', default_value=default_map, description='Full path to map yaml file to load'),
        DeclareLaunchArgument('params_file', default_value=default_param, description='Full path to the ROS2 parameters file to use'),
        DeclareLaunchArgument('use_sim_time', default_value='false', description='Use simulation (Gazebo) clock if true'),

        # ===== 1. 접근 금지 구역 (Keepout) 필터 =====
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_filter_info_server',
            output='screen',
            parameters=[param_path],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_mask_server',
            output='screen',
            parameters=[{
                'yaml_filename': keepout_mask_yaml,
                'topic_name': 'keepout_filter_mask', 
                'frame_id': 'map',
                'use_sim_time': use_sim_time,
            }],
        ),

        # ===== 2. 서행 구역 (Speed Limit) 필터 =====
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_filter_info_server',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'type': 1,                                    # 1: SpeedLimit
                'filter_info_topic': '/speed_filter_info',
                'mask_topic': '/speed_filter_mask',
                'base': 0.0,
                'multiplier': 1.0,                            # 마스크 픽셀값 비율(%) 반영
            }],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_mask_server',
            output='screen',
            parameters=[{
                'yaml_filename': speed_mask_yaml,
                'topic_name': 'speed_filter_mask', 
                'frame_id': 'map',
                'use_sim_time': use_sim_time,
            }],
        ),

        # ===== 코스트맵 필터 라이프사이클 매니저 =====
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': [
                    'keepout_filter_info_server',
                    'keepout_mask_server',
                    'speed_filter_info_server',
                    'speed_mask_server',
                ],
            }],
        ),
        # ===== Nav2 Bringup (그룹 리매핑 적용) =====
        GroupAction(
            actions=[
                # 이 그룹 안에 있는 모든 노드의 /cmd_vel 출력을 /cmd_vel_nav로 강제 변경
                SetRemap(src='/cmd_vel', dst='/cmd_vel_nav'),
                SetRemap(src='/cmd_vel_smoothed', dst='/cmd_vel_nav_smoothed'),
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(nav2_launch_dir, 'bringup_launch.py')),
                    launch_arguments={
                        'map': map_path,
                        'use_sim_time': use_sim_time,
                        'params_file': param_path,
                        'use_composition': 'False',
                    }.items(),
                )
            ]
        ),
        
        # ===== RViz2 Node =====
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            additional_env={'QT_QPA_PLATFORM': 'xcb'},
            output='screen'
        ),
    ])