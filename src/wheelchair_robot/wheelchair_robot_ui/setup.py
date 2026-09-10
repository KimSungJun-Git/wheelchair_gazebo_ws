from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wheelchair_robot_ui'

setup(
    name=package_name,
    version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ament 인덱스
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # package.xml
        ('share/' + package_name, ['package.xml']),
        # launch 파일들
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 웹 정적 파일 — colcon build 후
        # install/wheelchair_robot_ui/share/wheelchair_robot_ui/web/ 에 복사됨
        # index.html(랜딩) + Wheelchair_SLAM_UI.html(탑승자 UI)
        (os.path.join('share', package_name, 'web'), glob('web/*.html')),
        (os.path.join('share', package_name, 'web', 'js'),
            glob('web/js/*.js') + glob('web/js/*.jsx')),
        # admin/Dashboard.html — 관제 대시보드
        (os.path.join('share', package_name, 'web', 'admin'), glob('web/admin/*.html')),
        (os.path.join('share', package_name, 'web', 'admin', 'src'), glob('web/admin/src/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kim',
    maintainer_email='ksjun100848@naver.com',
    description='휠체어 로봇 웹 UI — 탑승자 UI + 관제 대시보드 + 대시보드 API 서버',
    license='Apache 2.0',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            # 관제 대시보드 백엔드 (FastAPI, 기본 8090)
            'admin_server = wheelchair_robot_ui.admin_server:main',
        ],
    },
)
