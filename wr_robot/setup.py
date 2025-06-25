from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'wr_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # ROS package index
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),

        # package.xml
        ('share/' + package_name, ['package.xml']),

        # launch 파일 (루트 launch/ 기준으로 glob 사용)
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),

        # pictures 폴더 전체 포함
        *[
            (os.path.join('share', package_name, 'pictures', os.path.relpath(root, 'pictures')),
            [os.path.join(root, f) for f in files])
            for root, _, files in os.walk('pictures')
        ],
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='we',
    maintainer_email='llaayy.kr@gmail.com',
    description='Z-sensitive drawing robot system using Doosan',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'visual = wr_robot.visual:main',
            'move_robot = wr_robot.move_robot:main',
            'move_path = wr_robot.move_path:main',
            'control_robot = wr_robot.control_robot:main',
            'ui = wr_robot.ui:main',
            'df_vis = wr_robot.df_vis:main'
             
        ],
    },
)
