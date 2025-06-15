from setuptools import setup
import os
from glob import glob

package_name = 'auto_explore_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 런치 파일 설치 추가
        (os.path.join('share', package_name, 'launch'),
         glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='user',
    maintainer_email='user@example.com',
    description='TurtleBot3 Auto Exploration Controller',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'position_manager = auto_explore_controller.position_manager:main',
            'timer_controller = auto_explore_controller.timer_controller:main',
            'navigation_controller = auto_explore_controller.navigation_controller:main',
            'exploration_controller = auto_explore_controller.exploration_controller:main',
        ],
    },
)
