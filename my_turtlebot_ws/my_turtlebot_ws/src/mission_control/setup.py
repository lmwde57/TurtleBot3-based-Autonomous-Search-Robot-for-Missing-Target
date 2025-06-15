from setuptools import find_packages, setup

package_name = 'mission_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yoon',
    maintainer_email='yoon@todo.todo',
    description='A package to manage the mission of a TurtleBot3, including mapping, patrolling, and returning.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # --- 여기가 핵심 수정 부분입니다 ---
            # 'mission_manager' 라는 이름의 실행 파일을 만들고,
            # 이 파일은 mission_control 패키지 안의 mission_manager_node.py 파일에 있는
            # main 함수를 실행시킨다는 의미입니다.
            'mission_manager = mission_control.mission_manager_node:main',
        ],
    },
)
