from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            output='screen',
            arguments=[
                '--ros-args',
                '-p', 'width:=640',
                '-p', 'height:=480',
                '-p', 'format:=MJPEG'
            ]
        )
    ])
