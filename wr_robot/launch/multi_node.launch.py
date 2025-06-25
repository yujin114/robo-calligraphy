from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='dr_writer',
            executable='move_robot',
            name='move_robot',  
            namespace='dsr01',           
            output='screen'
        ),
        Node(
            package='dr_writer',
            executable='visual',
            name='visual',
            output='screen'
        ),
    ])
