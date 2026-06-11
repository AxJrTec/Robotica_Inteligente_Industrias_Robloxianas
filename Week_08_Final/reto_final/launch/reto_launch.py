from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Nodo Compresor
        Node(
            package='reto_final',
            executable='compresor',
            name='compresor_node',
            output='screen'
        ),
        
        # 2. Nodo Odometría
        Node(
            package='reto_final',
            executable='odometry',
            name='odometry_node',
            output='screen'
        ),
        
        # 3. Nodo Controlador
        Node(
            package='reto_final',
            executable='controller',
            name='controller_node',
            output='screen'
        )
    ])