# full_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    nodes = []

    # turtlesim node
    nodes.append(Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim'
    ))

    # mission_manager
    nodes.append(Node(
        package='ros2_rover',
        executable='mission_manager',
        name='mission_manager',
        parameters=[{'mission_mode': 'manual'}]
    ))

    # controllers & simulators
    nodes.append(Node(package='ros2_rover', executable='turtlesim_controller', name='turtlesim_controller'))
    nodes.append(Node(package='ros2_rover', executable='gnss_simulator', name='gnss_simulator'))
    nodes.append(Node(package='ros2_rover', executable='camera_node', name='camera_node', parameters=[{'use_webcam': False}]))
    nodes.append(Node(package='ros2_rover', executable='vision_detector', name='vision_detector'))
    nodes.append(Node(package='ros2_rover', executable='science_node', name='science_node'))
    nodes.append(Node(package='ros2_rover', executable='delivery_node', name='delivery_node'))
    nodes.append(Node(package='ros2_rover', executable='equipment_node', name='equipment_node'))
    nodes.append(Node(package='ros2_rover', executable='autonav_node', name='autonav_node'))
    # manual teleop optional
    nodes.append(Node(package='ros2_rover', executable='manual_teleop', name='manual_teleop'))

    return LaunchDescription(nodes)
