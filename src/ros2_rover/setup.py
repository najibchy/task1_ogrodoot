from setuptools import setup

package_name = 'ros2_rover'

setup(
    name=package_name,
    version='0.0.1',
    packages=['ros2_rover'],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/ros2_rover']),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='You',
    maintainer_email='you@example.com',
    description='ROS2 Rover nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mission_manager = ros2_rover.nodes.mission_manager:main',
            'turtlesim_controller = ros2_rover.nodes.turtlesim_controller:main',
            'gnss_simulator = ros2_rover.nodes.gnss_simulator:main',
            'camera_node = ros2_rover.nodes.camera_node:main',
            'vision_detector = ros2_rover.nodes.vision_detector:main',
            'science_node = ros2_rover.nodes.science_node:main',
            'delivery_node = ros2_rover.nodes.delivery_node:main',
            'equipment_node = ros2_rover.nodes.equipment_node:main',
            'autonav_node = ros2_rover.nodes.autonav_node:main',
            'manual_teleop = ros2_rover.nodes.manual_teleop:main',
        ],
    },
)
