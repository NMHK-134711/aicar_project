import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    camera_ros_pkg_dir = get_package_share_directory('camera_ros')
    camera_launch_file = os.path.join(
        camera_ros_pkg_dir,
        'launch',
        'camera.launch.py'
    )
    
    camera_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    lane_detector_node = Node(
        package='aicar_vision',
        executable='lane_detector_node',
        name='lane_detector_node'
    )

    controller_node = Node(
        package='aicar_controller',
        executable='pid_controller_node',
        name='pid_controller_node'
    )

    motor_driver_node = Node(
        package='aicar_driver',
        executable='differential_drive_node',
        name='differential_drive_node',
        output='screen'
    )

    return LaunchDescription([
        camera_node_launch,
        lane_detector_node,
        controller_node,
        motor_driver_node,
    ])