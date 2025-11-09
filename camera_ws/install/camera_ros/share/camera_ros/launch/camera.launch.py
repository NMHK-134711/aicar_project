from ament_index_python.resources import has_resource
from launch.launch_description import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description() -> LaunchDescription:
    """
    Generate a launch description with for the camera node and a visualiser.

    Returns
    -------
        LaunchDescription: the launch description

    """
    
    camera_node = ComposableNode(
        package='camera_ros',
        plugin='camera::CameraNode',
        name='camera_node',
        parameters=[{
            "camera": "/base/axi/pcie@120000/rp1/i2c@88000/ov5647@36",
            "plugin": "rpi/pisp", 
            
            "width": 640,
            "height": 480,
            "framerate": 30.0,
            "format": "BGR888",

            "orientation": 180,
          
            "image_topic": "/raw_image",
            "camera_info_topic": "/camera_info",
            "camera_calibration_file": "",
        }],
        extra_arguments=[{'use_intra_process_comms': True}],
    )

    composable_nodes = [
        camera_node,
    ]

    container = ComposableNodeContainer(
        name='camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=composable_nodes,
        output='screen',
    )

    return LaunchDescription([
        container,
    ])