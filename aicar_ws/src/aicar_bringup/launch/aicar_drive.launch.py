import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    # --- 1. C++ 카메라 노드 실행 ---
    # 'camera_ros' 패키지의 'install' 디렉토리에서 런치 파일 경로를 찾습니다.
    camera_ros_pkg_dir = get_package_share_directory('camera_ros')
    camera_launch_file = os.path.join(
        camera_ros_pkg_dir, 
        'launch', 
        'camera.launch.py' # 우리가 수정한 그 런치 파일
    )
    
    # C++ 카메라 런치 파일을 포함시킵니다.
    camera_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(camera_launch_file)
    )

    # --- 2. 파이썬 차선 감지 노드 실행 ---
    lane_detector_node = Node(
        package='aicar_vision',           # 패키지 이름
        executable='lane_detector_node',  # setup.py의 'console_scripts' 이름
        name='lane_detector_node'         # 노드 이름
    )

    # --- 3. 파이썬 제어 노드 실행 ---
    controller_node = Node(
        package='aicar_controller',       # 패키지 이름
        executable='pure_pursuit_node', # setup.py의 'console_scripts' 이름
        name='pure_pursuit_node'        # 노드 이름
    )

    motor_driver_node = Node(
        package='aicar_driver',
        executable='motor_controller_node',
        name='motor_controller_node',
        output='screen' # 모터 로그를 터미널에 바로 출력
    )

    # --- 4. 모든 노드를 런치 파일에 등록 ---
    return LaunchDescription([
        camera_node_launch,
        lane_detector_node,
        controller_node,
        motor_driver_node,
    ])