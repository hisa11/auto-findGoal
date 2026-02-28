import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    realsense_dir = get_package_share_directory('realsense2_camera')

    # 1. RealSenseカメラの起動設定
    # 引数名を rs_launch.py の仕様（.profile）に合わせて修正
    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(realsense_dir, 'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_depth': 'true',
            'enable_color': 'true',
            'align_depth.enable': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
            'rgb_camera.enable_auto_exposure': 'false',
            'rgb_camera.exposure': '30',
            'rgb_camera.gain': '0'
        }.items(),
    )

    # 2. 露出・ゲインを「強制的に」後出しで設定するアクション
    # カメラが完全に立ち上がるのを3秒待ってから、手動コマンドと同じ内容を実行します
    set_params_action = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/camera/camera',
                     'rgb_camera.enable_auto_exposure', 'false'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set', '/camera/camera',
                     'rgb_camera.exposure', '30'],
                output='screen'
            ),
            ExecuteProcess(
                cmd=['ros2', 'param', 'set',
                     '/camera/camera', 'rgb_camera.gain', '0'],
                output='screen'
            ),
        ]
    )

    # 3. YOLOノードの起動もここに入れておくと便利です
    yolo_node = Node(
        package='can_motor_driver',
        executable='advanced_yolo_detector.py',
        name='advanced_yolo_detector',
        parameters=[{'target_color': 'BLUE'}],  # 追従したい色
        output='screen'
    )

    return LaunchDescription([
        realsense_node,
        set_params_action,
        yolo_node
    ])
