"""
robot.launch.py
~~~~~~~~~~~~~~~
全サブシステムを一括起動する統合ランチファイル。

  ros2 launch web_ui_server robot.launch.py

オプション例:
  ros2 launch web_ui_server robot.launch.py target_color:=RED
  ros2 launch web_ui_server robot.launch.py enable_camera:=false
  ros2 launch web_ui_server robot.launch.py web_port:=8443 use_https:=true
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    TimerAction,
    ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  # ---- 引数宣言 ------------------------------------------------
  args = [
      DeclareLaunchArgument(
          'target_color',
          default_value='BLUE',
          description='YOLOが追従するゴール色 (RED / BLUE / GREEN)',
      ),
      DeclareLaunchArgument(
          'enable_camera',
          default_value='true',
          description='RealSense + YOLOノードを起動するか',
      ),
      DeclareLaunchArgument(
          'enable_drive',
          default_value='true',
          description='joy_node + motor_node を起動するか',
      ),
      DeclareLaunchArgument(
          'web_port',
          default_value='8443',
          description='WebUIサーバーのポート番号',
      ),
      DeclareLaunchArgument(
          'use_https',
          default_value='true',
          description='HTTPS を使用するか',
      ),
  ]

  # ---- 1. ROSBridge WebSocket サーバー -------------------------
  rosbridge_node = Node(
      package='rosbridge_server',
      executable='rosbridge_websocket',
      name='rosbridge_websocket',
      parameters=[{
          'port': 9090,
          'address': '0.0.0.0',
      }],
      output='screen',
  )

  # ---- 2. WebUI HTTPS サーバー ---------------------------------
  webui_node = Node(
      package='web_ui_server',
      executable='server_node',
      name='web_ui_server',
      output='screen',
      parameters=[{
          'port': LaunchConfiguration('web_port'),
          'use_https': LaunchConfiguration('use_https'),
      }],
  )

  # ---- 3. RealSense カメラ + YOLO ゴール検出 ------------------
  realsense_dir = get_package_share_directory('realsense2_camera')

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
          'rgb_camera.gain': '0',
      }.items(),
      condition=IfCondition(LaunchConfiguration('enable_camera')),
  )

  # カメラ起動後 3 秒待ってから露出を強制セット
  camera_params_action = TimerAction(
      period=3.0,
      actions=[
          ExecuteProcess(
              cmd=['ros2', 'param', 'set', '/camera/camera',
                   'rgb_camera.enable_auto_exposure', 'false'],
              output='screen',
          ),
          ExecuteProcess(
              cmd=['ros2', 'param', 'set', '/camera/camera',
                   'rgb_camera.exposure', '30'],
              output='screen',
          ),
          ExecuteProcess(
              cmd=['ros2', 'param', 'set', '/camera/camera',
                   'rgb_camera.gain', '0'],
              output='screen',
          ),
      ],
      condition=IfCondition(LaunchConfiguration('enable_camera')),
  )

  yolo_node = Node(
      package='can_motor_driver',
      executable='advanced_yolo_detector.py',
      name='advanced_yolo_detector',
      parameters=[{'target_color': LaunchConfiguration('target_color')}],
      output='screen',
      condition=IfCondition(LaunchConfiguration('enable_camera')),
  )

  # ---- 4. 砲塔制御ノード (main_node: YOLO → CAN) --------------
  #   カメラが有効な場合のみ起動
  turret_node = Node(
      package='can_motor_driver',
      executable='main_node',
      name='main_controller_node',
      output='screen',
      condition=IfCondition(LaunchConfiguration('enable_camera')),
  )

  # ---- 5. 走行系: joy_node + motor_node ------------------------
  joy_node = Node(
      package='joy',
      executable='joy_node',
      name='joy_node',
      parameters=[{
          'deadzone': 0.05,
          'autorepeat_rate': 20.0,
      }],
      condition=IfCondition(LaunchConfiguration('enable_drive')),
  )

  motor_node = Node(
      package='can_motor_driver',
      executable='motor_node',
      name='motor_node',
      output='screen',
      condition=IfCondition(LaunchConfiguration('enable_drive')),
  )

  # ---- LaunchDescription 組み立て -----------------------------
  return LaunchDescription(args + [
      rosbridge_node,
      webui_node,
      realsense_node,
      camera_params_action,
      yolo_node,
      turret_node,
      joy_node,
      motor_node,
  ])
