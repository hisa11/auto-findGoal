"""
robot.launch.py
~~~~~~~~~~~~~~~
全サブシステムを一括起動する統合ランチファイル。

起動コマンド:
  ros2 launch web_ui_server robot.launch.py

オプション:
  target_color:=RED|BLUE|GREEN  追従するゴール色 (デフォルト: BLUE)
  enable_camera:=true|false     RealSense + YOLO を起動するか (デフォルト: true)
  web_port:=8443                WebUI ポート番号 (デフォルト: 8443)
  use_https:=true|false         HTTPS を使うか (デフォルト: true)

起動するノード一覧:
  [常時]
    rosbridge_websocket   … WebSocket ブリッジ (port 9090, TLS)
    web_ui_server         … WebUI HTTPS サーバー (port 8443)
    main_node             … CAN 全制御 (DM motor / C610 / VESC オムニ / 砲塔)

  [enable_camera=true 時]
    realsense2_camera     … RealSense D435i (640x480x30, 露出固定)
    advanced_yolo_detector… YOLO ゴール検出 → /yolo/goal_info パブリッシュ
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
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
          'use_dm_motor',
          default_value='true',
          description='DM モータ (砲塔 pitch/yaw) を使用するか。false にすると起動シーケンスをスキップし C610 のみ即起動',
      ),
      DeclareLaunchArgument(
          'target_color',
          default_value='BLUE',
          description='YOLOが追従するゴール色 (RED / BLUE / GREEN)',
      ),
      DeclareLaunchArgument(
          'enable_camera',
          default_value='true',
          description='RealSense + YOLO ノードを起動するか',
      ),
      DeclareLaunchArgument(
          'web_port',
          default_value='8443',
          description='WebUI サーバーのポート番号',
      ),
      DeclareLaunchArgument(
          'use_https',
          default_value='true',
          description='HTTPS (TLS) を使用するか',
      ),
  ]

  _pkg_share = get_package_share_directory('web_ui_server')

  # ---- 1. ROSBridge WebSocket サーバー -------------------------
  rosbridge_node = Node(
      package='rosbridge_server',
      executable='rosbridge_websocket',
      name='rosbridge_websocket',
      parameters=[{
          'port': 9090,
          'address': '0.0.0.0',
          'ssl': True,
          'certfile': os.path.join(_pkg_share, 'certs', '192.168.2.8+2.pem'),
          'keyfile': os.path.join(_pkg_share, 'certs', '192.168.2.8+2-key.pem'),
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

  # ---- 3. メインコントローラーノード ---------------------------
  #   DM motor / C610 (ロボマス) / VESC オムニ / 砲塔制御
  #   CAN の送受信はすべてこのノードが担う
  main_node = Node(
      package='can_motor_driver',
      executable='main_node',
      name='main_controller_node',
      output='screen',
      parameters=[{
          'use_dm_motor': LaunchConfiguration('use_dm_motor'),
      }],
  )

  # ---- 4. RealSense カメラ ------------------------------------
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

  # カメラ完全起動後 3 秒待ってから露出を強制セット
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

  # ---- 5. YOLO ゴール検出ノード --------------------------------
  #   /yolo/goal_info (geometry_msgs/Point) をパブリッシュ
  #   /yolo/target_color (std_msgs/String) をサブスクライブ
  yolo_node = Node(
      package='can_motor_driver',
      executable='advanced_yolo_detector.py',
      name='advanced_yolo_detector',
      parameters=[{'target_color': LaunchConfiguration('target_color')}],
      output='screen',
      condition=IfCondition(LaunchConfiguration('enable_camera')),
  )

  # ---- LaunchDescription 組み立て -----------------------------
  return LaunchDescription(args + [
      rosbridge_node,   # WebSocket ブリッジ
      webui_node,       # WebUI サーバー
      main_node,        # CAN 全制御 (常時起動)
      realsense_node,   # カメラ (enable_camera=true のみ)
      camera_params_action,  # 露出強制セット (3秒後)
      yolo_node,        # YOLO 検出 (enable_camera=true のみ)
  ])
