"""
web_ui.launch.py
~~~~~~~~~~~~~~~~~
pixel-landscape-app のWebUIサーバーを起動するlaunchファイル。

使い方:
    ros2 launch web_ui_server web_ui.launch.py
    ros2 launch web_ui_server web_ui.launch.py port:=8443 use_https:=true
    ros2 launch web_ui_server web_ui.launch.py use_https:=false port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  return LaunchDescription([
      # ---- 起動引数 ----
      DeclareLaunchArgument(
          'host',
          default_value='0.0.0.0',
          description='バインドするIPアドレス（デフォルト: 全インターフェース）',
      ),
      DeclareLaunchArgument(
          'port',
          default_value='8443',
          description='リッスンするポート番号（443は root 権限が必要）',
      ),
      DeclareLaunchArgument(
          'use_https',
          default_value='true',
          description='HTTPS（TLS）を使用するか（true/false）',
      ),

      # ---- Webサーバーノード ----
      Node(
          package='web_ui_server',
          executable='server_node',
          name='web_ui_server',
          output='screen',
          parameters=[{
              'host': LaunchConfiguration('host'),
              'port': LaunchConfiguration('port'),
              'use_https': LaunchConfiguration('use_https'),
          }],
      ),
  ])
