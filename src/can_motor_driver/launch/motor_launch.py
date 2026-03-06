from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  return LaunchDescription([
      # VESC 4輪オムニドライブノード
      # /cmd_vel (geometry_msgs/Twist) を購読し、CAN 経由で各 VESC に RPM 指令を送信
      Node(
          package='can_motor_driver',
          executable='motor_node',
          name='vesc_omni_node',
          output='screen'
      )
  ])
