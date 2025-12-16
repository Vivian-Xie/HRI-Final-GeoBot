from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_control_architecture_pkg',
            executable='robot_control_architecture_node',
            name='robot_control_architecture_node',
            output='screen',
            parameters=['config/params.yaml'],
            # 如需重映射，例如激光是 /scan_raw：
            # remappings=[('cmd_vel', '/cmd_vel'), ('scan', '/scan_raw')],
        )
    ])
