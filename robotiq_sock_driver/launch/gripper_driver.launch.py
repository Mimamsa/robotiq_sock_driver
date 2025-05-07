from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    griper_driver_node = Node(package='robotiq_sock_driver',
                             executable='gripper_driver',
                             name='gripper_driver',
                             parameters=[{"num_grippers": 1}, {"ip_address": "192.168.10.4"}, {"port": "63352"}],
                             output='screen',)

    return LaunchDescription([griper_driver_node])
