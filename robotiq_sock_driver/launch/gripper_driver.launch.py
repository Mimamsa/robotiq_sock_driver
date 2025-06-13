from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    declared_arguments = []

    num_grippers = LaunchConfiguration('num_grippers', default=1)
    ip_address = LaunchConfiguration('ip_address', default='')
    port = LaunchConfiguration('port', default=65532)
    inversed_pos = LaunchConfiguration('inversed_pos', default=True)
    auto_calibrate = LaunchConfiguration('auto_calibrate', default=True)

    griper_driver_node = Node(package='robotiq_sock_driver',
                             executable='gripper_driver',
                             name='gripper_driver',
                             parameters=[
                                {"num_grippers": num_grippers},
                                {"ip_address": ip_address},
                                {"port": port},
                                {"inversed_pos": inversed_pos},
                                {"auto_calibrate": auto_calibrate}
                             ],
                             output='screen',)

    return LaunchDescription([
        griper_driver_node
    ])
