from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    #num_grippers_launch_arg = DeclareLaunchArgument(
    #    'num_grippers',
    #    default_value='1'
    #)
    
    #ip_address_launch_arg = DeclareLaunchArgument(
    #    'ip_address',
    #    default_value='192.168.10.4'
    #)
    
    #port_launch_arg = DeclareLaunchArgument(
    #    'port',
    #    default_value='63352'
    #)
    
    #inversed_pos_launch_arg = DeclareLaunchArgument(
    #    'inversed_pos',
    #    default_value='True'
    #)
    
    #auto_calibrate_launch_arg = DeclareLaunchArgument(
    #    'auto_calibrate',
    #    default_value='True'
    #)

    griper_driver_node = Node(package='robotiq_sock_driver',
                             executable='gripper_driver',
                             name='gripper_driver',
                             parameters=[{"num_grippers": 1}, {"ip_address": "192.168.10.4"}, {"port": 63352}, {"inversed_pos": True}, {"auto_calibrate": True}],
                             output='screen',)

    return LaunchDescription([
        #num_grippers_launch_arg,
        #ip_address_launch_arg,
        #port_launch_arg,
        #inversed_pos_launch_arg,
        #auto_calibrate_launch_arg,
        griper_driver_node
    ])
