"""


Reference:
    https://github.com/PickNikRobotics/robotiq_85_gripper/blob/0f8410468ffd7b45a3345f411bacd855920c612e/robotiq_85_driver/

"""
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException

from robotiq_sock_driver.robotiq_gripper import RobotiqGripper
from robotiq_sock_driver_msgs.msg import GripperCmd, GripperStat
from sensor_msgs.msg import JointState
import time


class GripperDriver(Node):
    def __init__(self):
        super().__init__('gripper_driver')

        self.declare_parameter('num_grippers', 1)
        self.declare_parameter('ip_address', '')
        self.declare_parameter('port', 63352)
        self.declare_parameter('inversed_pos', True)
        self.declare_parameter('auto_calibrate', True)

        self._num_grippers = self.get_parameter('num_grippers').get_parameter_value().integer_value
        self._ip_address = self.get_parameter('ip_address').get_parameter_value().string_value
        self._port = self.get_parameter('port').get_parameter_value().integer_value
        self.inversed_pos = self.get_parameter('inversed_pos').get_parameter_value().bool_value
        self.auto_calibrate = self.get_parameter('auto_calibrate').get_parameter_value().bool_value

        self.get_logger().info("Parameters Num grippers: %i, IP address: %s, Port: %s " % (self._num_grippers, self._ip_address, self._port))

        # Connect to gripper
        self._gripper = RobotiqGripper(hostname=self._ip_address, port=self._port, inversed_pos=self.inversed_pos)
        try:
            self._gripper.connect()
        except Exception as e:
            self.get_logger().error("Unable to open commport to %s: " % self._ip_address)
            self.get_logger().error(e)
            return
        self.get_logger().info('Gripper connected.')

        # Create publishers & subscribers
        if (self._num_grippers == 1):
            self.create_subscription(GripperCmd, "/gripper/cmd", self._update_gripper_cmd, 10)
            self._gripper_pub = self.create_publisher(GripperStat, '/gripper/stat', 10)
            self._gripper_joint_state_pub = self.create_publisher(JointState, '/gripper/joint_states', 10)
        else:
            self.get_logger().error("Number of grippers not supported (needs to be 1)")
            return
        self.get_logger().info('Publisher & subscriber created.')

        # Create internal variables
        self._prev_js_pos = 0.0  # previous joint position
        self._prev_js_time = self.get_time()  # timestamp for previous joint position
        self._driver_state = 0
        self._driver_ready = False

        # Test activating gripper
        try:
            self._gripper.activate(auto_calibrate=False)
        except Exception as e:
            self.get_logger().error("Failed to activate gripper....ABORTING")
            self.get_logger().error(e)
            return
        self.get_logger().info('Gripper activated.')
        
        print('self.auto_calibrate: ', self.auto_calibrate)
        
        # Auto-calibrate
        if self.auto_calibrate:
            self._gripper.auto_calibrate()
            self.get_logger().info('Auto-calibration finished.')
            open_pos = self._gripper.get_open_position()
            closed_pos = self._gripper.get_closed_position()
            self.get_logger().info('Reachable open/closed position: [{}, {}]'.format(open_pos, closed_pos))


        self._last_time = self.get_time()
        
        # 5 Hz timer 
        self.timer = self.create_timer(0.2, self._timer_callback)
        self.get_logger().info('Timer created.')

    def __del__(self):
        self._gripper.disconnect()

    def get_time(self):
        time_msg = self.get_clock().now().to_msg()
        return float(time_msg.sec) + (float(time_msg.nanosec) * 1e-9)

    def _clamp_cmd(self, cmd, lower, upper):
        if (cmd < lower):
            return lower
        elif (cmd > upper):
            return upper
        else:
            return cmd

    def _update_gripper_cmd(self, cmd):
        """Callback for subscriber """
        if (True == cmd.emergency_release):
            self._gripper.activate_emergency_release(open_gripper=cmd.emergency_release_dir)
            return
        else:
            self._gripper.deactivate_emergency_release()

        if (True == cmd.stop):
            self._gripper.stop()
        else:
            pos = self._clamp_cmd(cmd.position, 0, 60)  # stroke: 0 mm - 50 mm
            vel = self._clamp_cmd(cmd.speed, 20, 150)  # speed: 20 mm/s - 150 mm/s
            force = self._clamp_cmd(cmd.force, 20, 185)  # grip force: 20 N - 185 N
            #self._gripper.move(position=pos, speed=vel, force=force)  # for real-time control
            self._gripper.move_and_wait_for_pos(position=pos, speed=vel, force=force)

    def _update_gripper_stat(self):
        """ """
        stat = GripperStat()
        stat.header.stamp = self.get_clock().now().to_msg()
        stat.is_ready = self._gripper.is_ready()
        stat.is_reset = self._gripper.is_reset()
        stat.is_moving = self._gripper.is_moving()
        stat.obj_detected = self._gripper.object_detected()
        stat.fault_status = self._gripper.get_fault_status()
        stat.position = self._gripper.get_current_position()
        stat.requested_position = self._gripper.get_position_requested()
        stat.current = 0.0  #self._gripper.get_current()
        return stat

    def _update_gripper_joint_state(self):
        js = JointState()
        # update header
        js.header.frame_id = ''
        js.header.stamp = self.get_clock().now().to_msg()
        # update name
        js.name = ['robotiq_hande_joint']
        # update position
        pos = self._gripper.get_current_position()
        js.position = [pos]
        # update velocity
        dt = self.get_time() - self._prev_js_time
        self._prev_js_time = self.get_time()
        js.velocity = [(pos-self._prev_js_pos)/dt]
        self._prev_js_pos = pos
        return js

    def _timer_callback(self):
        """Callback for the 2 publisher """
        dt = self.get_time() - self._last_time
        #print("cb:", dt)

        if (0 == self._driver_state):
            if (dt < 0.5):
                self._gripper.deactivate()
            else:
                self._driver_state = 1
        elif (1 == self._driver_state):
            grippers_activated = True
            self._gripper.activate()
            grippers_activated &= self._gripper.is_ready()
            if (grippers_activated):
                self._driver_state = 2
        elif (2 == self._driver_state):
            self._driver_ready = True

        # Update & publish gripper status & joints values
        stat = GripperStat()
        js = JointState()
        try:
            stat = self._update_gripper_stat()
            js = self._update_gripper_joint_state()
            self._gripper_pub.publish(stat)
            self._gripper_joint_state_pub.publish(js)
        except Exception as e:
            self.get_logger().error("Failed to contact gripper")
            self.get_logger().error(e)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = GripperDriver()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
