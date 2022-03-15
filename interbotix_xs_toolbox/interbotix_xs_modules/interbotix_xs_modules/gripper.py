import time
import rclpy
import threading

from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Empty

from interbotix_xs_msgs.srv import RobotInfo
from interbotix_xs_msgs.msg import JointSingleCommand
from interbotix_xs_modules.core import InterbotixRobotXSCore


class InterbotixGripperXS:
    """Standalone Module to control an Interbotix Gripper using PWM or
    Current control
    """

    def __init__(
        self,
        robot_model: str,
        gripper_name: str,
        robot_name: str = None,
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
        init_node: bool = True,
    ) -> None:
        """Definition of the Standalone Interbotix Gripper Module

        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'vx300s')
        :param gripper_name: name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param robot_name: defaults to value given to 'robot_model'; this can be
            customized to best suit the user's needs
        :param gripper_pressure: fraction from 0 - 1 where '0' means the gripper
            operates at 'gripper_pressure_lower_limit' and '1' means the gripper
            operates at 'gripper_pressure_upper_limit'
        :param gripper_pressure_lower_limit: lowest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 0; it should be high enough to
            open/close the gripper (~150 PWM or ~400 mA current)
        :param gripper_pressure_upper_limit: largest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 1; it should be low enough that
            the motor doesn't 'overload' when gripping an object for a few seconds (~350
            PWM or ~900 mA)
        :param init_node: set to True if the InterbotixRobotXSCore class should
            initialize the ROS node - this is the most Pythonic approach; to incorporate
            a robot into an existing ROS node though, set to False
        :details: note that this module doesn't really have any use case except in
            controlling just the gripper joint on an Interbotix Arm.
        """
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node)
        self.gripper = InterbotixGripperXSInterface(
            self.dxl,
            gripper_name,
            gripper_pressure,
            gripper_pressure_lower_limit,
            gripper_pressure_upper_limit,
        )

        rclpy.spin_once(self.dxl)
        self.dxl.initialize()
        rclpy.spin_once(self.dxl)
        self.gripper.initialize()
        # TODO: is this the best way to do this?
        self._execution_thread = threading.Thread(target=rclpy.spin, args=(self.dxl,))
        self._execution_thread.start()

    def shutdown(self):
        """Destroys the node and shuts down all threads and processes"""
        print("Destroying InterbotixGripperXS...")
        self.dxl.destroy_node()
        rclpy.shutdown()
        time.sleep(0.5)
        self._execution_thread.join(timeout=5.0)
        if self._execution_thread.is_alive():
            print("Taking a long time to destroy. Press Ctrl+C twice to exit.")
        print("Destroyed InterbotixGripperXS!")


class InterbotixGripperXSInterface:
    def __init__(
        self,
        core: InterbotixRobotXSCore,
        gripper_name: str,
        gripper_pressure: float = 0.5,
        gripper_pressure_lower_limit: int = 150,
        gripper_pressure_upper_limit: int = 350,
    ) -> None:
        """Definition of the Interbotix Gripper Module

        :param core: reference to the InterbotixRobotXSCore class containing the
            internal ROS plumbing that drives the Python API
        :param gripper_name: name of the gripper joint as defined in the 'motor_config'
            yaml file; typically, this is 'gripper'
        :param gripper_pressure: fraction from 0 - 1 where '0' means the gripper
            operates at 'gripper_pressure_lower_limit' and '1' means the gripper
            operates at 'gripper_pressure_upper_limit'
        :param gripper_pressure_lower_limit: lowest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 0; it should be high enough to
            open/close the gripper (~150 PWM or ~400 mA current)
        :param gripper_pressure_upper_limit: largest 'effort' that should be applied to
            the gripper if gripper_pressure is set to 1; it should be low enough that
            the motor doesn't 'overload' when gripping an object for a few seconds (~350
            PWM or ~900 mA)
        """
        self.core = core
        self.gripper_name = gripper_name
        self.gripper_pressure = gripper_pressure
        self.future_gripper_info = self.core.srv_get_info.call_async(
            RobotInfo.Request(cmd_type="single", name=gripper_name)
        )
        self.gripper_moving: bool = False
        self.gripper_command = JointSingleCommand(name="gripper")
        self.gripper_pressure_lower_limit = gripper_pressure_lower_limit
        self.gripper_pressure_upper_limit = gripper_pressure_upper_limit

        # value = lower + pressure * range
        self.gripper_value = gripper_pressure_lower_limit + (
            gripper_pressure
            * (gripper_pressure_upper_limit - gripper_pressure_lower_limit)
        )

        self.tmr_gripper_state = self.core.create_timer(
            timer_period_sec=0.02, callback=self.gripper_state
        )

    def initialize(self) -> None:
        """Initialize the InterbotixGripperXSInterface object"""
        while rclpy.ok() and not self.future_gripper_info.done():
            rclpy.spin_until_future_complete(self.core, self.future_gripper_info)
            rclpy.spin_once(self.core)

        self.gripper_info = self.future_gripper_info.result()
        self.left_finger_index = self.core.js_index_map[
            self.gripper_info.joint_names[0]
        ]
        self.left_finger_lower_limit = self.gripper_info.joint_lower_limits[0]
        self.left_finger_upper_limit = self.gripper_info.joint_upper_limits[0]

        if self.gripper_info.mode != "current" and self.gripper_info.mode != "pwm":
            self.core.get_logger().err(
                "Please set the gripper's 'operating mode' to 'pwm' or 'current'."
            )

        time.sleep(0.5)
        print(
            (
                f"Gripper Name: {self.gripper_name}\n"
                f"Gripper Pressure: {self.gripper_pressure*100}%"
            )
        )
        print("Initialized InterbotixGripperXSInterface!\n")

    def gripper_state(self):
        """ROS Timer Callback function to stop the gripper moving past its
        limits when in PWM mode
        """
        if self.gripper_moving:
            # update gripper position
            with self.core.js_mutex:
                gripper_pos = self.core.joint_states.position[self.left_finger_index]
            # stop the gripper if it has reached the lower or upper limit
            if (
                self.gripper_command.cmd > 0
                and gripper_pos >= self.left_finger_upper_limit
            ) or (
                self.gripper_command.cmd < 0
                and gripper_pos <= self.left_finger_lower_limit
            ):
                self.gripper_command.cmd = float(0.0)
                self.core.pub_single.publish(self.gripper_command)
                self.gripper_moving = False

    def gripper_controller(self, effort: float, delay: float):
        """Helper function used to publish effort commands to the gripper (when in 'pwm'
        or 'current' mode)

        :param effort: effort command to send to the gripper motor
        :param delay: number of seconds to wait before returning control to the user
        """
        self.gripper_command.cmd = effort
        # update gripper position
        with self.core.js_mutex:
            gripper_pos = self.core.joint_states.position[self.left_finger_index]
        # check if the gripper is within its limits
        if (
            self.gripper_command.cmd > 0 and gripper_pos < self.left_finger_upper_limit
        ) or (
            self.gripper_command.cmd < 0 and gripper_pos > self.left_finger_lower_limit
        ):
            self.core.pub_single.publish(self.gripper_command)
            self.gripper_moving = True
            time.sleep(delay)

    def set_pressure(self, pressure: float):
        """Set the amount of pressure that the gripper should use when grasping an object
        (when in 'effort' control mode)

        :param pressure: a scaling factor from 0 to 1 where the pressure increases as
            the factor increases
        """
        self.gripper_value = self.gripper_pressure_lower_limit + pressure * (
            self.gripper_pressure_upper_limit - self.gripper_pressure_lower_limit
        )

    def open(self, delay: float = 1.0):
        """Opens the gripper (when in 'pwm' control mode)

        :param delay: number of seconds to delay before returning control to the user
        """
        self.gripper_controller(self.gripper_value, delay)

    def close(self, delay: float = 1.0):
        """Closes the gripper (when in 'pwm' control mode)

        :param delay: number of seconds to delay before returning control to the user
        """
        self.gripper_controller(-self.gripper_value, delay)
