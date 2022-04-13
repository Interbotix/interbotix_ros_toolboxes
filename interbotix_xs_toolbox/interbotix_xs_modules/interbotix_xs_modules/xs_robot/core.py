import copy
import rclpy
import threading
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.logging import LoggingSeverity
from interbotix_xs_msgs.msg import *
from interbotix_xs_msgs.srv import *
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

NODE_NAME: str = "robot_manipulation"


class InterbotixRobotXSCore(Node):
    def __init__(
        self,
        robot_model: str,
        robot_name: str = None,
        init_node: bool = True,
        joint_state_topic: str = "joint_states",
        logging_level: LoggingSeverity = LoggingSeverity.INFO,
    ) -> None:
        """Class that interfaces with the xs_sdk node ROS interfaces

        :param robot_model: Interbotix Arm model (ex. 'wx200' or 'vx300s')
        :param robot_name: (optional) defaults to value given to 'robot_model';
            this can be customized if controlling two of the same arms from one
            computer (like 'arm1/wx200' and 'arm2/wx200')
        :param init_node: (optional) set to `True` if the InterbotixRobotXSCore
            class should initialize the ROS node - this is the most Pythonic
            approach; to incorporate a robot into an existing ROS node though,
            set to `False`
        :param joint_state_topic: (optional) the specifc JointState topic output
            by the xs_sdk node
        """
        self.robot_model = robot_model
        self.robot_name = robot_name

        if self.robot_name is None:
            self.robot_name = robot_model

        if init_node:
            rclpy.init()
            rclpy.logging.set_logger_level(NODE_NAME, logging_level)
            super().__init__(node_name=NODE_NAME, namespace=robot_name)

        self.get_logger().debug(
            (
                f"Created node with name='{self.get_name()}' "
                f"in namespace='{self.get_namespace()}'"
            )
        )

        self.joint_state_topic = joint_state_topic
        self.joint_states: JointState = None
        self.js_mutex = threading.Lock()

        self.srv_set_op_modes = self.create_client(
            OperatingModes, f"/{self.robot_name}/set_operating_modes"
        )
        self.srv_set_pids = self.create_client(
            MotorGains, f"/{self.robot_name}/set_motor_pid_gains"
        )
        self.srv_set_reg = self.create_client(
            RegisterValues, f"/{self.robot_name}/set_motor_registers"
        )
        self.srv_get_reg = self.create_client(
            RegisterValues, f"/{self.robot_name}/get_motor_registers"
        )
        self.srv_get_info = self.create_client(
            RobotInfo, f"/{self.robot_name}/get_robot_info"
        )
        self.srv_torque = self.create_client(
            TorqueEnable, f"/{self.robot_name}/torque_enable"
        )
        self.srv_reboot = self.create_client(
            Reboot, f"/{self.robot_name}/reboot_motors"
        )

        # Check for xs_sdk by looking for set_operating_modes
        if not self.srv_set_op_modes.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(
                (
                    "Failed to find services under namespace "
                    f'"{self.robot_name}". Shutting down...'
                )
            )
            exit(1)
        self.srv_set_pids.wait_for_service()
        self.srv_set_reg.wait_for_service()
        self.srv_get_reg.wait_for_service()
        self.srv_get_info.wait_for_service()
        self.srv_torque.wait_for_service()
        self.srv_reboot.wait_for_service()
        self.pub_group = self.create_publisher(
            JointGroupCommand, f"/{self.robot_name}/commands/joint_group", 10
        )
        self.pub_single = self.create_publisher(
            JointSingleCommand, f"/{self.robot_name}/commands/joint_single", 10
        )
        self.pub_traj = self.create_publisher(
            JointTrajectoryCommand, f"/{self.robot_name}/commands/joint_trajectory", 10
        )
        self.sub_joint_states = self.create_subscription(
            JointState,
            f"/{self.robot_name}/{joint_state_topic}",
            self.joint_state_cb,
            10,
        )

    def initialize(self) -> None:
        """Initialize the InterbotixRobotXSCore object"""
        self.get_logger().debug(
            (
                "Trying to find joint states on topic "
                f'"/{self.robot_name}/{self.joint_state_topic}"...'
            )
        )
        while self.joint_states is None and rclpy.ok():
            rclpy.spin_once(self)
        self.get_logger().debug("Found joint states. Continuing...")
        self.js_index_map = dict(
            zip(self.joint_states.name, range(len(self.joint_states.name)))
        )
        print(f"Robot Name: {self.robot_name}\n" f"Robot Model: {self.robot_model}")
        print("Initialized InterbotixRobotXSCore!\n")

    def robot_set_operating_modes(
        self,
        cmd_type: str,
        name: str,
        mode: str,
        profile_type: str = "velocity",
        profile_velocity: int = 0,
        profile_acceleration: int = 0,
    ) -> None:
        """Set the operating mode for either a single motor or a group of motors

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param mode: desired operatinge mode like "position" or "velocity".
        :param profile_type: (optional) can be "time" or "velocity".
        :param profile_velocity: (optional) passthrough to the Profile_Velocity
            register.
        :param profile_acceleration: (optional) passthrough to the Profile_Acceleration
            register.
        :details: See the OperatingModes Service description for all choices
        """
        self.srv_set_op_modes.call(
            OperatingModes.Request(
                cmd_type=cmd_type,
                name=name,
                mode=mode,
                profile_type=profile_type,
                profile_velocity=profile_velocity,
                profile_acceleration=profile_acceleration,
            )
        )

    def robot_set_motor_pid_gains(
        self,
        cmd_type: str,
        name: str,
        kp_pos: int,
        ki_pos: int = 0,
        kd_pos: int = 0,
        k1: int = 0,
        k2: int = 0,
        kp_vel: int = 100,
        ki_vel: int = 1920,
    ) -> None:
        """Set the internal PID gains for either a single motor or a group of motors

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param kp_pos: passthrough to the Position_P_Gain register.
        :param ki_pos: (optional) passthrough to the Position_I_Gain register.
        :param kd_pos: (optional) passthrough to the Position_D_Gain register.
        :param k1: (optional) passthrough to the Feedforward_1st_Gain register.
        :param k2: (optional) passthrough to the Feedforward_2nd_Gain register.
        :param kp_vel: (optional) passthrough to the Velocity_P_Gain register.
        :param ki_vel: (optional) passthrough to the Velocity_I_Gain register.
        :details: See the MotorGains Service description for details
        """
        self.srv_set_pids.call(
            MotorGains.Request(
                cmd_type=cmd_type,
                name=name,
                kp_pos=kp_pos,
                ki_pos=ki_pos,
                kd_pos=kd_pos,
                k1=k1,
                k2=k2,
                kp_vel=kp_vel,
                ki_vel=ki_vel,
            )
        )

    def robot_set_motor_registers(
        self, cmd_type: str, name: str, reg: str, value: int
    ) -> None:
        """Set the desired register for either a single motor or a group of motors

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param reg: desired register name
        :param value: desired value for the above register
        """
        self.srv_set_reg.call(
            RegisterValues.Request(cmd_type=cmd_type, name=name, reg=reg, value=value)
        )

    def robot_get_motor_registers(self, cmd_type: str, name: str, reg: str) -> list:
        """Get the desired register value from either a single motor or a group of motors

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param reg: desired register name
        :return: list of register values
        """
        return self.srv_get_reg.call(
            RegisterValues.Request(cmd_type=cmd_type, name=name, reg=reg)
        )

    def robot_get_robot_info(self, cmd_type: str, name: str) -> RobotInfo.Response:
        """Get information about the robot - mostly joint limit data

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :return: an object with the same structure as a RobotInfo Service description
        """
        return self.srv_get_info(cmd_type, name)

    def robot_torque_enable(self, cmd_type: str, name: str, enable: bool) -> None:
        """Torque a single motor or a group of motors to be on or off

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param enable: `True` to torque on or `False` to torque off
        """
        self.srv_torque.call(
            TorqueEnable.Request(cmd_type=cmd_type, name=name, enable=enable)
        )

    def robot_reboot_motors(
        self, cmd_type: str, name: str, enable: bool, smart_reboot: bool = False
    ):
        """Reboot a single motor or a group of motors if they are in an error state

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param enable: `True` to torque on or `False` to leave torqued off after
            rebooting
        :param smart_reboot: (optional) if `cmd_type` is set to 'group', setting this to
            `True` will only reboot those motors that are in an error state (as opposed
            to all motors within the group regardless of if they are in an error state)
        """
        self.srv_reboot.call(
            Reboot.Request(
                cmd_type=cmd_type, name=name, enable=enable, smart_reboot=smart_reboot
            )
        )

    def robot_write_commands(self, group_name: str, commands: list) -> None:
        """Command a group of motors (refer to the JointGroupCommand Message description
        for more info)

        :param group_name: the group name of the motors to command
        :param commands: desired list of commands
        """
        self.pub_group.publish(JointGroupCommand(name=group_name, cmd=commands))

    def robot_write_joint_command(self, joint_name: str, command: float) -> None:
        """Command a single motor (refer to the JointSingleCommand Message description
        for more info)

        :param joint_name: the name of the motor to command
        :param command: desired command
        """
        self.pub_single.publish(JointSingleCommand(name=joint_name, cmd=command))

    def robot_write_trajectory(
        self, cmd_type: str, name: str, type: str, raw_traj: dict
    ) -> None:
        """Command a trajectory of positions or velocities to a single motor or a group
        of motors

        :param cmd_type: can be "group" for a group of motors or "single" for a single
            motor
        :param name: group name if `cmd_type` is 'group' or the motor name if `cmd_type`
            is 'single'
        :param type: "position" if the trajectory is a list of positions [rad];
            otherwise "velocity" if the trajectory is a list of velocities [rad/s]
        :param raw_traj: list of dictionaries where each dictionary is made up of a
            float / list of float pairs. the 'key' is the desired time [sec] from start
            that the 'value' (list of floats) should be executed.
        :details - an example input trajectory for a pan/tilt mechansim could look like:
            [{0.0, [1,  1]   },
             {1.5, [-1, 0.75]},
             {2.3, [0,  0]   }]
        """
        traj = JointTrajectory()
        for point in raw_traj:
            for key, value in point.items():
                traj_point = JointTrajectoryPoint()
                if type == "position":
                    traj_point.positions = value
                elif type == "velocity":
                    traj_point.velocities = value
                traj_point.time_from_start = Duration(seconds=key)
                traj.points.append(traj_point)
        msg = JointTrajectoryCommand(cmd_type, name, traj)
        self.pub_traj.publish(msg)

    def robot_get_joint_states(self) -> JointState:
        """Get the current joint states (position, velocity, effort) of all DYNAMIXEL
        motors

        :return: JointState ROS message. Refer to online documenation to see its
            structure
        """
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        return joint_states

    def robot_get_single_joint_state(self, name: str) -> dict:
        """Get a single joint state for the specified DYNAMIXEL motor

        :param name: desired motor name for which to get the joint state
        :return: dictionary with 3 keys: "position", "velocity", and "effort". Units are
            rad, rad/s, and mA
        """
        joint_states = None
        with self.js_mutex:
            joint_states = copy.deepcopy(self.joint_states)
        joint_index = joint_states.name.index(name)
        joint_info = {}
        joint_info["position"] = joint_states.position[joint_index]
        joint_info["velocity"] = joint_states.velocity[joint_index]
        joint_info["effort"] = joint_states.effort[joint_index]
        return joint_info

    def joint_state_cb(self, msg: JointState):
        """ROS Subscriber Callback function to get the latest JointState message

        :param msg: JointState message
        """
        with self.js_mutex:
            self.joint_states = msg
