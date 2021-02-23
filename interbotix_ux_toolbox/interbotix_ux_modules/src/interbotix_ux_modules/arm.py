import math
import rospy
import numpy as np
import modern_robotics as mr
from urdf_parser_py.urdf import URDF
from interbotix_ux_modules import mr_descriptions as mrd
from interbotix_common_modules import angle_manipulation as ang
from interbotix_ux_modules.core import InterbotixRobotUXCore
from interbotix_ux_modules.gripper import InterbotixGripperUXInterface

### Note that this module uses the Modern Robotics approach to perform Inverse Kinematics instead of the Xarm built-in one
### To use the Xarm built-in Inverse Kinematics solver, use the InterbotixRobotUXCore class instead

### @brief Standalone Module to control a Universal Factory Xarm and Gripper
### @param robot_model - Universal Factor Xarm model (ex. 'uxarm5' or 'uxarm6')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same arms from one computer (like 'arm1/uxarm6' and 'arm2/uxarm6')
### @param mode - Universal Factory Xarm Mode; can be 0 (pose mode), 1 (servo mode), or 2 (teach mode)
### @param wait_for_finish - set to True to have the function wait until arm is finished moving before returning (only applicable in Mode 0); otherwise, set to False
### @param ee_offset - transform from the current end-effector frame to the desired end-effector frame [x, y, z, roll, pitch, yaw] (units are meters/radians)
### @param init_node - set to True if the InterbotixRobotUXCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
### @param joint_state_topic - desired joint_state topic name to subscribe to; note that the name is resolved relative to the 'robot_name' namespace
### @param pulse_vel - desired gripper speed [1 - 5000]
### @param pulse - desired initial gripper pulse from 0 (closed) to 850 (fully open)
### @param gripper_type - type of gripper being used; currently, only "gripper" for the standard gripper and 'None' for no gripper are supported
class InterbotixManipulatorUX(object):
    def __init__(self, robot_model, robot_name=None, mode=0, wait_for_finish=True, ee_offset=None, init_node=True, joint_state_topic="joint_states", pulse_vel=1500, pulse=850, gripper_type="gripper"):
        self.ux = InterbotixRobotUXCore(robot_model, robot_name, mode, wait_for_finish, ee_offset, init_node, joint_state_topic)
        self.arm = InterbotixArmUXInterface(self.ux)
        if gripper_type is "gripper":
            self.gripper = InterbotixGripperUXInterface(self.ux, pulse_vel, pulse)

### @brief Definition of the Interbotix Arm Module
### @param core - reference to the InterbotixRobotUXCore class containing the internal ROS plumbing that drives the Python API
### @param robot_model - Universal Factor Xarm model (ex. 'uxarm5' or 'uxarm6')
class InterbotixArmUXInterface(object):
    def __init__(self, core):
        self.core = core                                                                             # Reference to the InterbotixRobotUXCore object
        self.robot_des = getattr(mrd, self.core.robot_model)                                         # Modern Robotics parameters
        self.limits = {name : {"lower":0, "upper":0} for name in self.core.joint_names}              # Limit Info for the joints
        self.get_urdf_limits()
        self.joint_commands = []                                                                     # Holds the latest joint commands
        self.index_map = dict(zip(self.core.joint_names, range(len(self.core.joint_names))))         # Maps joint names with their indexes
        self.initial_guesses = self.robot_des.Guesses                                                # Initial guesses for seeding the IK Solver
        self.initial_guesses.append(self.joint_commands)
        self.hold_up_positions = [0] * self.core.dof                                                  # Define Hold-Up pose as shown in MoveIt
        self.hold_up_positions[-2] = -1.57                                                            # DO NOT use -math.pi/2 to avoid potential singularities
        if self.core.ee_offset is not None:                                                           # Adjust M-matrix based on desired ee_offset
            T_bf = ang.poseToTransformationMatrix(self.core.ee_offset)
            self.robot_des.M = np.dot(self.robot_des.M, T_bf)
        self.capture_joint_positions()
        rospy.loginfo("Initializing InterbotixArmUXInterface...")
        rospy.loginfo("Complete!")

    ### @brief Get joint limit information from the URDF
    def get_urdf_limits(self):
        full_rd_name = "/" + self.core.robot_name + "/robot_description"
        while rospy.has_param(full_rd_name) != True: pass
        robot_description = URDF.from_parameter_server(key=full_rd_name)
        for joint in self.core.joint_names:
            joint_object = next((j for j in robot_description.joints if j.name == joint), None)
            self.limits[joint]["lower"] = joint_object.limit.lower
            self.limits[joint]["upper"] = joint_object.limit.upper

    ### @brief Helper function to publish joint positions
    ### @param positions - desired joint positions
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @param mode - Mode the robot should be in before executing commands (0 for pose mode or 1 for servo mode)
    def command_positions(self, positions, vel=1.0, accel=5.0, mode=0):
        self.joint_commands = list(positions)
        if self.core.mode != mode:
            self.core.robot_smart_mode_reset(mode)
        if (mode == 0): self.core.robot_move_joint(positions, vel, accel)
        elif (mode == 1): self.core.robot_move_servoj(positions)
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)


    ### @brief Helper function to check to make sure the desired joint positions are all within their respective limits
    ### @param positions - the positions [rad] to check
    ### @return <bool> - True if all positions are within limits; False otherwise
    def check_joint_limits(self, positions):
        theta_list = [int(elem * 1000)/1000.0 for elem in positions]
        cntr = 0
        for name in self.core.joint_names:
            if not (self.limits[name]["lower"] <= theta_list[cntr] <= self.limits[name]["upper"]):
                return False
            cntr += 1
        return True

    ### @brief Helper function to check to make sure a desired position for a given joint is within its limits
    ### @param joint_name - desired joint name
    ### @param position - desired joint position [rad]
    ### @return <bool> - True if within limits; False otherwise
    def check_single_joint_limit(self, joint_name, position):
        theta = int(position * 1000)/1000.0
        if not (self.limits[joint_name]["lower"] <= theta <= self.limits[joint_name]["upper"]):
            return False
        return True

    ### @brief Helper function to set one of two preset poses
    ### @param preset_pose - can be either "Home" or "Hold-Up" (like in MoveIt)
    ### @param vel - max velocity [rad/s] of the joints
    ### @param accel - max acceleration [rad/s^2] of the joints
    def move_to_preset_pose(self, preset_pose="Home", vel=1.0, accel=5.0):
        if preset_pose == "Home":
            positions = [0] * self.core.dof
        elif preset_pose == "Hold-Up":
            positions = self.hold_up_positions
        self.command_positions(positions, vel, accel, 0)

    ### @brief Commands the robot to its Home pose
    ### @param vel - max velocity [rad/s] of the joints
    ### @param accel - max acceleration [rad/s^2] of the joints
    def go_to_home_pose(self, vel=1.0, accel=5.0):
        self.move_to_preset_pose("Home", vel, accel)

    ### @brief Commands the robot to its Hold-Up pose
    ### @param vel - max velocity [rad/s] of the joints
    ### @param accel - max acceleration [rad/s^2] of the joints
    def go_to_holdup_pose(self, vel=1.0, accel=5.0):
        self.move_to_preset_pose("Hold-Up", vel, accel)

    ### @brief Command positions to the arm joints
    ### @param positions - desired joint positions [rad]
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @param mode - Mode the robot should be in before executing commands (0 for pose mode or 1 for servo mode)
    ### @return <bool> - True if positions were commanded; False if they weren't due to being outside limits
    def set_joint_positions(self, positions, vel=1.0, accel=5.0, mode=0):
        if (self.check_joint_limits(joint_positions)):
            self.command_positions(joint_positions, vel, accel, 0)
        else:
            return False

    ### @brief Command a single joint to a desired position
    ### @param joint_name - name of the joint to control
    ### @param position - desired position [rad]
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @param mode - Mode the robot should be in before executing commands (0 for pose mode or 1 for servo mode)
    ### @return <bool> - True if the position was commanded; False if it wasn't due to being outside limits
    def set_single_joint_position(self, joint_name, position, vel=1.0, accel=5.0, mode=0):
        if not self.check_single_joint_limit(joint_name, position):
            return False
        self.joint_commands[self.index_map[joint_name]] = position
        self.command_positions(self.joint_commands, vel, accel, mode)
        return True

    ### @brief Command a desired end-effector pose
    ### @param T_sd - 4x4 Transformation Matrix representing the transform from the /<robot_name>/base_link frame to the end-effector frame
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @param mode - Mode the robot should be in before executing commands (0 for pose mode or 1 for servo mode)
    ### @return theta_list - joint values needed to get the end-effector to the desired pose
    ### @return <bool> - True if a valid solution was found; False otherwise
    def set_ee_pose_matrix(self, T_sd, custom_guess=None, execute=True, vel=1.0, accel=5.0, mode=0):
        if (custom_guess is None):
            initial_guesses = self.initial_guesses
            initial_guesses[3] = self.joint_commands
        else:
            initial_guesses = [custom_guess]

        for guess in initial_guesses:
            theta_list, success = mr.IKinSpace(self.robot_des.Slist, self.robot_des.M, T_sd, guess, 0.0001, 0.0001)
            solution_found = True

            # Check to make sure a solution was found and that no joint limits were violated
            if success:
                solution_found = self.check_joint_limits(theta_list)
            else:
                solution_found = False

            if solution_found:
                if execute:
                    self.command_positions(theta_list, vel, accel, mode)
                return theta_list, True

        rospy.logwarn("No valid pose could be found")
        return theta_list, False

    ### @brief Command a desired end-effector pose w.r.t. the Space frame
    ### @param x - linear position along the X-axis of the Space frame [m]
    ### @param y - linear position along the Y-axis of the Space frame [m]
    ### @param z - linear position along the Z-axis of the Space frame [m]
    ### @param roll - angular position around the X-axis of the Space frame [rad]
    ### @param pitch - angular position around the Y-axis of the Space frame [rad]
    ### @param yaw - angular position around the Z-axis of the Space frame [rad]
    ### @param custom_guess - list of joint positions with which to seed the IK solver
    ### @param execute - if True, this moves the physical robot after planning; otherwise, only planning is done
    ### @param vel - desired joint velocity [rad/s]
    ### @param accel - desired joint acceleration [rad/s^2]
    ### @return theta_list - joint values needed to get the end-effector to the desired pose
    ### @return <bool> - True if a valid solution was found; False otherwise
    def set_ee_pose_components(self, x=0, y=0, z=0, roll=math.pi, pitch=0, yaw=0, custom_guess=None, execute=True, vel=1.0, accel=5.0):
        T_sd = ang.poseToTransformationMatrix([x, y, z, roll, pitch, yaw])
        return self.set_ee_pose_matrix(T_sd, custom_guess, execute, vel, accel, 0)

    ### @brief Command a desired end-effector displacement that will follow a straight line path (when in 'servo' control mode)
    ### @param x - linear displacement along the X-axis w.r.t. T_sy [m]
    ### @param y - linear displacement along the Y-axis w.r.t. T_sy [m]
    ### @param z - linear displacement along the Z-axis w.r.t. T_sy [m]
    ### @param roll - angular displacement around the X-axis w.r.t. T_sy [rad]
    ### @param pitch - angular displacement around the Y-axis w.r.t. T_sy [rad]
    ### @param yaw - angular displacement around the Z-axis w.r.t. T_sy [rad]
    ### @param moving_time - duration in seconds that the robot should move
    ### @param wp_period - duration in seconds between each waypoint
    ### @return <bool> - True if a trajectory was succesfully planned and executed; otherwise False
    ### @details - T_sy is a 4x4 transformation matrix representing the pose of a virtual frame w.r.t. /<robot_name>/base_link.
    ###            This virtual frame has the exact same x, y, z, roll, and pitch of /<robot_name>/base_link but contains the yaw
    ###            of the end-effector frame (/<robot_name>/linkX).
    def set_ee_cartesian_trajectory(self, x=0, y=0, z=0, roll=0, pitch=0, yaw=0, moving_time=2.0, wp_period=0.02):
        rpy = ang.rotationMatrixToEulerAngles(self.T_sb[:3,:3])
        T_sy = np.identity(4)
        T_sy[:2,:2] = ang.yawToRotationMatrix(rpy[2])
        T_yb = np.dot(mr.TransInv(T_sy), self.T_sb)
        rpy = ang.rotationMatrixToEulerAngles(T_yb[:3,:3])
        N = int(moving_time / wp_period)
        inc = 1.0 / float(N)
        joint_traj = []
        joint_positions = list(self.joint_commands)
        for i in range(N+1):
            joint_traj.append(joint_positions)
            if (i == N):
                break
            T_yb[:3,3] += [inc * x, inc * y, inc * z]
            rpy[0] += inc * roll
            rpy[1] += inc * pitch
            rpy[2] += inc * yaw
            T_yb[:3,:3] = ang.eulerAnglesToRotationMatrix(rpy)
            T_sd = np.dot(T_sy, T_yb)
            theta_list, success = self.set_ee_pose_matrix(T_sd, joint_positions, False)
            if success:
                joint_positions = theta_list
            else:
                rospy.loginfo("%.1f%% of trajectory successfully planned. Trajectory will not be executed." % (i/float(N) * 100))
                break

        if success:
            mode = self.core.mode
            if (mode != 1):
                self.core.robot_smart_mode_reset(1)
            r = rospy.Rate(1/wp_period)
            for cmd in joint_traj:
                self.core.robot_move_servoj(cmd)
                r.sleep()
            self.T_sb = T_sd
            self.joint_commands = joint_positions

        return success

    ### @brief Get the latest commanded joint positions
    ### @return - list of latest commanded joint positions [rad]
    def get_joint_commands(self):
        return list(self.joint_commands)

    ### @brief Get the latest commanded position for a given joint
    ### @param joint_name - joint for which to get the position
    ### @return - desired position [rad]
    def get_single_joint_command(self, joint_name):
        return self.joint_commands[self.index_map[joint_name]]

    ### @brief Get the latest commanded end-effector pose w.r.t the Space frame
    ### @return <4x4 matrix> - Transformation matrix
    def get_ee_pose_command(self):
        return np.array(self.T_sb)

    ### @brief Get the actual end-effector pose w.r.t the Space frame
    ### @return <4x4 matrix> - Transformation matrix
    def get_ee_pose(self):
        joint_states = [self.core.joint_states.position[self.core.js_index_map[name]] for name in self.group_info.joint_names]
        T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, joint_states)
        return T_sb

    ### @brief Resets self.joint_commands to be the actual positions seen by the encoders
    ### @details - should be used whenever joints are torqued off / zero-gravity, right after torquing them on again
    def capture_joint_positions(self):
        self.joint_commands = []
        for name in self.core.joint_names:
            self.joint_commands.append(self.core.joint_states.position[self.core.js_index_map[name]])
        self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)
