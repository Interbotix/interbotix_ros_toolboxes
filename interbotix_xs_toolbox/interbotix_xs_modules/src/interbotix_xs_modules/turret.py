import math
import rospy
import numpy as np
from interbotix_xs_sdk.msg import *
from interbotix_xs_modules.core import InterbotixRobotXSCore

### @brief Standalone Module to control an Interbotix Turret
### @param robot_model - Interbotix turret model (ex. 'wxxmt' or 'pxxls')
### @param robot_name - defaults to value given to 'robot_model'; this can be customized if controlling two of the same turrets from one computer (like 'turret1/wxxmt' and 'turret2/pxxls')
### @param group_name - name of the desired Turret's pan-tilt joint group
### @param pan_profile_type - 'pan' joint settting; refer to the OperatingModes Service file for an explanation - can be either 'time' or 'velocity'
### @param pan_profile_velocity - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param pan_profile_acceleration - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param tilt_profile_type - 'tilt' joint settting; refer to the OperatingModes Service file for an explanation - can be either 'time' or 'velocity'
### @param tilt_profile_velocity - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param tilt_profile_acceleration - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param init_node - set to True if the InterbotixRobotXSCore class should initialize the ROS node - this is the most Pythonic approach; to incorporate a robot into an existing ROS node though, set to False
class InterbotixTurretXS(object):
    def __init__(self, robot_model, robot_name=None, group_name="turret", pan_profile_type="time", pan_profile_velocity=2.0, pan_profile_acceleration=0.3, tilt_profile_type="time", tilt_profile_velocity=2.0, tilt_profile_acceleration=0.3, init_node=True):
        self.dxl = InterbotixRobotXSCore(robot_model, robot_name, init_node)
        self.turret = InterbotixTurretXSInterface(self.dxl, group_name, pan_profile_type, pan_profile_velocity, pan_profile_acceleration, tilt_profile_type, tilt_profile_velocity, tilt_profile_acceleration)

### @brief Definition of the Interbotix Turret Module
### @param core - reference to the InterbotixRobotXSCore class containing the internal ROS plumbing that drives the Python API
### @param group_name - name of the desired Turret's pan-tilt joint group
### @param pan_profile_type - 'pan' joint settting; refer to the OperatingModes Service file for an explanation - can be either 'time' or 'velocity'
### @param pan_profile_velocity - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param pan_profile_acceleration - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param tilt_profile_type - 'tilt' joint settting; refer to the OperatingModes Service file for an explanation - can be either 'time' or 'velocity'
### @param tilt_profile_velocity - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
### @param tilt_profile_acceleration - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
class InterbotixTurretXSInterface(object):
    def __init__(self, core, group_name="turret", pan_profile_type="time", pan_profile_velocity=2.0, pan_profile_acceleration=0.3, tilt_profile_type="time", tilt_profile_velocity=2.0, tilt_profile_acceleration=0.3):
        self.core = core
        group_info = self.core.srv_get_info("group", group_name)
        self.group_name = group_name
        self.pan_name = group_info.joint_names[0]
        self.tilt_name = group_info.joint_names[1]
        pan_limits = [group_info.joint_lower_limits[0], group_info.joint_upper_limits[0]]
        tilt_limits = [group_info.joint_lower_limits[1], group_info.joint_upper_limits[1]]
        pan_position = self.core.joint_states.position[self.core.js_index_map[self.pan_name]]
        tilt_position = self.core.joint_states.position[self.core.js_index_map[self.tilt_name]]
        self.info = {self.pan_name : {"command" : pan_position, "profile_type" : pan_profile_type, "profile_velocity" : pan_profile_velocity, "profile_acceleration" : pan_profile_acceleration, "lower_limit" : pan_limits[0], "upper_limit" : pan_limits[1]}, \
                     self.tilt_name : {"command" : tilt_position, "profile_type" : tilt_profile_type, "profile_velocity" : tilt_profile_velocity, "profile_acceleration" : tilt_profile_acceleration, "lower_limit" : tilt_limits[0], "upper_limit" : tilt_limits[1]}}
        self.change_profile(self.pan_name, pan_profile_type, pan_profile_velocity, pan_profile_acceleration)
        self.change_profile(self.tilt_name, tilt_profile_type, tilt_profile_velocity, tilt_profile_acceleration)
        print("Turret Group Name: %s\nPan Name: %s, Profile Type: %s, Profile Velocity: %.1f, Profile Acceleration: %.1f\nTilt Name: %s, Profile Type: %s, Profile Velocity: %.1f, Profile Acceleration: %.1f" \
        % (group_name, self.pan_name, pan_profile_type, pan_profile_velocity, pan_profile_acceleration, self.tilt_name, tilt_profile_type, tilt_profile_velocity, tilt_profile_acceleration))
        print("Initialized InterbotixTurretXSInterface!\n")

    ### @brief Helper function to command the 'Profile_Velocity' and 'Profile_Acceleration' motor registers
    ### @param joint_name - joint to change
    ### @param profile_velocity - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param profile_acceleration - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @details - note that if 'profile_velocity' and 'profile_acceleration' are not set, they retain the values they were set with previously
    def set_trajectory_profile(self, joint_name, profile_velocity=None, profile_acceleration=None):
        if (profile_velocity != None and profile_velocity != self.info[joint_name]["profile_velocity"]):
            if (self.info[joint_name]["profile_type"] == "velocity"):
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Velocity", value=profile_velocity)
            else:
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Velocity", value=int(profile_velocity * 1000))
            self.info[joint_name]["profile_velocity"] = profile_velocity
        if (profile_acceleration != None and profile_acceleration != self.info[joint_name]["profile_acceleration"]):
            if (self.info[joint_name]["profile_type"] == "velocity"):
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Acceleration", value=profile_acceleration)
            else:
                self.core.srv_set_reg(cmd_type="single", name=joint_name, reg="Profile_Acceleration", value=int(profile_acceleration * 1000))
            self.info[joint_name]["profile_acceleration"] = profile_acceleration

    ### @brief Helper function to move a turret joint
    ### @param joint_name - joint to change
    ### @param position - desired goal position [rad]
    ### @param profile_velocity - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param profile_acceleration - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param blocking - if 'profile_type' is 'time' and this is set to True, the function waits 'profile_velocity' seconds before returning control to the user; otherwise, the 'delay' parameter is used
    ### @param delay - number of seconds to wait after executing the position command before returning control to the user
    ### @details - note that if 'profile_velocity' and 'profile_acceleration' are not set, they retain the values they were set with previously
    def move(self, joint_name, position, profile_velocity=None, profile_acceleration=None, blocking=True, delay=0):
        if (self.info[joint_name]["lower_limit"] <= position <= self.info[joint_name]["upper_limit"]):
            self.set_trajectory_profile(joint_name, profile_velocity, profile_acceleration)
            self.core.pub_single.publish(JointSingleCommand(joint_name, position))
            self.info[joint_name]["command"] = position
            if (self.info[joint_name]["profile_type"] == "time" and blocking == True):
                rospy.sleep(self.info[joint_name]["profile_velocity"])
            else:
                rospy.sleep(delay)
        else:
            rospy.logwarn("Goal position is outside the %s joint's limits." % joint_name)

    ### @brief Commands the Pan joint on the Turret
    ### @param position - desired goal position [rad]
    ### @param profile_velocity - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param profile_acceleration - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param blocking - if 'profile_type' is 'time' and this is set to True, the function waits 'profile_velocity' seconds before returning control to the user; otherwise, the 'delay' parameter is used
    ### @param delay - number of seconds to wait after executing the position command before returning control to the user
    ### @details - note that if 'profile_velocity' and 'profile_acceleration' are not set, they retain the values they were set with previously
    def pan(self, position, profile_velocity=None, profile_acceleration=None, blocking=True, delay=0):
        self.move(self.pan_name, position, profile_velocity, profile_acceleration, blocking, delay)

    ### @brief Commands the Tilt joint on the Turret
    ### @param position - desired goal position [rad]
    ### @param profile_velocity - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param profile_acceleration - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param blocking - if 'profile_type' is 'time' and this is set to True, the function waits 'profile_velocity' seconds before returning control to the user; otherwise, the 'delay' parameter is used
    ### @param delay - number of seconds to wait after executing the position command before returning control to the user
    ### @details - note that if 'profile_velocity' and 'profile_acceleration' are not set, they retain the values they were set with previously
    def tilt(self, position, profile_velocity=None, profile_acceleration=None, blocking=True, delay=0):
        self.move(self.tilt_name, position, profile_velocity, profile_acceleration, blocking, delay)

    ### @brief Resets the Turret to its Home pose (0 rad for both joints)
    ### @param pan_profile_velocity - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param pan_profile_acceleration - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param tilt_profile_velocity - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param tilt_profile_acceleration - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param blocking - if 'profile_type' for both joints is 'time' and this is set to True, the function waits either 'pan_profile_velocity' or 'tilt_profile_velocity' seconds (whichever is greater) before returning control to the user; otherwise, the 'delay' parameter is used
    ### @param delay - number of seconds to wait after executing the position command before returning control to the user
    ### @details - note that if the 'profile_velocity' and 'profile_acceleration' parameters are not set, they retain the values they were set with previously
    def pan_tilt_go_home(self, pan_profile_velocity=None, pan_profile_acceleration=None, tilt_profile_velocity=None, tilt_profile_acceleration=None, blocking=True, delay=0):
        self.pan_tilt_move(0, 0, pan_profile_velocity, pan_profile_acceleration, tilt_profile_velocity, tilt_profile_acceleration, blocking, delay)

    ### @brief Commands the pan and tilt joints on the Turret simultaneously
    ### @param pan_position - desired pan goal position [rad]
    ### @param tilt_position - desired tilt goal position [rad]
    ### @param pan_profile_velocity - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param pan_profile_acceleration - 'pan' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param tilt_profile_velocity - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param tilt_profile_acceleration - 'tilt' joint setting; refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param blocking - if 'profile_type' for both joints is 'time' and this is set to True, the function waits either 'pan_profile_velocity' or 'tilt_profile_velocity' seconds (whichever is greater) before returning control to the user; otherwise, the 'delay' parameter is used
    ### @param delay - number of seconds to wait after executing the position command before returning control to the user
    ### @details - note that if the 'profile_velocity' and 'profile_acceleration' parameters are not set, they retain the values they were set with previously
    def pan_tilt_move(self, pan_position, tilt_position, pan_profile_velocity=None, pan_profile_acceleration=None, tilt_profile_velocity=None, tilt_profile_acceleration=None, blocking=True, delay=0):
        if (self.info[self.pan_name]["lower_limit"] <= pan_position <= self.info[self.pan_name]["upper_limit"]) and \
           (self.info[self.tilt_name]["lower_limit"] <= tilt_position <= self.info[self.tilt_name]["upper_limit"]):
           self.set_trajectory_profile(self.pan_name, pan_profile_velocity, pan_profile_acceleration)
           self.set_trajectory_profile(self.tilt_name, tilt_profile_velocity, tilt_profile_acceleration)
           self.core.pub_group.publish(JointGroupCommand(self.group_name, [pan_position, tilt_position]))
           self.info[self.pan_name]["command"] = pan_position
           self.info[self.tilt_name]["command"] = tilt_position
           if (self.info[self.pan_name]["profile_type"] == "time" and self.info[self.tilt_name]["profile_type"] == "time" and blocking == True):
               rospy.sleep(max(self.info[self.pan_name]["profile_velocity"], self.info[self.tilt_name]["profile_velocity"]))
           else:
               rospy.sleep(delay)
        else:
           rospy.logwarn("One or both goal positions are outside the limits!")

    ### @brief Change the Profile Type for a given joint
    ### @param joint_name - joint to change
    ### @param profile_type - either 'time' or 'velocity'; refer to the OperatingModes Service file for details
    ### @param profile_velocity - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    ### @param profile_acceleration - refer to the OperatingModes Service file for an explanation - note that when 'profile_type' is 'time', units are in seconds, not milliseconds
    def change_profile(self, joint_name, profile_type, profile_velocity, profile_acceleration):
        if (profile_type == "velocity"):
            self.core.srv_set_op_modes("single", joint_name, "position", "velocity", profile_velocity, profile_acceleration)
            self.info[joint_name]["profile_velocity"] = profile_velocity
            self.info[joint_name]["profile_acceleration"] = profile_acceleration
        else:
            self.core.srv_set_op_modes("single", joint_name, "position", "time", int(profile_velocity * 1000), int(profile_acceleration * 1000))
            self.info[joint_name]["profile_velocity"] = profile_velocity
            self.info[joint_name]["profile_acceleration"] = profile_acceleration
        self.info[joint_name]["profile_type"] = profile_type

    ### @brief Get the last commanded position for a given joint
    ### @param joint_name - desired joint name
    ### @return last commanded position [rad]
    def get_command(self, joint_name):
        return self.info[joint_name]["command"]

    ### @brief Get the last commanded positions for the joints
    ### @return list of last commanded positions [rad]
    def get_joint_commands(self):
        return [self.info[self.pan_name]["command"], self.info[self.tilt_name]["command"]]
