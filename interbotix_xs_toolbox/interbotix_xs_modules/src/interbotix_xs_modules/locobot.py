import rospy
from interbotix_xs_modules.core import InterbotixRobotXSCore
from interbotix_xs_modules.arm import InterbotixArmXSInterface
from interbotix_xs_modules.turret import InterbotixTurretXSInterface
from interbotix_xs_modules.gripper import InterbotixGripperXSInterface
from interbotix_perception_modules.pointcloud import InterbotixPointCloudInterface
from interbotix_perception_modules.armtag import InterbotixArmTagInterface

### @brief Standalone Module to control an Interbotix LoCoBot with a mobile base
### @param robot_model - Interbotix LoCoBot model (ex. 'locobot_px100' or 'locobot_base')
### @param arm_model - Interbotix Manipulator model (ex. 'mobile_px100' or 'mobile_wx200'); if set
###        to `None`, does not add an arm, gripper, or apriltag component to the robot
### @param arm_group_name - joint group name that contains the 'arm' joints as defined in the
###        'motor_config' yaml file
### @param gripper_name - name of the gripper joint as defined in the 'motor_config' yaml file;
###        typically, this is 'gripper'
### @param turret_group_name - joint group name that contains the 'turret' joints as defined in the
###        'motor_config' yaml file; typically, this is 'camera'
### @param robot_name - defaults to the value given to 'robot_model' if unspecified; this can be
###        customized if controlling two or more LoCoBots from one computer (like 'locobot1' and
###        'locobot2')
### @param init_node - set to `True` if the InterbotixRobotXSCore class should initialize the ROS
###        node - this is the most Pythonic approach; to incorporate a robot into an existing ROS
###        node though, set to `False`
### @param dxl_joint_states - name of the joint states topic that contains just the states of the
###        dynamixel servos
### @param base_joint_states - name of the joints states topic that contains the states of the
###        base's two wheels
### @param use_move_base_action - whether or not Move-Base's Action Server should be used instead
###        of the Topic interface; set to `True` to make the 'move_to_pose' function block until
###        the robot reaches its goal pose
class InterbotixLocobotXS(object):
    def __init__(
        self,
        robot_model,
        arm_model=None,
        arm_group_name="arm",
        gripper_name="gripper",
        turret_group_name="camera",
        robot_name="locobot",
        init_node=True,
        dxl_joint_states="dynamixel/joint_states",
        base_joint_states="mobile_base/joint_states",
        use_move_base_action=False
    ):
        # Create DYNAMIXEL core interface
        self.dxl = InterbotixRobotXSCore(
            robot_model=robot_model,
            robot_name=robot_name,
            init_node=init_node,
            joint_state_topic=dxl_joint_states
        )
        # Create turret interface for the camera
        self.camera = InterbotixTurretXSInterface(
            core=self.dxl,
            group_name=turret_group_name
        )
        # Create mobile base interface if using base
        if rospy.has_param("/" + robot_name + "/use_base") and rospy.get_param("/" + robot_name + "/use_base"):
            base_type = rospy.get_param("/" + robot_name + "/base_type", "create3")
            if base_type == "kobuki":
                from interbotix_xs_modules.kobuki import InterbotixKobukiInterface
                self.base = InterbotixKobukiInterface(
                    robot_name=robot_name,
                    base_joint_states=base_joint_states,
                    use_move_base_action=use_move_base_action
                )
            elif base_type == "create3":
                from interbotix_xs_modules.create3 import InterbotixCreate3Interface
                self.base = InterbotixCreate3Interface(
                    robot_name=robot_name,
                    base_joint_states="/mobile_base/wheel_ticks",
                    use_move_base_action=use_move_base_action
                )
            else:
                raise ValueError(
                    "Parameter " + "'/" + robot_name + "/base_type' is an invalid value: was '"
                    + base_type + "' but must be 'kobuki' or 'create3'."
                )
        # Create PointCloud Interface if using perception
        if rospy.has_param("/" + robot_name + "/use_perception") and rospy.get_param("/" + robot_name + "/use_perception"):
            self.pcl = InterbotixPointCloudInterface(
                filter_ns=robot_name + "/pc_filter",
                init_node=False
            )
        # Create Arm and Gripper interfaces if LoCoBot has an arm (if arm_model was specified)
        if arm_model is not None:
            self.arm = InterbotixArmXSInterface(
                core=self.dxl,
                robot_model=arm_model,
                group_name=arm_group_name
            )
            self.gripper = InterbotixGripperXSInterface(self.dxl, gripper_name)
            # Create ArmTag interface if using armtag
            if rospy.has_param("/" + robot_name + "/use_armtag") and rospy.get_param("/" + robot_name + "/use_armtag"):
                self.armtag = InterbotixArmTagInterface(
                    armtag_ns=robot_name + "/armtag",
                    apriltag_ns=robot_name + "/apriltag",
                    init_node=False
                )
