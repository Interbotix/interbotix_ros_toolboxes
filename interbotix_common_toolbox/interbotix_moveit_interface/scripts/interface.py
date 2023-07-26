#!/usr/bin/env python3

import os
import sys
import yaml
import rclpy
import numpy as np

# message libraries
from geometry_msgs.msg import PoseStamped, Pose

# moveit_py
from moveit.planning import MoveItPy
from moveit.core.robot_state import RobotState

# config file libraries
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory


# we need to specify our moveit_py config at the top of each notebook we use.
# this is since we will start spinning a moveit_py node within this notebook.
def main():

    print("This works")
    # moveit_config = (
    #     MoveItConfigsBuilder(robot_name="dx400", package_name="interbotix_xscobot_moveit")
    #     .robot_description(file_path="/config/dx400.urdf.xacro")
    #     .trajectory_execution(file_path="config/moveit_controllers.yaml")
    #     .moveit_cpp(
    #         file_path=os.path.join(
    #             get_package_share_directory("interbotix_moveit_interface"),
    #             "config",
    #             "planning.yaml"
    #     )
    # )
    # .to_moveit_configs()
    # ).to_dict()

    # # initialise rclpy (only for logging purposes)
    # rclpy.init()

    # # instantiate moveit_py instance and a planning component for the panda_arm
    # xscobot = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    # interbotix_arm = xscobot.get_planning_component("interbotix_arm")




if __name__ == '__main__':
    main()





# def plan_and_execute(
#     robot,
#     planning_component,
#     single_plan_parameters=None,
#     multi_plan_parameters=None,
# ):
#     """A helper function to plan and execute a motion."""
#     # plan to goal
#     if multi_plan_parameters is not None:
#         plan_result = planning_component.plan(
#             multi_plan_parameters=multi_plan_parameters
#         )
#     elif single_plan_parameters is not None:
#         plan_result = planning_component.plan(
#             single_plan_parameters=single_plan_parameters
#         )
#     else:
#         plan_result = planning_component.plan()

#     # execute the plan
#     if plan_result:
#         robot_trajectory = plan_result.trajectory
#         robot.execute(robot_trajectory, controllers=[])
#     else:
#         print("Planning failed")

# # set plan start state using predefined state
# panda_arm.set_start_state("ready")

# # set pose goal using predefined state
# panda_arm.set_goal_state(configuration_name = "extended")

# # plan to goal
# plan_and_execute(panda, panda_arm)



# # set plan start state using predefined state
# panda_arm.set_start_state("ready") # This conflicts with the current robot configuration and will cause an error

# # set goal using a pose message this time
# pose_goal = PoseStamped()
# pose_goal.header.frame_id = "panda_link0"
# pose_goal.pose.orientation.w = 1.0
# pose_goal.pose.position.x = 0.28
# pose_goal.pose.position.y = -0.2
# pose_goal.pose.position.z = 0.5
# panda_arm.set_goal_state(pose_stamped_msg = pose_goal, pose_link = "panda_link8")

# # plan to goal
# plan_and_execute(panda, panda_arm)

