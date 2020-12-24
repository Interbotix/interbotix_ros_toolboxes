# interbotix_ux_modules

## Overview
This package contains Python modules meant to work with any of our UFactory xArm robots. It is what allows users with minimal ROS experience to get started with our platforms. Example demo scripts that build from these modules can be found in the *python_demos* directory in any repository that depends on this one.

## Structure
For these modules to work, the launch file located in any *interbotix_XXXXX_control* package must first be started. Additionally, the desired module must also be imported in your Python script. A brief description of each module is shown below. Note that for some modules, there are two classes defined. Both have the same name, but one includes 'Interface' as well. The one without 'Interface' in it is just a collection of various 'Interface' objects which make up a robot platform. That is the one which should be imported in Python scripts. The 'Interface' one though is what actually defines the functionality for a given object. This should only be imported when designing a new robot platform.

To clarify that point, here's an example. The Arm module contains both the 'InterbotixManipulatorUX' class and the 'InterbotixArmUXInterface' class. The Interface class defines how desired movements should be converted into joint commands, and is just one of potentially three objects within the InterbotixManipulatorUX class. The other two include an instance of the InterbotixRobotUXCore class (see below) and an instance of the InterbotixGripperUXInterface class (used to control the UFactory gripper). Together, these three Interface modules (yeah - ok, the InterbotixRobotUXCore class doesn't have 'Interface' tacked on to the end of it, but that's the only exception) define what an Interbotix UFactory Manipulator *is*. It's not just something you can move around using just the Arm interface, but you can also use the Core class to get access to xArm specific functionality, or the Gripper class to grip an object.

- [mr_descriptions](src/interbotix_ux_modules/mr_descriptions.py) - contains the Screw axes (as defined in Modern Robotics by Kevin Lynch) for each Interbotix arm; these are necessary to do inverse kinematics via the Product of Exponentials approach.

- [core](src/interbotix_ux_modules/core.py) - known as *InterbotixRobotUXCore*, this is the 'base' Python module that can be used to control any UFactory xArm robot platform; it contains ROS Service clients for every ROS Service server advertised from the **xarm_drive_node** node, and subscribes to the joint states published by the **xarm_driver_node** node; every xArm module (arm, gripper) builds up from this one.

- [gripper](src/interbotix_ux_modules/gripper.py) - allows easy PWM or Current control of a UFactory xArm gripper; it contains the *InterbotixRobotUXCore* and *InterbotixGripperUXInterface* submodules.

- [arm](src/interbotix_ux_modules/arm.py) - contains an inverse kinematics solver to allow end-effector control in Cartesian space for any UFactory xArm manipulator; it contains the *InterbotixRobotUXCore*, *InterbotixManipulatorUXInterface*, and *InterbotixGripperUXInterface* submodules. To import, write `from interbotix_ux_modules.arm import InterbotixManipulatorUX` at the top of your Python script.

## Usage
As mentioned above, to use any UFactory xArm Python module, you first must start the launch file in a given *interbotix_XXXXX_control* ROS package. Only then can you run your Python script. For more info, check out the *python_demos* directory in any of our repositories.
