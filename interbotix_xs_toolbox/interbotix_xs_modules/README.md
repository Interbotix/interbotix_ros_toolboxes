# interbotix_xs_modules

## Overview
This package contains Python modules meant to work with any of our Interbotix X-Series platforms. It is what allows users with minimal ROS experience to get started with our platforms. Example demo scripts that build from these modules can be found in the *python_demos* directory in any repository that depends on this one.

## Structure
For these modules to work, the launch file located in any *interbotix_XXXXX_control* package must first be started. Additionally, the desired module must also be imported in your Python script. A brief description of each module is shown below. Note that for some modules, there are two classes defined. Both have the same name, but one includes 'Interface' as well. The one without 'Interface' in it is just a collection of various 'Interface' objects which make up a robot platform. That is the one which should be imported in Python scripts. The 'Interface' one though is what actually defines the functionality for a given object. This should only be imported when designing a new robot platform.

To clarify that point, here's an example. The Hexapod module contains both the 'InterbotixHexapodXS' class and the 'InterbotixHexapodXSInterface' class. The Interface class defines how desired movements should be converted into joint commands, and is just one of three objects within the InterbotixHexapodXS class. The other two include an instance of the InterbotixRobotXSCore class (see below) and an instance of the InterbotixRpiPixelInterface class (used to control the NeoPixels on an RPi4 board). Together, these three Interface modules (yeah - ok, the InterbotixRobotXSCore class doesn't have 'Interface' tacked on to the end of it, but that's the only exception) define what an Interbotix X-Series Hexapod *is*. It's not just something you can walk around using just the Hexpod interface, but you can also use the Core class to get access to individual or groups of motors, or the Pixel class to flash the LEDs.

- [mr_descriptions](src/interbotix_xs_modules/mr_descriptions.py) - contains the Screw axes (as defined in Modern Robotics by Kevin Lynch) for each Interbotix arm; these are necessary to do inverse kinematics via the Product of Exponentials approach.

- [core](src/interbotix_xs_modules/core.py) - known as *InterbotixRobotXSCore*, this is the 'base' Python module that can be used to control any X-Series robot platform; it contains ROS Service clients for every ROS Service server advertised from the **xs_sdk** node, subscribes to the joint states published by the **xs_sdk** node, and has a ROS publisher interface for each topic the **xs_sdk** node subscribes to; every X-Series module (arm, gripper, hexapod, turret, locobot) builds up from this one.

- [gripper](src/interbotix_xs_modules/gripper.py) - allows easy PWM or Current control of an Interbotix X-Series gripper; it contains the *InterbotixRobotXSCore* and *InterbotixGripperXSInterface* submodules.

- [arm](src/interbotix_xs_modules/arm.py) - contains an inverse kinematics solver to allow end-effector control in Cartesian space for any Interbotix X-Series manipulator; it contains the *InterbotixRobotXSCore*, *InterbotixManipulatorXSInterface*, and *InterbotixGripperXSInterface* submodules. To import, write `from interbotix_xs_modules.arm import InterbotixManipulatorXS` at the top of your Python script.

- [turret](src/interbotix_xs_modules/turret.py) - contains a small API that simplifies controlling any Interbotix X-Series Turret platform; it contains the *InterbotixRobotXSCore* and *InterbotixTurretXSInterface* submodules. To import, write `from interbotix_xs_modules.turret import InterbotixTurretXS` at the top of your Python script.

- [locobot](src/interbotix_xs_modules/locobot.py) - a locobot is composed of a turret (to control the camera motion), a Kobuki base, and potentially an arm and gripper. As such, it contains the *InterbotixRobotXSCore*, *InterbotixTurretXSInterface*, *InterbotixManipulatorXSInterface*, *InterbotixGripperXSInterface*, and the *InterbotixKobukiInterface* submodules. To import, write `from interbotix_xs_modules.locobot import InterbotixLocobotXS` at the top of your Python script.

- [hexapod](src/interbotix_xs_modules/hexapod.py) - contains inverse kinematics and gait solvers to move any Interbotix X-Series Hexapod; it contains the *InterbotixRobotXSCore*, *InterbotixHexapodXSInterface*, and *InterbotixRpiPixelInterface* (see the *interbotix_rpi_modules* ROS package for details) submodules. To import, write `from interbotix_xs_modules.hexapod import InterbotixHexapodXS` at the top of your Python script.

## Usage
As mentioned above, to use any X-Series Python module, you first must start the launch file in a given *interbotix_XXXXX_control* ROS package. Only then can you run your Python script. For more info, check out the *python_demos* directory in any of our repositories.
