# interbotix_gravity_compensation

## Overview
This package contains the C++ implementation of the gravity compensation features of a common robotic arm.

## Gravity Compensation
### Introduction
As its name suggests, gravity compensation cancels out the effect of the gravity on a system.
In our case, we compensate the gravity on a robotic arm with its joint motors.
This feature is useful when we use the arm as a teleoperation remote or when we teach it a specific trajectory and don't want to keep holding the arm to avoid it from collapsing.

### Formal Formulation
Computing the torques needed to compensate for the gravity is a special case of the [inverse dynamics](https://en.wikipedia.org/wiki/Inverse_dynamics) problem where the gravity is the only external force.
More specifically, one is interested in finding the following mapping:
$$\tau=\text{ID}(\text{model}, \mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})$$
where $\text{model}$ specifies the inertia of the system and the forces applied to it, $\mathbf{q}$, $\dot{\mathbf{q}}$, and $\ddot{\mathbf{q}}\in\Bbb{R}^6$ are the joint positions, velocities, and accelerations.

One way to solve the inverse dynamics problem is the *recursive Newton-Euler algorithm*.
It consists three steps:
1. Compute the velocities and accelerations of the links
2. Compute the forces required to produce such motions, i.e., fictitious forces
3. Compute the forces acting upon the links

Step 1: the velocity and acceleration of the $i$'th link $\mathbf{v}_i, \mathbf{a}_i\in\Bbb{S}$ are given by the following recursive relations associated with the angular velocity and acceleration of the $i$'th joint $\dot{q}_i, \ddot{q}_i\in\Bbb{R}$, and the $i$'th joint axis $\mathbf{s}_i\in\Bbb{S}$, where $\Bbb{S}$ is the set of [screws](https://en.wikipedia.org/wiki/Screw_theory).
Note that $\mathbf{s}_i$ is determined by the current joint positions $\mathbf{q}$ through [forward kinematics](https://en.wikipedia.org/wiki/Forward_kinematics).
$$\mathbf{v}_{i+1}=\mathbf{v}_{i}+\mathbf{s}_i\dot{q}_i\text{ with }\mathbf{v}_0=\mathbf{0}$$
$$\mathbf{a}_{i+1}=\mathbf{a}_{i}+\mathbf{s}_i\ddot{q}_i+\dot{\mathbf{s}}_i\dot{q}_i\text{ with }\mathbf{a}_0=\mathbf{0}$$

Step 2: the fictitious force $\mathbf{f}^B_i\in\Bbb{S}$ is associated with the inertia $\mathbf{I}_i\in\Bbb{R}^{6\times6}$, velocity $\mathbf{v}_i$, and accelerations $\mathbf{a}_i$ of the $i$'th link.
$$\mathbf{f}^B_i=\mathbf{I}_i\cdot\mathbf{a}_i+\mathbf{v}_i\times(\mathbf{I}_i\cdot\mathbf{v}_i)$$

Step 3: the forces acting on the $i$'th link should balance. $\mathbf{f}_{i}\in\Bbb{S}$ is the force applied through the $i$'th joint to the $i+1$'th link and $\mathbf{f}^x_i\in\Bbb{S}$ is the net external force acting on the $i$'th link.
$$\mathbf{f}^B_i=\mathbf{f}_{i-1}+\mathbf{f}^x_i-\mathbf{f}_{i}$$

The resulting torque of the $i$'th joint is given by
$$\tau_i=\mathbf{s}_i\cdot\mathbf{f}_{i}$$

The inverse dynamics solver used in the package is ported from the [Orocos Kinematics and Dynamics Library (KDL)](https://www.orocos.org/kdl.html).
Please refer to the KDL doc page for its [derivations](https://link.springer.com/book/10.1007/978-1-4899-7560-7) and [implementation](https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1ChainIdSolver__RNE.html) details.


## Structure
### Publisher
- `joint_group_pub_`: publishes the desired current commands to the `/<robot_name>/commands/joint_group` topic for the robotic arm to execute.

### Subscription
- `joint_state_sub_`: subscribes to the `/<robot_name>/joint_states` topic.
When a new message arrives, it does nothing if the `gravity_compensation_enabled_` flag is set false.
Otherwise, it solves for the torques required to counteract the gravity, i.e., inverse dynamics.
Then, it adds padding torques depending on the directions of joint movements to ease the joint frictions.
Finally, it converts the resulting torques into motor current commands and publishes it via `joint_group_pub_`.

### Service
- `gravity_compensation_enable_srv_`: hosts a service to conveniently enable/disable the gravity compensation feature.
When a enable request is received, it sets the operating mode of all joints to the "current" mode.
Then, it torques on the joints which support the "current" mode and torques off the others.
When a disable request is received, it sets the "arm" joint group to the "position" mode and the "gripper" joint to the "current_based_position" mode.
Then, it torques on all joints.
All aforementioned operations are done via service calls to the `xs_sdk` node.
Finally, it stores the request flags for the service callbacks indicating whether the joint requested to be ready for gravity compensation.

### Client
- `operating_modes_client_`: sets the operating modes of the joints via service call to the `/<robot_name>/set_operating_modes` service.
- `torque_enable_client_`: torques on/off the joints via service call to the `/<robot_name>/torque_enable` service.

### Misc
- `load_motor_specs(...)`: loads the motor specs from `motor_specs.yaml` and initializes the motor specs vectors.
It also initializes the operating mode and torque enable request/response flags and sets the number of joints in the 'arm' group.
- `set_operating_modes_callback(...)` and `torque_enable_callback(...)`: when the responses from the service calls return, these callbacks set the corresponding response flags.
If all response flags are set true, the `gravity_compensation_enabled_` will be set true and false otherwise.
- `prepare_tree(...)`: loads the "robot_description" parameter from the `/<robot_name>/robot_state_publisher` node and parse it into a KDL [tree](https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Tree.html) object used by the inverse dynamics solver.

## Configuration
The `motor_specs.yaml` hosts the motor specifications used in the node and provides knobs for motor assistance against joint frictions.
A template file is given below:
```
# Motor Assist: scale the no-load currents which alleviate the effects of friction
# If the values are invalid, they defaults to 0
motor_assist:
  # Set 'all' to [0, 1] to scale the no load currents of all joints uniformly
  # Or to -1 and use joint specific values
  all: -1
  # Set the joint specific values to [0, 1] to scale differently for each joint
  waist: 0.5
  shoulder: 0.5
  elbow: 0.5
  forearm_roll: 0.5
  wrist_angle: 0.5
  wrist_rotate: 0.5

motor_specs:
  waist:
    # torque constant (Nm/A): how much torque is produced per Amp of current
    torque_constant: 1.793
    # current unit (A): how much current command is needed to produce 1 Amp of current
    current_unit: 0.00269
    # no load current (A): the maximum no load current applied when motor_assist == 1
    # It should be as large as possible without the joint accelerating by itself
    no_load_current: 0.1

  shoulder:
    torque_constant: 1.793
    current_unit: 0.00269
    no_load_current: 0.0

  elbow:
    torque_constant: 1.793
    current_unit: 0.00269
    no_load_current: 0.0

  forearm_roll:
    torque_constant: 0.897
    current_unit: 0.00269
    no_load_current: 0.1

  wrist_angle:
    torque_constant: 0.897
    current_unit: 0.00269
    no_load_current: 0.0

  wrist_rotate:
    torque_constant: 0.897
    current_unit: 0.00269
    no_load_current: 0.1

# Joints specified here but not in motor_assist or motor_specs
# do not support the current control mode
joint_names:
  [waist, shoulder, elbow, forearm_roll, wrist_angle, wrist_rotate, gripper]

```
