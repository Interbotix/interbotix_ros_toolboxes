# interbotix_gravity_compensation

## Overview
This package contains the C++ implementation of the gravity compensation features of a general robotic arm.

## Gravity Compensation
### Introduction
As its name suggests, gravity compensation cancels out the effect of the gravity on a system. A toy example is the elevator. The counterweight of the elevator counteracts the gravity applied on the elevator, allowing it to move up and down effortlessly. In our case, we compensate the gravity on a robotic arm with its joint motors. This feature is useful when we use the arm as a teleoperation remote or when we teach it a specific trajectory and don't want to keep holding the arm to avoid it from collapsing.

### Formal Formulation
This is a special case of the [inverse dynamics](https://en.wikipedia.org/wiki/Inverse_dynamics) problem where the external forces are 0s. More specifically, one is interested in finding the following mapping:
$$\tau=\text{ID}(\text{model}, \mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})$$
where $\text{model}$ specifies the mechanical characteristics of the system, $\mathbf{q}$, $\dot{\mathbf{q}}$, and $\ddot{\mathbf{q}}$ are the joint positions, velocities, and accelerations.

One way to solve the inverse dynamics problem is the *recursive Newton-Euler algorithm*. It consists three steps:
1. Compute the velocities and accelerations of the links
2. Compute the forces required to produce such motions, i.e., fictitious forces
3. Compute the forces acting upon the links

Step 1: the velocity and accelerations of the link $i$ are given by the following recursive relations, where $\mathbf{v}_i$ and $\mathbf{a}_i$ are the linear velocity and acceleration of the $i$'th link, $\dot{\mathbf{q}}$ and $\ddot{\mathbf{q}}$ are the angular velocity and acceleration of the $i$'th joint connecting the two links, and $\mathbf{S}_i$ is the axis vector of the $i$'th joint.
$$\mathbf{v}_i=\mathbf{v}_{i-1}+\mathbf{S}_i\dot{\mathbf{q}}_i\text{ with }\mathbf{v}_0=\mathbf{0}$$
$$\mathbf{a}_i=\mathbf{a}_{i-1}+\mathbf{S}_i\ddot{\mathbf{q}}+\dot{\mathbf{S}}_i\dot{\mathbf{q}}\text{ with }\mathbf{a}_0=\mathbf{0}$$

Step 2: the fictitious forces $\mathbf{f}^B_i$ are associated with the inertia of the link $\mathbf{I}_i$, linear velocities $\mathbf{v}_i$, and accelerations $\mathbf{a}_i$. The symbol $\times^*$ denotes the spatial vector cross product.
$$\mathbf{f}^B_i=\mathbf{I}_i\mathbf{a}_i+\mathbf{v}_i\times^*\mathbf{I}_i\mathbf{v}_i$$

Step 3: the forces acting on a link should balance. $\mathbf{f}_{i}$ and $\mathbf{f}_{i+1}$ are the forces transmitted through the $i$'th joint and $i+1$'th joint and $\mathbf{f}^x_i$ is the external forces acting on the $i$'th link.
$$\mathbf{f}^B_i=\mathbf{f}_i+\mathbf{f}^x_i-\mathbf{f}_{i+1}$$

The resulting torque of the $i$'th joint is given by
$$\tau_i=\mathbf{S}_i^\intercal\mathbf{f}_{i}$$

For more details, please read the references in the Acknowledgement section.

## Structure
This section introduces the compositions of the package.

### Publisher
- `joint_group_pub_`: publishes the desired current commands to the `/<robot_name>/commands/joint_group` topic for the robotic arm to execute.

### Subscription
- `joint_state_sub_`: subscribes to the `/<robot_name>/joint_states` topic. When a new message arrives, it does nothing if the `gravity_compensation_enabled_` flag is set false. Otherwise, it solves for the torques required to counteract the gravity, i.e., inverse dynamics. Then, it adds padding torques depending on the directions of joint movements to ease the joint frictions. Finally, it converts the resulting torques into motor current commands and publishes it via `joint_group_pub_`.

### Service
- `gravity_compensation_enable_srv_`: hosts a service to conveniently enable/disable the gravity compensation feature. When a enable request is received, it sets the operating mode of all joints to the "current" mode. Then, it torques on the joints which support the "current" mode and torques off the others. When a disable request is received, it sets the "arm" joint group to the "position" mode and the "gripper" joint to the "current_based_position" mode. Then, it torques on all joints. All aforementioned operations are done via service calls to the `xs_sdk` node. Finally, it stores the request flags for the service callbacks indicating whether the joint is ready for gravity compensation.

### Client
- `operating_modes_client_`: sets the operating modes of the joints via service call to the `/<robot_name>/set_operating_modes` service.
- `torque_enable_client_`: torques on/off the joints via service call to the `/<robot_name>/torque_enable` service.

### Misc
- `load_motor_specs(...)`: loads the motor specs from `motor_specs.yaml` and initializes the motor specs vectors. It also initializes the operating mode and torque enable request/response flags and sets the number of joints in the 'arm' group.
- `set_operating_modes_callback(...)` and `torque_enable_callback(...)`: when the responses from the service calls return, these callbacks set the corresponding response flag. If all response flags are set true, the `gravity_compensation_enabled_` will be set true and false otherwise.
- `prepare_tree(...)`: loads the "robot_description" parameter from the `/<robot_name>/robot_state_publisher` node and parse it into a KDL [tree](https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1Tree.html) object used by the inverse dynamics solver.

## Acknowledgement
The inverse dynamics solver is ported from the [Orocos Kinematics and Dynamics Library (KDL)](https://www.orocos.org/kdl.html). Please refer to the KDL doc page for its [derivations](https://link.springer.com/book/10.1007/978-1-4899-7560-7) and [implementation](https://docs.ros.org/en/indigo/api/orocos_kdl/html/classKDL_1_1ChainIdSolver__RNE.html) details.
