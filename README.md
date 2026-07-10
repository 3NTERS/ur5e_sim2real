# UR5e Sim-to-Real RL Control

ROS Noetic package for connecting a six-dimensional reinforcement-learning action
to a UR5e trajectory controller, visualizing commands in RViz, and returning the
robot's measured joint state to the RL environment.

Only `src/ur5e_rl_control` is maintained by this repository. Other packages that may
exist in a local catkin workspace are intentionally ignored by Git.

## Control and feedback interfaces

The control path is:

```text
RL policy
  -> /rl_action (std_msgs/Float64MultiArray, 6 values)
  -> ur5e_execution_node
  -> FollowJointTrajectory
  -> UR5e controller
```

The six action values follow this fixed order:

```text
shoulder_pan_joint, shoulder_lift_joint, elbow_joint,
wrist_1_joint, wrist_2_joint, wrist_3_joint
```

The feedback path publishes `/rl_state` as an 18-dimensional
`std_msgs/Float64MultiArray`:

```text
[q1..q6, dq1..dq6, tau1..tau6]
```

Joint positions use radians, velocities use radians per second, and effort values are
those reported by the active driver (normally N·m for revolute joints).

## Build

```bash
cd ~/ur5e_sim2real
catkin_make
source devel/setup.bash
```

ROS Noetic runtime dependencies include `robot_state_publisher`, `rviz`, and the
Universal Robots driver packages. Package details and installation notes are in
[`src/ur5e_rl_control/README.md`](src/ur5e_rl_control/README.md).

## RViz-only operation

Start the command, execution, state-publication and visualization chain:

```bash
roslaunch ur5e_rl_control ur5e_rl_control.launch
```

Controller output is disabled by default. Send an absolute-position test action:

```bash
rostopic pub -1 /rl_action std_msgs/Float64MultiArray \
  "data: [0.0, -1.2, 1.4, -1.0, -1.57, 0.5]"
```

Run the built-in step or sine generator:

```bash
roslaunch ur5e_rl_control ur5e_action_test.launch signal_type:=step
roslaunch ur5e_rl_control ur5e_action_test.launch signal_type:=sine
```

## Hardware operation

After connecting and validating the UR driver, controller, joint limits, workspace,
protective stop and emergency stop, hardware output must be enabled explicitly:

```bash
roslaunch ur5e_rl_control ur5e_rl_control.launch send_to_controller:=true
```

The default action server is:

```text
/scaled_pos_joint_traj_controller/follow_joint_trajectory
```

All topics, action semantics, scaling, trajectory timing, joint limits and generator
settings are configured in `src/ur5e_rl_control/config/cfg.yaml`.

> Robot motion can cause injury or equipment damage. Validate policies in RViz and
> use conservative limits before enabling controller output.
