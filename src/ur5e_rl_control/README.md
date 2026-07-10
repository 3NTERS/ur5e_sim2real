# install ur ros driver 换清华源安装
sudo sed -i 's|http://packages.ros.org/ros/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu|g' /etc/apt/sources.list.d/ros-latest.list
sudo apt update
sudo apt install ros-noetic-ur-robot-driver ros-noetic-universal-robots

## RViz joint visualization

Build and launch:

```bash
cd ~/ur5e_sim2real
catkin_make -j4
source devel/setup.bash
roslaunch ur5e_rl_control ur5e_rl_control.launch
```

Send a six-joint command (positions are radians):

```bash
rostopic pub -1 /ur5e/joint_command sensor_msgs/JointState \
  "{name: ['shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'], position: [0.0, -1.2, 1.4, -1.0, -1.57, 0.5]}"
```

Send an RL action directly (six absolute joint positions by default):

```bash
rostopic pub -1 /rl_action std_msgs/Float64MultiArray \
  "data: [0.0, -1.2, 1.4, -1.0, -1.57, 0.5]"
```

The default `send_to_controller: false` only moves the RViz model. After the UR
driver and trajectory controller are connected and safety has been checked, enable
hardware output explicitly:

```bash
roslaunch ur5e_rl_control ur5e_rl_control.launch send_to_controller:=true
```

`action_mode: position` treats actions as radians. `action_mode: delta` adds each
scaled action to the latest `/joint_states` position. Both modes apply the configured
joint limits before publishing or sending a trajectory.

## RL state feedback

`ur5e_execution_node` subscribes to the actual `/joint_states`, reorders every field
to the configured UR5e joint order, and publishes `/rl_state` as an 18-element
`std_msgs/Float64MultiArray`:

```text
[q1, q2, q3, q4, q5, q6,
 dq1, dq2, dq3, dq4, dq5, dq6,
 tau1, tau2, tau3, tau4, tau5, tau6]
```

Here `q` is joint position in rad, `dq` is joint velocity in rad/s, and `tau` is the
driver-reported `JointState.effort` (normally N·m for revolute joints). Its exact
physical meaning depends on the active UR driver/controller. Inspect it with:

```bash
rostopic echo /rl_state
rostopic hz /rl_state
```

The feedback rate follows the incoming `/joint_states` rate. Frames missing a required
joint or position are rejected. Missing/non-finite velocity or effort entries are
reported and replaced by zero so the RL observation always remains 18-dimensional.

## Automatic action generator

Use the dedicated test launch to check the complete action-to-trajectory path. It
starts in RViz-only mode and publishes the baseline pose for two seconds before the
selected test signal begins.

Step input:

```bash
roslaunch ur5e_rl_control ur5e_action_test.launch signal_type:=step
```

Sine input:

```bash
roslaunch ur5e_rl_control ur5e_action_test.launch signal_type:=sine
```

Delta-mode sine input:

```bash
roslaunch ur5e_rl_control ur5e_action_test.launch \
  signal_type:=sine action_mode:=delta
```

In delta mode the generator subscribes to `/rl_state`, extracts its first six
position values, and publishes the element-wise normalized position error:

```text
action[i] = (desired_position[i] - actual_position[i]) / action_scale
```

Before the signal starts, `desired_position` is the configured baseline. During the
test it becomes `baseline + amplitude * wave`. The generator waits for a valid
18-dimensional state before publishing and clamps each test action to
`generator_max_delta_action` by default. It never subtracts Python lists directly.

Generator parameters are in `config/cfg.yaml`:

```yaml
generator_start_delay: 2.0
sine_frequency: 0.2
generator_max_delta_action: 1.0
generator_baseline: [0.0, -1.5708, 1.5708, -1.5708, -1.5708, 0.0]
generator_amplitude: [0.35, 0.0, 0.0, 0.0, 0.0, 0.0]
```

The example excites only `shoulder_pan_joint`; assign nonzero amplitudes to other
joints to test them. Values are radians and sine frequency is Hz. Only after checking
the range, environment and robot safety should controller output be enabled:

```bash
roslaunch ur5e_rl_control ur5e_action_test.launch \
  signal_type:=sine send_to_controller:=true
```

The bridge also accepts a `JointState` with an empty `name` array when `position`
contains exactly six values in the order shown above. To use another command topic:

```bash
roslaunch ur5e_rl_control ur5e_rl_control.launch input_topic:=/my_joint_command
```

## Isaac Gym `.pth` policy inference

`scripts/ur5e_traj_bridge.py` loads a trusted PyTorch policy, subscribes to the
18-dimensional `/rl_state`, and publishes its six-dimensional output to `/rl_action`.
Start in RViz-only mode:

```bash
catkin_make
source devel/setup.bash
roslaunch ur5e_rl_control ur5e_policy.launch \
  policy_path:=/absolute/path/to/policy.pth \
  policy_device:=cpu
```

The loader accepts, in order:

1. TorchScript saved with `torch.jit.save` (recommended for deployment).
2. A checkpoint containing a complete `torch.nn.Module`.
3. An actor state_dict from common Isaac Gym training layouts.

For a pure state_dict, actor layers are auto-detected when their keys contain
`actor`, `policy`, or `mu`. If detection is ambiguous, specify the exact linear-layer
base keys in execution order:

```yaml
policy_state_dict_key: model_state_dict
policy_layer_keys:
  - actor.0
  - actor.2
  - actor.4
  - actor.6
```

The node logs every detected layer key. A chain must map
`policy_observation_dim: 18` to `policy_action_dim: 6`.

The policy input is formed as:

```text
observation[i] =
  (rl_state[policy_observation_indices[i]] - policy_observation_offset[i])
  * policy_observation_scale[i]
```

These indices, offsets, scales, clipping values, action scale, action semantics and
joint ordering must exactly match training. The default identity mapping consumes
`[q(6), dq(6), tau(6)]`. A policy trained with relative joint angles, commands,
previous actions or other observations cannot use that default unchanged.

Typical normalized delta-action deployment uses matching settings such as:

```yaml
action_mode: delta
action_scale: 0.05
policy_action_clip: 1.0
```

Policy publication stops when `/rl_state` is stale or invalid. Controller output is
still disabled by default. Only after checking `/rl_state`, normalized observations
and `/rl_action` in RViz should hardware output be explicitly enabled.

> Load only `.pth` files you trust. Python pickle-based PyTorch checkpoints can
> execute code while loading. Model files are ignored by Git in this repository.

# 
ur5e_rl_control/
├──asset/
|  ├──ur5e.urdf
├──include/
│   └── ur5e_rl_control/
│       ├── ur5e_execution_node.h
│       ├── safety_filter.h
│       ├── trajectory_sender.h
│       ├── rl_command.h
│       └── ur5e_limits.h
├── src/
│   ├── ur5e_execution_node.cpp
│   ├── safety_filter.cpp
│   ├── trajectory_sender.cpp
│   └── main.cpp
├── scripts/
│   └── policy_inference_node.py
├── launch/
│   └── ur5e_rl_control.launch
├── config/
│   └── ur5e_control.yaml
├── CMakeLists.txt
└── package.xml
