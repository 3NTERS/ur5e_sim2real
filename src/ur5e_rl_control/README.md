# install ur ros driver 换清华源安装
sudo sed -i 's|http://packages.ros.org/ros/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu|g' /etc/apt/sources.list.d/ros-latest.list
sudo apt update
sudo apt install ros-noetic-ur-robot-driver ros-noetic-universal-robots

## RViz joint visualization

Build and launch:

```bash
cd ~/ur5e_sim2real
catkin_make
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

Generator parameters are in `config/cfg.yaml`:

```yaml
generator_start_delay: 2.0
sine_frequency: 0.2
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
