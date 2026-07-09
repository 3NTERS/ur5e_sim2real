# install ur ros driver 换清华源安装
sudo sed -i 's|http://packages.ros.org/ros/ubuntu|https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu|g' /etc/apt/sources.list.d/ros-latest.list
sudo apt update
sudo apt install ros-noetic-ur-robot-driver ros-noetic-universal-robots

# 
ur5e_rl_control/
├── include/
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