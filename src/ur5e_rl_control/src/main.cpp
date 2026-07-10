#include <ros/ros.h>

#include <exception>

#include "ur5e_rl_control/ur5e_execution_node.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "ur5e_execution_node");

  try {
    ur5e_rl_control::UR5eExecutionNode node(ros::NodeHandle(),
                                            ros::NodeHandle("~"));
    ros::spin();
  } catch (const std::exception& error) {
    ROS_FATAL("Failed to start UR5e execution node: %s", error.what());
    return 1;
  }
  return 0;
}
