#ifndef UR5E_RL_CONTROL_UR5E_EXECUTION_NODE_H
#define UR5E_RL_CONTROL_UR5E_EXECUTION_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include <memory>
#include <string>
#include <vector>

#include "ur5e_rl_control/trajectory_sender.h"

namespace ur5e_rl_control {

class UR5eExecutionNode {
 public:
  UR5eExecutionNode(ros::NodeHandle nh, ros::NodeHandle private_nh);

 private:
  void actionCallback(const std_msgs::Float64MultiArrayConstPtr& message);
  void jointStateCallback(const sensor_msgs::JointStateConstPtr& message);
  bool readAndValidateParameters();
  bool makeTarget(const std::vector<double>& action,
                  std::vector<double>* target) const;
  void publishVisualization(const std::vector<double>& target);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber action_subscriber_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher visualization_publisher_;
  std::unique_ptr<TrajectorySender> trajectory_sender_;

  std::vector<std::string> joint_names_;
  std::vector<double> current_positions_;
  std::vector<double> lower_limits_;
  std::vector<double> upper_limits_;
  std::string action_mode_;
  double action_scale_;
  double trajectory_duration_;
  bool clamp_to_limits_;
  bool send_to_controller_;
};

}  // namespace ur5e_rl_control

#endif  // UR5E_RL_CONTROL_UR5E_EXECUTION_NODE_H
