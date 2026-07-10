#ifndef UR5E_RL_CONTROL_TRAJECTORY_SENDER_H
#define UR5E_RL_CONTROL_TRAJECTORY_SENDER_H

#include <actionlib/client/simple_action_client.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <ros/duration.h>

#include <string>
#include <vector>

namespace ur5e_rl_control {

class TrajectorySender {
 public:
  TrajectorySender(const std::string& action_name,
                   const std::vector<std::string>& joint_names,
                   double start_delay);

  bool waitForServer(const ros::Duration& timeout);
  bool isServerConnected();
  bool sendPositions(const std::vector<double>& positions,
                     const ros::Duration& movement_duration);
  void cancel();

 private:
  using Client =
      actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const control_msgs::FollowJointTrajectoryResultConstPtr& result);

  Client client_;
  std::vector<std::string> joint_names_;
  double start_delay_;
};

}  // namespace ur5e_rl_control

#endif  // UR5E_RL_CONTROL_TRAJECTORY_SENDER_H
