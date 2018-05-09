/* Author: Sakshi Shrivastava 
   Email: sshriva3@uncc.edu */

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "demo_5");
	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Setup

	static const std::string PLANNING_GROUP = "right_arm";
	moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	const robot_state::JointModelGroup *joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
	
	// Visualization

	namespace rvt = rviz_visual_tools;
	moveit_visual_tools::MoveItVisualTools visual_tools("odom_combined");
	visual_tools.deleteAllMarkers();
	visual_tools.loadRemoteControl();
	Eigen::Affine3d text_pose = Eigen::Affine3d::Identity();
  	text_pose.translation().z() = 1.75; // above head of PR2
  	visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);
	visual_tools.trigger();

	// Planning

	ROS_INFO_NAMED("tutorial", "Reference frame: %s", move_group.getPlanningFrame().c_str());
	ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());


  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.2;
  move_group.setPoseTarget(target_pose1);
  visual_tools.publishAxisLabeled(target_pose1, "goal1");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	
  robot_state::RobotState start_state(*move_group.getCurrentState());


  // Dual-arm pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms.
  static const std::string PLANNING_GROUP2 = "arms";
  moveit::planning_interface::MoveGroupInterface two_arms_move_group(PLANNING_GROUP2);

  // Define two separate pose goals, one for each end-effector. Note that
  // we are reusing the goal for the right arm above
  two_arms_move_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.28;
  target_pose2.position.y = 0.7;
  target_pose2.position.z = 1.2;
  visual_tools.publishAxisLabeled(target_pose2, "goal2");

  two_arms_move_group.setPoseTarget(target_pose2, "l_wrist_roll_link");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroupInterface::Plan two_arms_plan;

  bool success = (two_arms_move_group.plan(two_arms_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 7 (dual arm plan) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishAxisLabeled(target_pose1, "goal1");
  visual_tools.publishAxisLabeled(target_pose2, "goal2");
  visual_tools.publishText(text_pose, "Two Arm Goal", rvt::WHITE, rvt::XLARGE);
  joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP2);
  visual_tools.publishTrajectoryLine(two_arms_plan.trajectory_, joint_model_group);
  visual_tools.trigger();

	ros::shutdown();
	return 0;

}
