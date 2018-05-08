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
	ros::init(argc, argv, "demo_1");
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

	// Planning to a pose goal

	geometry_msgs::Pose target_pose1;
  	target_pose1.orientation.w = 1.0;
  	target_pose1.position.x = 0.28;
  	target_pose1.position.y = -0.7;
  	target_pose1.position.z = 1.0;
  	move_group.setPoseTarget(target_pose1);
	

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
	ROS_INFO_NAMED("tutorial", "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

	ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");

	visual_tools.publishAxisLabeled(target_pose1, "pose1");
  	visual_tools.publishText(text_pose, "Pose Goal", rvt::WHITE, rvt::XLARGE);
  	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  	visual_tools.trigger();
  	visual_tools.prompt("next step");

	ros::shutdown();
	return 0;

}
