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

	geometry_msgs::Pose target_pose1;
  	target_pose1.orientation.w = 1.0;
  	target_pose1.position.x = 0.28;
  	target_pose1.position.y = -0.5;
 	target_pose1.position.z = 0.5;
  	move_group.setPoseTarget(target_pose1);

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	// Planning with Path Constraints
  	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  	//
  	// Path constraints can easily be specified for a link on the robot.
  	// Let's specify a path constraint and a pose goal for our group.
  	// First define the path constraint.
  	moveit_msgs::OrientationConstraint ocm;
  	ocm.link_name = "r_wrist_roll_link";
  	ocm.header.frame_id = "base_link";
  	ocm.orientation.w = 0.5;
  	ocm.absolute_x_axis_tolerance = 0.1;
  	ocm.absolute_y_axis_tolerance = 0.1;
  	ocm.absolute_z_axis_tolerance = 0.1;
  	ocm.weight = 1.0;

  	// Now, set it as the path constraint for the group.
  	moveit_msgs::Constraints test_constraints;
  	test_constraints.orientation_constraints.push_back(ocm);
  	move_group.setPathConstraints(test_constraints);

  	// We will reuse the old goal that we had and plan to it.
  	// Note that this will only work if the current state already
  	// satisfies the path constraints. So, we need to set the start
  	// state to a new pose.
  	robot_state::RobotState start_state(*move_group.getCurrentState());
  	geometry_msgs::Pose start_pose2;
  	start_pose2.orientation.w = 1.0;
  	start_pose2.position.x = 0.05;
  	start_pose2.position.y = 0.5;
  	start_pose2.position.z = 0.5;
  	start_state.setFromIK(joint_model_group, start_pose2);
  	move_group.setStartState(start_state);

  	// Now we will plan to the earlier pose target from the new
  	// start state that we have just created.
  	move_group.setPoseTarget(target_pose1);

  	// Planning with constraints can be slow because every sample must call an inverse kinematics solver.
  	// Lets increase the planning time from the default 5 seconds to be sure the planner has enough time to succeed.
  	move_group.setPlanningTime(10.0);

  	bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  	ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

  	// Visualize the plan in Rviz
  	visual_tools.deleteAllMarkers();
  	visual_tools.publishAxisLabeled(start_pose2, "start");
  	visual_tools.publishAxisLabeled(target_pose1, "goal");
  	visual_tools.publishText(text_pose, "Constrained Goal", rvt::WHITE, rvt::XLARGE);
  	visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  	visual_tools.trigger();
  	visual_tools.prompt("next step");

  	// When done with the path constraint be sure to clear it.
  	move_group.clearPathConstraints();

	ros::shutdown();
	return 0;

}
