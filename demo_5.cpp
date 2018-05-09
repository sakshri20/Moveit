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
  target_pose1.position.y = -0.5;
  target_pose1.position.z = 0.7;
  move_group.setPoseTarget(target_pose1);
  visual_tools.publishAxisLabeled(target_pose1, "target_pose");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;

	
  robot_state::RobotState start_state(*move_group.getCurrentState());


  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Define a collision object ROS message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = move_group.getPlanningFrame();

  // The id of the object is used to identify it.
  collision_object.id = "box1";

  // Define a box to add to the world.
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.4;

  //Define a pose for the box (specified relative to frame_id)
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x = 0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z = 1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO_NAMED("tutorial", "Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Add object", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  // Sleep to allow MoveGroup to recieve and process the collision object message
  ros::Duration(1.0).sleep();

  // Now when we plan a trajectory it will avoid the obstacle
  move_group.setStartState(*move_group.getCurrentState());
  move_group.setPoseTarget(target_pose1);

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal move around cuboid) %s", success ? "" : "FAILED");

  // Visualize the plan in Rviz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Obstacle Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  visual_tools.prompt("next step");

  // Now, let's attach the collision object to the robot.
  ROS_INFO_NAMED("tutorial", "Attach the object to the robot");
  move_group.attachObject(collision_object.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object attached to robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Sleep to allow MoveGroup to recieve and process the attached collision object message */
  ros::Duration(1.0).sleep();

  // Now, let's detach the collision object from the robot.
  ROS_INFO_NAMED("tutorial", "Detach the object from the robot");
  move_group.detachObject(collision_object.id);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object dettached from robot", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Sleep to allow MoveGroup to recieve and process the detach collision object message */
  ros::Duration(1.0).sleep();

  // Now, let's remove the collision object from the world.
  ROS_INFO_NAMED("tutorial", "Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  planning_scene_interface.removeCollisionObjects(object_ids);

  // Show text in Rviz of status
  visual_tools.publishText(text_pose, "Object removed", rvt::WHITE, rvt::XLARGE);
  visual_tools.trigger();

  /* Sleep to give Rviz time to show the object is no longer there.*/
  ros::Duration(1.0).sleep();

	ros::shutdown();
	return 0;

}
