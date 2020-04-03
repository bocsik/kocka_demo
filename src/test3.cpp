#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(4);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "panda_link0";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.3;
  collision_objects[0].primitives[0].dimensions[1] = 0.6;
  collision_objects[0].primitives[0].dimensions[2] = 0;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.4;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;



  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[1].header.frame_id = "panda_link0";
  collision_objects[1].id = "object1";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.045;
  collision_objects[1].primitives[0].dimensions[1] = 0.045;
  collision_objects[1].primitives[0].dimensions[2] = 0.045;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.3;
  collision_objects[1].primitive_poses[0].position.y = -0.2;
  collision_objects[1].primitive_poses[0].position.z = 0.0225;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "object2";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.045;
  collision_objects[2].primitives[0].dimensions[1] = 0.045;
  collision_objects[2].primitives[0].dimensions[2] = 0.045;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = -0.2;
  collision_objects[2].primitive_poses[0].position.z = 0.0225;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  collision_objects[3].header.frame_id = "panda_link0";
  collision_objects[3].id = "object3";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = 0.045;
  collision_objects[3].primitives[0].dimensions[1] = 0.045;
  collision_objects[3].primitives[0].dimensions[2] = 0.045;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.4;
  collision_objects[3].primitive_poses[0].position.y = 0.2;
  collision_objects[3].primitive_poses[0].position.z = 0.0225;
  // END_SUB_TUTORIAL

  collision_objects[3].operation = collision_objects[3].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
  visual_tools.deleteAllMarkers();



  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface move_group("panda_arm");

  const robot_state::JointModelGroup* joint_model_group =
     move_group.getCurrentState()->getJointModelGroup("panda_arm");
  move_group.setPlanningTime(45.0);

  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(3.0).sleep();

  tf2::Quaternion orientation;
  orientation.setRPY(0, M_PI, 3*M_PI / 4);

  geometry_msgs::Pose target_pose1;

  target_pose1.orientation.x = orientation[0];
  target_pose1.orientation.y = orientation[1];
  target_pose1.orientation.z = orientation[2];
  target_pose1.orientation.w = orientation[3];

  target_pose1.position.x = 0.3;
  target_pose1.position.y = -0.2;
  target_pose1.position.z = 0.135;

  move_group.setPoseTarget(target_pose1);


  // Now, we call the planner to compute the plan and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // We can also visualize the plan as a line with markers in RViz.
  /*ROS_INFO_NAMED("tutorial", "Visualizing plan 1 as trajectory line");
  visual_tools.publishAxisLabeled(target_pose1, "pose1");
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);*/

  move_group.move();

  move_group.attachObject("object1");
  ros::WallDuration(0.5).sleep();

  geometry_msgs::Pose target_pose2;

  target_pose2.orientation.x = orientation[0];
  target_pose2.orientation.y = orientation[1];
  target_pose2.orientation.z = orientation[2];
  target_pose2.orientation.w = orientation[3];


  target_pose2.position.x = 0.4;
  target_pose2.position.y = 0;
  target_pose2.position.z = 0.135;

  move_group.setPoseTarget(target_pose2);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan2;

  success = (move_group.plan(my_plan2) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  move_group.detachObject("object1");

  geometry_msgs::Pose target_pose3;

  target_pose3.orientation.x = orientation[0];
  target_pose3.orientation.y = orientation[1];
  target_pose3.orientation.z = orientation[2];
  target_pose3.orientation.w = orientation[3];


  target_pose3.position.x = 0.5;
  target_pose3.position.y = -0.2;
  target_pose3.position.z = 0.135;

  move_group.setPoseTarget(target_pose3);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan3;

  success = (move_group.plan(my_plan3) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  move_group.attachObject("object2");
  ros::WallDuration(0.5).sleep();

  geometry_msgs::Pose target_pose4;

  target_pose4.orientation.x = orientation[0];
  target_pose4.orientation.y = orientation[1];
  target_pose4.orientation.z = orientation[2];
  target_pose4.orientation.w = orientation[3];


  target_pose4.position.x = 0.4;
  target_pose4.position.y = 0;
  target_pose4.position.z = 0.18; //0.135+0.045

  move_group.setPoseTarget(target_pose4);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan4;

  success = (move_group.plan(my_plan4) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 4 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  move_group.detachObject("object2");



  geometry_msgs::Pose target_pose5;

  target_pose5.orientation.x = orientation[0];
  target_pose5.orientation.y = orientation[1];
  target_pose5.orientation.z = orientation[2];
  target_pose5.orientation.w = orientation[3];


  target_pose5.position.x = 0.4;
  target_pose5.position.y = 0.2;
  target_pose5.position.z = 0.135;

  move_group.setPoseTarget(target_pose5);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan5;

  success = (move_group.plan(my_plan5) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 5 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  move_group.attachObject("object3");
  ros::WallDuration(0.5).sleep();


  geometry_msgs::Pose target_pose6;

  target_pose6.orientation.x = orientation[0];
  target_pose6.orientation.y = orientation[1];
  target_pose6.orientation.z = orientation[2];
  target_pose6.orientation.w = orientation[3];


  target_pose6.position.x = 0.4;
  target_pose6.position.y = 0;
  target_pose6.position.z = 0.24;

  move_group.setPoseTarget(target_pose6);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan6;

  success = (move_group.plan(my_plan6) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan 6 (pose goal) %s", success ? "" : "FAILED");

  move_group.move();

  move_group.detachObject("object3");

  ros::WallDuration(1.0).sleep();

  //place(group);

  ros::waitForShutdown();
  return 0;
}
