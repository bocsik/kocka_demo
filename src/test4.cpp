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

#include <map>

#include "demo_common.h"


void pick(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string object)
{
  std::vector<std::string> names = planning_scene_interface.getKnownObjectNames (false);

  std::map< std::string,geometry_msgs::Pose > cubes = planning_scene_interface.getObjectPoses(names);

  geometry_msgs::Pose cube_pose = cubes.find(object)->second; // todo: ide kell hibakezelés ha rossz az objet név

  //fölé állás:

  tf2::Quaternion orientation;
  orientation.setRPY(0, M_PI, 3*M_PI / 4);

  geometry_msgs::Pose pre_grasp_pose;

  pre_grasp_pose.position.x = cube_pose.position.x;
  pre_grasp_pose.position.y = cube_pose.position.y;
  pre_grasp_pose.position.z = cube_pose.position.z + 0.22;

  pre_grasp_pose.orientation.x = orientation[0];
  pre_grasp_pose.orientation.y = orientation[1];
  pre_grasp_pose.orientation.z = orientation[2];
  pre_grasp_pose.orientation.w = orientation[3];

  move_group.setPoseTarget(pre_grasp_pose);

  moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;

  bool success = (move_group.plan(pre_grasp_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan fole megy (pose goal) %s", success ? "" : "FAILED");

  move_group.move();


  //lineárisan le:

  std::vector<geometry_msgs::Pose> aproach;
  //aproach.push_back(pre_grasp_pose);

  geometry_msgs::Pose grasp_pose;

  grasp_pose.position.x = cube_pose.position.x;
  grasp_pose.position.y = cube_pose.position.y;
  grasp_pose.position.z = cube_pose.position.z + 0.11;

  grasp_pose.orientation.x = orientation[0];
  grasp_pose.orientation.y = orientation[1];
  grasp_pose.orientation.z = orientation[2];
  grasp_pose.orientation.w = orientation[3];

  aproach.push_back(grasp_pose);

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory aproach_trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double aproach_fraction = move_group.computeCartesianPath(aproach, eef_step, jump_threshold, aproach_trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan lemegy (Cartesian path) (%.2f%% acheived)", aproach_fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan aproach_plan;

  aproach_plan.trajectory_ = aproach_trajectory;

  move_group.execute(aproach_plan);


  ros::WallDuration(1.0).sleep();


  move_group.attachObject(object); //megfogás


  //lineárisan fel:

  std::vector<geometry_msgs::Pose> retreat;
  //retreat.push_back(grasp_pose);
  retreat.push_back(pre_grasp_pose); //lehetne egy külön post_grasp_pose-t is definiálni ha kell

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory retreat_trajectory;
  double retreat_fraction = move_group.computeCartesianPath(retreat, eef_step, jump_threshold, retreat_trajectory);
  ROS_INFO_NAMED("demo", "Visualizing plan felmegy (Cartesian path) (%.2f%% acheived)", retreat_fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;

  retreat_plan.trajectory_ = retreat_trajectory;

  move_group.execute(retreat_plan);

}


void place(moveit::planning_interface::MoveGroupInterface& move_group,moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, std::string object, geometry_msgs::Pose place_pose)
{

  static int height = 0;

  std::map< std::string,moveit_msgs::AttachedCollisionObject > attached_cubes = planning_scene_interface.getAttachedObjects({object});

  if(attached_cubes.empty())
  {
    ROS_WARN_NAMED("demo", "No object attached, nothing to place");
    return;
  }

  //fölé állás:


  geometry_msgs::Pose pre_place_pose;

  pre_place_pose.position.x = place_pose.position.x;
  pre_place_pose.position.y = place_pose.position.y;
  pre_place_pose.position.z = place_pose.position.z + height * 0.045 + 0.22;

  pre_place_pose.orientation.x = place_pose.orientation.x;
  pre_place_pose.orientation.y = place_pose.orientation.y;
  pre_place_pose.orientation.z = place_pose.orientation.z;
  pre_place_pose.orientation.w = place_pose.orientation.w;

  move_group.setPoseTarget(pre_place_pose);

  moveit::planning_interface::MoveGroupInterface::Plan pre_place_plan;

  bool success = (move_group.plan(pre_place_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  ROS_INFO_NAMED("tutorial", "Visualizing plan fole megy (pose goal) %s", success ? "" : "FAILED");

  move_group.move();


  //lineárisan le:

  std::vector<geometry_msgs::Pose> aproach;

  geometry_msgs::Pose grasp_pose;

  place_pose.position.z += 0.135 + 0.046 * height;

  aproach.push_back(place_pose);

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory aproach_trajectory;
  const double jump_threshold = 0.0;
  const double eef_step = 0.01;
  double aproach_fraction = move_group.computeCartesianPath(aproach, eef_step, jump_threshold, aproach_trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan lemegy (Cartesian path) (%.2f%% acheived)", aproach_fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan aproach_plan;

  aproach_plan.trajectory_ = aproach_trajectory;

  move_group.execute(aproach_plan);


  ros::WallDuration(1.0).sleep();


  move_group.detachObject(object); //elengedés


  //lineárisan fel:

  std::vector<geometry_msgs::Pose> retreat;
  //retreat.push_back(place_pose);
  retreat.push_back(pre_place_pose); //lehetne egy külön post_grasp_pose-t is definiálni ha kell

  move_group.setMaxVelocityScalingFactor(0.1);

  moveit_msgs::RobotTrajectory retreat_trajectory;
  double retreat_fraction = move_group.computeCartesianPath(retreat, eef_step, jump_threshold, retreat_trajectory);
  ROS_INFO_NAMED("tutorial", "Visualizing plan felmegy (Cartesian path) (%.2f%% acheived)", retreat_fraction * 100.0);

  moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;

  retreat_plan.trajectory_ = retreat_trajectory;

  move_group.execute(retreat_plan);

  height++;

}


void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface)
{

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
  collision_objects[1].id = "cube1";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = CUBE_SIZE;
  collision_objects[1].primitives[0].dimensions[1] = CUBE_SIZE;
  collision_objects[1].primitives[0].dimensions[2] = CUBE_SIZE;

  /* Define the pose of the object. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0.3;
  collision_objects[1].primitive_poses[0].position.y = -0.2;
  collision_objects[1].primitive_poses[0].position.z = CUBE_SIZE / 2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  collision_objects[2].header.frame_id = "panda_link0";
  collision_objects[2].id = "cube2";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[2].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = CUBE_SIZE;
  collision_objects[2].primitives[0].dimensions[1] = CUBE_SIZE;
  collision_objects[2].primitives[0].dimensions[2] = CUBE_SIZE;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = -0.2;
  collision_objects[2].primitive_poses[0].position.z = CUBE_SIZE / 2;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  collision_objects[3].header.frame_id = "panda_link0";
  collision_objects[3].id = "cube3";

  /* Define the primitive and its dimensions. */
  collision_objects[3].primitives.resize(1);
  collision_objects[3].primitives[0].type = collision_objects[3].primitives[0].BOX;
  collision_objects[3].primitives[0].dimensions.resize(3);
  collision_objects[3].primitives[0].dimensions[0] = CUBE_SIZE;
  collision_objects[3].primitives[0].dimensions[1] = CUBE_SIZE;
  collision_objects[3].primitives[0].dimensions[2] = CUBE_SIZE;

  /* Define the pose of the object. */
  collision_objects[3].primitive_poses.resize(1);
  collision_objects[3].primitive_poses[0].position.x = 0.4;
  collision_objects[3].primitive_poses[0].position.y = 0.2;
  collision_objects[3].primitive_poses[0].position.z = CUBE_SIZE / 2;
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

  geometry_msgs::Pose place_pose; //ez majd legyen globális konstans

  tf2::Quaternion orientation;
  orientation.setRPY(0, M_PI, 3*M_PI / 4);

  place_pose.position.x = 0.4;
  place_pose.position.y = 0;
  place_pose.position.z = 0;

  place_pose.orientation.x = orientation[0];
  place_pose.orientation.y = orientation[1];
  place_pose.orientation.z = orientation[2];
  place_pose.orientation.w = orientation[3];



  pick(move_group, planning_scene_interface, "cube1");
  place(move_group, planning_scene_interface, "cube1", place_pose);

  pick(move_group, planning_scene_interface, "cube2");
  place(move_group, planning_scene_interface, "cube2", place_pose);

  pick(move_group, planning_scene_interface, "cube3");
  place(move_group, planning_scene_interface, "cube3", place_pose);

  ros::shutdown();

  return 0;



}
