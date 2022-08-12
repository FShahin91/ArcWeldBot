/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta, Michael Lautman */
/* Modified by: Faris Shahin*/

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/ApplyPlanningScene.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "weldbot_scene");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  moveit_visual_tools::MoveItVisualTools visual_tools("base_link");
  visual_tools.deleteAllMarkers();
  
  ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
  ros::WallDuration sleep_t(0.5);
  while (planning_scene_diff_publisher.getNumSubscribers() < 1)
  {
    sleep_t.sleep();
  }

  moveit_msgs::AttachedCollisionObject cylinder_1, cylinder_2;
  cylinder_1.link_name = "world";
  cylinder_1.object.header.frame_id = "world";
  cylinder_1.object.id = "l_cylinder";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.position.x = 1.0;
  pose.position.y = 0.41;
  pose.position.z = 0.20;
  pose.orientation.w = 0.706825;
  pose.orientation.x = 0.707388;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  
  /* Define a cylinder to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.CYLINDER;
  primitive.dimensions.resize(2);
  primitive.dimensions[primitive.CYLINDER_HEIGHT]= 0.5;
  primitive.dimensions[primitive.CYLINDER_RADIUS]= 0.05; 

  cylinder_1.object.primitives.push_back(primitive);
  cylinder_1.object.primitive_poses.push_back(pose);
  cylinder_1.object.operation = cylinder_1.object.ADD;

  // Add an object into the environment

  ROS_INFO("Adding first pipe into the world.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(cylinder_1.object);

  cylinder_2.link_name = "world";
  cylinder_2.object.header.frame_id = "world";
  cylinder_2.object.id = "r_cylinder";
  pose.position.y = -0.1;

  cylinder_2.object.primitives.push_back(primitive);
  cylinder_2.object.primitive_poses.push_back(pose);
  cylinder_2.object.operation = cylinder_2.object.ADD;
  ROS_INFO("Adding second pipe into the world.");
  planning_scene.world.collision_objects.push_back(cylinder_2.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);


  ros::ServiceClient planning_scene_diff_client =
      node_handle.serviceClient<moveit_msgs::ApplyPlanningScene>("apply_planning_scene");
  planning_scene_diff_client.waitForExistence();
  moveit_msgs::ApplyPlanningScene srv;
  srv.request.scene = planning_scene;
  planning_scene_diff_client.call(srv);

  ros::shutdown();
  return 0;
}