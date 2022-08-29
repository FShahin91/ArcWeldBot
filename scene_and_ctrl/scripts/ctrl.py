#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2013, SRI International
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of SRI International nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Author: Acorn Pooley, Mike Lautman
# Modified by: Baker Shehadeh, Faris Shahin, Mahmoud Alqannas

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

from __future__ import print_function
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

from math import pi, dist, fabs, cos, sin, radians
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

## END_SUB_TUTORIAL


def all_close(goal, actual, tolerance):
    """
    Convenience method for testing if the values in two lists are within a tolerance of each other.
    For Pose and PoseStamped inputs, the angle between the two quaternions is compared (the angle
    between the identical orientations q and -q is calculated correctly).
    @param: goal       A list of floats, a Pose or a PoseStamped
    @param: actual     A list of floats, a Pose or a PoseStamped
    @param: tolerance  A float
    @returns: bool
    """
    if type(goal) is list:
        for index in range(len(goal)):
            if abs(actual[index] - goal[index]) > tolerance:
                return False

    elif type(goal) is geometry_msgs.msg.PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

    elif type(goal) is geometry_msgs.msg.Pose:
        x0, y0, z0, qx0, qy0, qz0, qw0 = pose_to_list(actual)
        x1, y1, z1, qx1, qy1, qz1, qw1 = pose_to_list(goal)
        # Euclidean distance
        d = dist((x1, y1, z1), (x0, y0, z0))
        # phi = angle between orientations
        cos_phi_half = fabs(qx0 * qx1 + qy0 * qy1 + qz0 * qz1 + qw0 * qw1)
        return d <= tolerance and cos_phi_half >= cos(tolerance / 2.0)

    return True


class MoveGroupPython(object):
    """MoveGroupPython"""

    def __init__(self):
        super(MoveGroupPython, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        planning_frame = move_group.get_planning_frame()
        eef_link = move_group.get_end_effector_link()
        group_names = robot.get_group_names()
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self, joint_goal):
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        if len(joint_goal) != 7:
            print("Number of joints must not be 7.")
            return False
        
        joint_goal[6] = 0
        self.move_group.go(joint_goal, wait=True)
        self.move_group.stop()
        current_joints = self.move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)

    def go_to_pose_goal(self, pose_goal):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_pose
        ##
        ## Planning to a Pose Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## We can plan a motion for this group to a desired pose for the
        ## end-effector:

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        plan = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets()
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        # For testing:
        # Note that since this section of code will not be included in the tutorials
        # we use the class variable rather than the copied state variable
        current_pose = self.move_group.get_current_pose().pose
        return all_close(pose_goal, current_pose, 0.01)

def main():
    try:
        print("Initializing control...")
        ctrlgroup = MoveGroupPython()
        print("Moving robot to home postion...")
        joint_target = [0, -pi/2, pi/2, 0, pi/2, pi, 0]
        if not ctrlgroup.go_to_joint_state(joint_target):
            print("WARNING: Current pose does not match the plan")

        print("Moving robot to position...")
        joint_target = ctrlgroup.move_group.get_current_joint_values()
        
        joint_target[0] = radians(-14)
        joint_target[1] = radians(-70)
        joint_target[2] = radians(97)
        joint_target[3] = radians(19)
        joint_target[4] = radians(80)
        joint_target[5] = radians(170)
        if not ctrlgroup.go_to_joint_state(joint_target):
            print("WARNING: Current pose does not match the plan")

        targetPose = geometry_msgs.msg.Pose()
        tempPose = geometry_msgs.msg.Pose()
        currentPose = ctrlgroup.move_group.get_current_pose()
        targetPose = copy.deepcopy(currentPose.pose)
        tempPose = copy.deepcopy(currentPose.pose)
        waypoints = []
        
        print("Welding first circle...")
        for angle in range (0, 360):
            x = (0.1 * cos(radians(angle))) - 0.1
            y = 0.1 * sin(radians(angle))
            targetPose.position.x = tempPose.position.x - x
            targetPose.position.y = tempPose.position.y - y
            waypoints.append(copy.deepcopy(targetPose))
        
        (plan, fraction) = ctrlgroup.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        ctrlgroup.move_group.execute(plan, wait=True)
        print("Welding done.")
        print("Welding second circle...")
        waypoints.clear()
        for angle in range (0, 360):           
            x = 0.1 - (0.1 * cos(radians(angle)))
            y = 0.1 * sin(radians(angle))
            targetPose.position.x = tempPose.position.x - x
            targetPose.position.y = tempPose.position.y - y
            waypoints.append(copy.deepcopy(targetPose))
        
        (plan, fraction) = ctrlgroup.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        ctrlgroup.move_group.execute(plan, wait=True)
        print("Welding done.")
        ########################################################
        print("Moving arm to new position...")
        joint_target = ctrlgroup.move_group.get_current_joint_values()
        joint_target[0] = radians(159)
        joint_target[1] = radians(-152)
        joint_target[2] = radians(-53)
        joint_target[3] = radians(-7)
        joint_target[4] = radians(106)
        joint_target[5] = radians(-8)

        if not ctrlgroup.go_to_joint_state(joint_target):
            print("WARNING: Current pose does not match the plan")
        print("Robot arm in postion.")
        targetPose = geometry_msgs.msg.Pose()
        tempPose = geometry_msgs.msg.Pose()
        currentPose = ctrlgroup.move_group.get_current_pose()

        targetPose = copy.deepcopy(currentPose.pose)
        tempPose = copy.deepcopy(currentPose.pose)
        waypoints.clear()
        print("Welding a cosine track...")
        for angle in range (0, 720):
            y = tempPose.position.y - 0.1 * cos(radians(angle))
            x = targetPose.position.x - 0.001
            targetPose.position.x =  x
            targetPose.position.y =  y
            waypoints.append(copy.deepcopy(targetPose))

        (plan, fraction) = ctrlgroup.move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0)
        ctrlgroup.move_group.execute(plan, wait=True)
        print("Welding done.")
        print("End of planning.")
        
        
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()