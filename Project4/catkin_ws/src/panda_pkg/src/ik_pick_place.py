#!/usr/bin/env python3

# Software License Agreement (BSD License)
#
# Copyright (c) 2021, Aswath Muthuselvam
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

## BEGIN_SUB_TUTORIAL imports
##
## To use the Python MoveIt interfaces, we will import the `moveit_commander`_ namespace.
## This namespace provides us with a `MoveGroupCommander`_ class, a `PlanningSceneInterface`_ class,
## and a `RobotCommander`_ class. More on these below. We also import `rospy`_ and some messages that we will use:
##

# Python 2/3 compatibility imports
from __future__ import print_function
import quaternion
from six.moves import input

import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import Float64
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from franka_interface import GripperInterface

from gazebo_msgs.srv import GetLinkState 
from geometry_msgs.msg import (
    PoseStamped,
    Pose,
    Point,
    Quaternion,
)
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from moveit_msgs.msg import PlanningScene, ObjectColor, Grasp, PlaceLocation

try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import numpy as np

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

def getLocations(model_info, name):
    obj = model_info(name,"world")
    x = obj.link_state.pose.position.x
    y = obj.link_state.pose.position.y
    z = obj.link_state.pose.position.z 
    orientation = obj.link_state.pose.orientation
    # print("WWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWWw : ", w) 
    return np.array([x, y, z, orientation])

class Gripper:
    def __init__(self):
        self.finger1_pub = rospy.Publisher('/panda_finger1_controller/command', Float64, queue_size=10)
        self.finger2_pub = rospy.Publisher('/panda_finger2_controller/command', Float64, queue_size=10)
        rospy.sleep(3)
        
    def grasp(self, finger1_y, finger2_y):
        finger1_data = Float64()
        finger1_data.data = finger1_y
        finger2_data = Float64()
        finger2_data.data = finger2_y
        
        self.finger1_pub.publish(finger1_data)
        self.finger2_pub.publish(finger2_data)

class MoveGroupPythonInterfaceTutorial(object):
    """MoveGroupPythonInterfaceTutorial"""

    def __init__(self):
        super(MoveGroupPythonInterfaceTutorial, self).__init__()

        ## BEGIN_SUB_TUTORIAL setup
        ##
        ## First initialize `moveit_commander`_ and a `rospy`_ node:
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("move_group_python_interface_tutorial", anonymous=True)

        ## Instantiate a `RobotCommander`_ object. Provides information such as the robot's
        ## kinematic model and the robot's current joint states
        robot = moveit_commander.RobotCommander()

        ## Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
        ## for getting, setting, and updating the robot's internal understanding of the
        ## surrounding world:
        scene = moveit_commander.PlanningSceneInterface()

        ## Instantiate a `MoveGroupCommander`_ object.  This object is an interface
        ## to a planning group (group of joints).  In this tutorial the group is the primary
        ## arm joints in the Panda robot, so we set the group's name to "panda_arm".
        ## If you are using a different robot, change this value to the name of your robot
        ## arm planning group.
        ## This interface can be used to plan and execute motions:
        group_name = "panda_arm"
        move_group = moveit_commander.MoveGroupCommander(group_name)
        

        print("dir_mg: ",dir(move_group.pick))
        ## Create a `DisplayTrajectory`_ ROS publisher which is used to display
        ## trajectories in Rviz:
        display_trajectory_publisher = rospy.Publisher(
            "/move_group/display_planned_path",
            moveit_msgs.msg.DisplayTrajectory,
            queue_size=20,
        )

        ## END_SUB_TUTORIAL

        ## BEGIN_SUB_TUTORIAL basic_info
        ##
        ## Getting Basic Information
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())
        print("")
        ## END_SUB_TUTORIAL

        # Misc variables
        self.box_name = ""
        self.robot = robot
        self.scene = scene
        self.move_group = move_group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

    def go_to_joint_state(self):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_to_joint_state
        ##
        ## Planning to a Joint Goal
        ## ^^^^^^^^^^^^^^^^^^^^^^^^
        ## The Panda's zero configuration is at a `singularity <https://www.quora.com/Robotics-What-is-meant-by-kinematic-singularity>`_, so the first
        ## thing we want to do is move it to a slightly better configuration.
        ## We use the constant `tau = 2*pi <https://en.wikipedia.org/wiki/Turn_(angle)#Tau_proposals>`_ for convenience:
        # We get the joint values from the group and change some of the values:
        joint_goal = move_group.get_current_joint_values()
        joint_goal[0] = 0
        joint_goal[1] = -tau / 8
        joint_goal[2] = 0
        joint_goal[3] = -tau / 4
        joint_goal[4] = 0
        joint_goal[5] = tau / 6  # 1/6 of a turn
        joint_goal[6] = 0

        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        move_group.go(joint_goal, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        move_group.stop()

        ## END_SUB_TUTORIAL

        # For testing:
        current_joints = move_group.get_current_joint_values()
        return all_close(joint_goal, current_joints, 0.01)



    def go_to_pose_goal(self,position):
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
        pose_goal = geometry_msgs.msg.Pose()
        print("-"*20,"\nGoing to position:\n",position)
        qw,qx,qy,qz = quaternion_from_euler(0,pi,0)

        pose_goal.orientation.w = qw
        pose_goal.orientation.x = qx
        pose_goal.orientation.y = qy
        pose_goal.orientation.z = qz
        
        # quaternion = Quaternion(x=position[3].x, y=position[3].y, z=position[3].z, w=position[3].w)
        # euler = euler_from_quaternion([position[3].x,position[3].y,position[3].z,position[3].w])
        # roll = euler[0]
        # pitch = euler[1]
        # yaw = euler[2]
        # print("RPY: ",roll,pitch,yaw)

        print(quaternion)
        pose_goal.position.x = position[0]
        pose_goal.position.y = position[1]
        pose_goal.position.z = position[2]+0.1

        # grasp = Grasp()
        # grasp.grasp_pose.pose = pose_goal
        # grasp.grasp_pose.header.frame_id = "panda_link0"


        # # // Setting pre-grasp approach
        # # // ++++++++++++++++++++++++++
        # # /* Defined with respect to frame_id */
        # grasp.pre_grasp_approach.direction.header.frame_id = "panda_link0"
        # # /* Direction is set as positive x axis */
        # grasp.pre_grasp_approach.direction.vector.z = -1.0
        # grasp.pre_grasp_approach.min_distance = 0.095
        # grasp.pre_grasp_approach.desired_distance = 0.115

        # # Setting post-grasp retreat
        # grasp.post_grasp_retreat.direction.header.frame_id = "panda_link0"
        # grasp.post_grasp_retreat.direction.vector.z = 1.0
        # grasp.post_grasp_retreat.min_distance = 0.1
        # grasp.post_grasp_retreat.desired_distance = 0.25


        # # hand_commander = moveit_commander.MoveGroupCommander("panda_hand")
        # # hand_commander.set_named_target("open")
        # # plan = hand_commander.plan()
        # # if not hand_commander.execute(plan, wait=True):
        # #     return False
        # # return True


        # grasp.pre_grasp_posture.joint_names.append("panda_finger_joint1")
        # grasp.pre_grasp_posture.joint_names.append("panda_finger_joint2")
        # jt = JointTrajectory()
        # jtp = JointTrajectoryPoint()
        
        # jtp.positions = (0.04, 0.04) 
        # jtp.time_from_start = rospy.Duration(1) #1 second
        # jt.joint_names = ["panda_finger_joint1","panda_finger_joint2"] 
        # jt.points.append(jtp)
        # grasp.grasp_posture.points.append(jtp)

        # grasp.grasp_posture.joint_names.append("panda_finger_joint1")
        # grasp.grasp_posture.joint_names.append("panda_finger_joint2")
        # jt = JointTrajectory()
        # jtp = JointTrajectoryPoint()
        
        # jtp.positions = (0.03, 0.01) 
        # jtp.time_from_start = rospy.Duration(1) #1 second
        # jt.joint_names = ["panda_finger_joint1","panda_finger_joint2"] 
        # jt.points.append(jtp)
        # grasp.grasp_posture.points.append(jtp)

        # move_group.pick("unit_box_0",grasp)

        # rospy.sleep(1)


        # place_location = PlaceLocation()

        # # // Setting place location pose
        # # // +++++++++++++++++++++++++++
        # place_location.place_pose.header.frame_id = "panda_link0"
        # place_location.place_pose.pose = pose_goal
        # place_location.place_pose.pose.position.y += 0.3

        # # // Setting pre-place approach
        # # // ++++++++++++++++++++++++++
        # # /* Defined with respect to frame_id */
        # place_location.pre_place_approach.direction.header.frame_id = "panda_link0"
        # # /* Direction is set as negative z axis */
        # place_location.pre_place_approach.direction.vector.z = -1.0
        # place_location.pre_place_approach.min_distance = 0.095
        # place_location.pre_place_approach.desired_distance = 0.115

        # # // Setting post-grasp retreat
        # # // ++++++++++++++++++++++++++
        # # /* Defined with respect to frame_id */
        # place_location.post_place_retreat.direction.header.frame_id = "panda_link0"
        # # /* Direction is set as negative y axis */
        # place_location.post_place_retreat.direction.vector.y = -1.0
        # place_location.post_place_retreat.min_distance = 0.1
        # place_location.post_place_retreat.desired_distance = 0.25

        # # // Setting posture of eef after placing object
        # # // +++++++++++++++++++++++++++++++++++++++++++
        # # /* Similar to the pick case */



        # place_location.post_place_posture.joint_names.append("panda_finger_joint1")
        # place_location.post_place_posture.joint_names.append("panda_finger_joint2")
        # jtp = JointTrajectoryPoint()
        
        # jtp.positions = (0.0, 0.0) 
        # jtp.time_from_start = rospy.Duration(1) #1 second
        # place_location.post_place_posture.points.append(jtp)

        



        # # // Set support surface as table2.
        # # move_group.setSupportSurfaceName("table")
        # # // Call place to place the object using the place locations given.
        # move_group.place("unit_box_0", place_location)

        # rate = rospy.Rate(1.0)

        # while not rospy.is_shutdown():
        #     rate.sleep()









        # hand_commander.set_named_target("close")
        # plan = hand_commander.plan()
        # if not hand_commander.execute(plan, wait=True):
        #     return False

        # return True
        # print(grasp.pre_grasp_posture.points.positions[0])
        # print("GRASP joint names: ", grasp.pre_grasp_posture.joint_names)

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

    def plan_cartesian_path(self, scale=1):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL plan_cartesian_path
        ##
        ## Cartesian Paths
        ## ^^^^^^^^^^^^^^^
        ## You can plan a Cartesian path directly by specifying a list of waypoints
        ## for the end-effector to go through. If executing  interactively in a
        ## Python shell, set scale = 1.0.
        ##
        waypoints = []

        wpose = move_group.get_current_pose().pose
        wpose.position.z -= scale * 0.1  # First move up (z)
        wpose.position.y += scale * 0.2  # and sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y -= scale * 0.1  # Third move sideways (y)
        waypoints.append(copy.deepcopy(wpose))

        # We want the Cartesian path to be interpolated at a resolution of 1 cm
        # which is why we will specify 0.01 as the eef_step in Cartesian
        # translation.  We will disable the jump threshold by setting it to 0.0,
        # ignoring the check for infeasible jumps in joint space, which is sufficient
        # for this tutorial.
        (plan, fraction) = move_group.compute_cartesian_path(
            waypoints, 0.01, 0.0  # waypoints to follow  # eef_step
        )  # jump_threshold

        # Note: We are just planning, not asking move_group to actually move the robot yet:
        return plan, fraction

        ## END_SUB_TUTORIAL

    def display_trajectory(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher

        ## BEGIN_SUB_TUTORIAL display_trajectory
        ##
        ## Displaying a Trajectory
        ## ^^^^^^^^^^^^^^^^^^^^^^^
        ## You can ask RViz to visualize a plan (aka trajectory) for you. But the
        ## group.plan() method does this automatically so this is not that useful
        ## here (it just displays the same trajectory again):
        ##
        ## A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
        ## We populate the trajectory_start with our current robot state to copy over
        ## any AttachedCollisionObjects and add our plan to the trajectory.
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        # Publish
        display_trajectory_publisher.publish(display_trajectory)

        ## END_SUB_TUTORIAL

    def execute_plan(self, plan):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        move_group = self.move_group

        ## BEGIN_SUB_TUTORIAL execute_plan
        ##
        ## Executing a Plan
        ## ^^^^^^^^^^^^^^^^
        ## Use execute if you would like the robot to follow
        ## the plan that has already been computed:
        move_group.execute(plan, wait=True)

        ## **Note:** The robot's current joint state must be within some tolerance of the
        ## first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
        ## END_SUB_TUTORIAL

    def wait_for_state_update(
        self, box_is_known=False, box_is_attached=False, timeout=4
    ):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL wait_for_scene_update
        ##
        ## Ensuring Collision Updates Are Received
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## If the Python node dies before publishing a collision object update message, the message
        ## could get lost and the box will not appear. To ensure that the updates are
        ## made, we wait until we see the changes reflected in the
        ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
        ## For the purpose of this tutorial, we call this function after adding,
        ## removing, attaching or detaching an object in the planning scene. We then wait
        ## until the updates have been made or ``timeout`` seconds have passed
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            # Note that attaching the box will remove it from known_objects
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True

            # Sleep so that we give other threads time on the processor
            rospy.sleep(0.1)
            seconds = rospy.get_time()

        # If we exited the while loop without returning then we timed out
        return False
        ## END_SUB_TUTORIAL

    def add_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL add_box
        ##
        ## Adding Objects to the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## First, we will create a box in the planning scene between the fingers:
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "panda_hand"
        box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = 0.11  # above the panda_hand frame
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.075, 0.075, 0.075))

        ## END_SUB_TUTORIAL
        # Copy local variables back to class variables. In practice, you should use the class
        # variables directly unless you have a good reason not to.
        self.box_name = box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        robot = self.robot
        scene = self.scene
        eef_link = self.eef_link
        group_names = self.group_names

        ## BEGIN_SUB_TUTORIAL attach_object
        ##
        ## Attaching Objects to the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
        ## robot be able to touch them without the planning scene reporting the contact as a
        ## collision. By adding link names to the ``touch_links`` array, we are telling the
        ## planning scene to ignore collisions between those links and the box. For the Panda
        ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
        ## you should change this value to the name of your end effector group name.
        grasping_group = "hand"
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=True, box_is_known=False, timeout=timeout
        )

    def detach_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene
        eef_link = self.eef_link

        ## BEGIN_SUB_TUTORIAL detach_object
        ##
        ## Detaching Objects from the Robot
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can also detach and remove the object from the planning scene:
        scene.remove_attached_object(eef_link, name=box_name)
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_known=True, box_is_attached=False, timeout=timeout
        )

    def remove_box(self, timeout=4):
        # Copy class variables to local variables to make the web tutorials more clear.
        # In practice, you should use the class variables directly unless you have a good
        # reason not to.
        box_name = self.box_name
        scene = self.scene

        ## BEGIN_SUB_TUTORIAL remove_object
        ##
        ## Removing Objects from the Planning Scene
        ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
        ## We can remove the box from the world.
        scene.remove_world_object(box_name)

        ## **Note:** The object must be detached before we can remove it from the world
        ## END_SUB_TUTORIAL

        # We wait for the planning scene to update.
        return self.wait_for_state_update(
            box_is_attached=False, box_is_known=False, timeout=timeout
        )


def main():
    try:
        print("")
        print("----------------------------------------------------------")
        print("Welcome to the MoveIt MoveGroup Python Interface Tutorial")
        print("----------------------------------------------------------")
        print("Press Ctrl-D to exit at any time")
        print("")
        # input(
        #     "============ Press `Enter` to begin the tutorial by setting up the moveit_commander ..."
        # )
        tutorial = MoveGroupPythonInterfaceTutorial()

        # input(
        #     "============ Press `Enter` to execute a movement using a joint state goal ..."
        # )
        # tutorial.go_to_joint_state()
        
        # Add Table mesh for planner
        scene_pub = rospy.Publisher('/move_group/monitored_planning_scene', PlanningScene, queue_size=10)
        # robot = moveit_commander.RobotCommander()
        model_info= rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)  
        
        REFERENCE_FRAME = '/world'
        table_id = "table"
        def getpose(pose):
            pose = pose.link_state.pose
            pose_Ground = geometry_msgs.msg.PoseStamped()
            pose_Ground.header.frame_id = REFERENCE_FRAME
            # pose_Ground.header.frame_id = robot.get_planning_frame()
            pose_Ground.pose.position.x = pose.position.x
            pose_Ground.pose.position.y = pose.position.y
            pose_Ground.pose.position.z = pose.position.z
            return pose_Ground
        
        def getColor(name, r, g, b, a = 0.9):
            # Initialize a MoveIt color object
            color = ObjectColor()

            # Set the id to the name given as an argument
            color.id = name

            # Set the rgb and alpha values given as input
            color.color.r = r
            color.color.g = g
            color.color.b = b
            color.color.a = a

            # Update the global color dictionary
            return color

            

        tutorial.scene.add_mesh(table_id,getpose(model_info("table","world")),'/home/aswath/umd/661/ENPM-661/Project4/catkin_ws/src/panda_pkg/models/table_scaled.stl')
        tutorial.scene.add_box("obstacle_box",getpose(model_info("unit_box::link","world")),size=(0.30307, 0.05, 0.407809))
        ### Make the target purple ###
        colors={}
        colors['obstacle_box']=getColor('obstacle_box',0.6, 0, 0, 1.0)
        # tutorial.scene.object_colors.append(colors)
        # tutorial.scene.setColor('obstacle_box',0.6, 0, 0, 1.0)


        tutorial.scene.add_box("object",getpose(model_info("unit_box_0::link","world")),size=(0.035, 0.035, 0.035))
        
        # scene_pub.publish(PlanningScene)

        gi = GripperInterface()
        print("Open performed sucessfully? ", gi.open())
        rospy.sleep(1)

        start_location = getLocations(model_info, "unit_box_0::link")
        # offset = getLocations(model_info, "panda::panda_link0")
        # start_location[:3] = offset[:3]- start_location[:3] 
        tutorial.go_to_pose_goal(start_location)
        print("Grasp performed sucessfully? ", gi.grasp( width=0.04, force=100, epsilon_inner = 0.001, epsilon_outer = 0.003))
        # print("Close performed sucessfully? ", gi.close())
        rospy.sleep(1)

        #Move to goal location
        goal_location = start_location
        goal_location[1] = goal_location[1]+0.3
        tutorial.go_to_pose_goal(goal_location)
        # print("Open performed sucessfully? ", gi.open())


        # input("============ Press `Enter` to plan and display a Cartesian path ...")
        # cartesian_plan, fraction = tutorial.plan_cartesian_path()

        # input(
        #     "============ Press `Enter` to display a saved trajectory (this will replay the Cartesian path)  ..."
        # )
        # tutorial.display_trajectory(cartesian_plan)

        # input("============ Press `Enter` to execute a saved path ...")
        # tutorial.execute_plan(cartesian_plan)

        # input("============ Press `Enter` to add a box to the planning scene ...")
        # tutorial.add_box()

        # input("============ Press `Enter` to attach a Box to the Panda robot ...")
        # tutorial.attach_box()

        # input(
        #     "============ Press `Enter` to plan and execute a path with an attached collision object ..."
        # )
        # cartesian_plan, fraction = tutorial.plan_cartesian_path(scale=-1)
        # tutorial.execute_plan(cartesian_plan)

        # # input("============ Press `Enter` to detach the box from the Panda robot ...")
        # # tutorial.detach_box()

        # # input(
        # #     "============ Press `Enter` to remove the box from the planning scene ..."
        # # )
        # # tutorial.remove_box()

        print("============ Python tutorial demo complete!")
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return


if __name__ == "__main__":
    main()

## BEGIN_TUTORIAL
## .. _moveit_commander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/namespacemoveit__commander.html
##
## .. _MoveGroupCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html
##
## .. _RobotCommander:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1robot_1_1RobotCommander.html
##
## .. _PlanningSceneInterface:
##    http://docs.ros.org/noetic/api/moveit_commander/html/classmoveit__commander_1_1planning__scene__interface_1_1PlanningSceneInterface.html
##
## .. _DisplayTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/DisplayTrajectory.html
##
## .. _RobotTrajectory:
##    http://docs.ros.org/noetic/api/moveit_msgs/html/msg/RobotTrajectory.html
##
## .. _rospy:
##    http://docs.ros.org/noetic/api/rospy/html/
## CALL_SUB_TUTORIAL imports
## CALL_SUB_TUTORIAL setup
## CALL_SUB_TUTORIAL basic_info
## CALL_SUB_TUTORIAL plan_to_joint_state
## CALL_SUB_TUTORIAL plan_to_pose
## CALL_SUB_TUTORIAL plan_cartesian_path
## CALL_SUB_TUTORIAL display_trajectory
## CALL_SUB_TUTORIAL execute_plan
## CALL_SUB_TUTORIAL add_box
## CALL_SUB_TUTORIAL wait_for_scene_update
## CALL_SUB_TUTORIAL attach_object
## CALL_SUB_TUTORIAL detach_object
## CALL_SUB_TUTORIAL remove_object
## END_TUTORIAL