#!/usr/bin/env python

import argparse
import rospy
import sys
# import socket
# import intera_interface
# import intera_external_devices
# from intera_interface import CHECK_VERSION

import numpy as np
import pdb

import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

class RobotController(object):

    def __init__(self):
        """Initializes a controller for the robot"""

        print("Initializing node... ")
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node("ur5_custom_controller")
        rospy.on_shutdown(self.clean_shutdown)

        robot = moveit_commander.RobotCommander()

        # rs = intera_interface.RobotEnable(CHECK_VERSION)
        # init_state = rs.state().enabled
        print("Robot enabled...")

        group = moveit_commander.MoveGroupCommander('manipulator')

        planning_frame = group.get_planning_frame()
        print "============ Reference frame: %s" % planning_frame

        eef_link = group.get_end_effector_link()
        print "============ End effector: %s" % eef_link

        group_names = robot.get_group_names()
        print "============ Robot Groups:", robot.get_group_names()

        print "============ Printing robot state"
        print robot.get_current_state()
        print ""

        self.robot = robot
        # self.scene = scene
        self.group = group
        # self.display_trajectory_publisher = display_trajectory_publisher
        #self.postion_publish = postion_publish
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names

        # self._low_bound = np.array([-0.416, 0.352, 0.167, -1])
        # self._high_bound = np.array([0.556, 0.926, 0.170, 1])
        self._low_bound = np.array([-0.35, 0.360, 0.118, -1])
        self._high_bound = np.array([0.23, 0.920, 0.125, 1])

        group.set_max_velocity_scaling_factor(0.1)
        group.set_max_acceleration_scaling_factor(0.08)
        # self.limb = intera_interface.Limb("right")

        # try:
        #     self.gripper = intera_interface.Gripper("right")
        # except:
        #     self.has_gripper = False
        #     rospy.logerr("Could not initalize the gripper.")
        # else:
        #     self.has_gripper = True

        # self.joint_names = self.limb.joint_names()
        print("Done initializing controller.")

        # set gripper
        # try:
        #     self.gripper = intera_interface.Gripper("right")
        # except ValueError:
        #     rospy.logerr("Could not detect a gripper attached to the robot.")
        #     return
        #
        # self.gripper.calibrate()
        # self.gripper.set_velocity(self.gripper.MAX_VELOCITY) #"set 100% velocity"),
        # self.gripper.open()


    def set_joint_delta(self, joint_name, delta):
        """Move a single joint by a delta"""
        current_position = self.limb.joint_angle(joint_name)
        self.set_joint(joint_name, current_position + delta)

    def set_joint(self, joint_name, pos):
        """Move a single joint to a target position"""
        joint_command = {joint_name: pos}
        self.limb.set_joint_positions(joint_command)

    def set_joints(self, command):
        """Move joints to commmand"""
        self.limb.move_to_joint_positions(command)

    def set_joints_nonblocking(self, command):
        """Move joints to commmand, resending until reached"""
        for i in range(100000):
            self.limb.set_joint_positions(command)
            current = self.limb.joint_angles()
            if np.all(abs(distance_between_commands(current, command)) < TOLERANCE):
                rospy.loginfo("Reached target")
                break
        rospy.loginfo("Finished motion")

    def set_gripper(self, action):
        if self.has_gripper:
            if action == "close":
                self.gripper.close()
            elif action == "open":
                self.gripper.open()
            elif action == "calibrate":
                self.gripper.calibrate()

    # def set_neutral(self, speed = .2):
    #     # using a custom handpicked neutral position
    #     # starting from j0 to j6:
    #     neutral_jointangles = [0.412271, -0.434908, -1.198768, 1.795462, 1.160788, 1.107675, 2.068076]
    #     cmd = dict(zip(self.joint_names, neutral_jointangles))
    #
    #     self.limb.set_joint_position_speed(speed)
    #
    #     done = False
    #     while not done:
    #         try:
    #             self.set_joints(cmd)
    #         except:
    #             print 'retrying set neutral...'
    #
    #         done = True

        # self.limb.move_to_neutral()
    def go_to_pose_goal(self, goal_pose):

        group = self.group

        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.orientation.w = 0.535960768954  #1.0
        pose_goal.orientation.x = -0.415964133446
        pose_goal.orientation.y = 0.352373748554
        pose_goal.orientation.z = 0.644633721705

        pose_goal.position.x = goal_pose[0]
        pose_goal.position.y = goal_pose[1]
        pose_goal.position.z = goal_pose[2]


        group.set_pose_target(pose_goal)
        #group.set_random_target()

        plan = group.go(wait=True)

        group.stop()

        group.clear_pose_targets()

        current_pose = group.get_current_pose().pose
        print("New current pose: ", current_pose.position)

        # current_pose = self.group.get_current_pose().pose
        # return all_close(pose_goal, current_pose, 0.01)


    def set_neutral(self):
        pose_goal = geometry_msgs.msg.Pose()

        pose_goal.position.x = -0.0
        pose_goal.position.y = 0.6
        pose_goal.position.z = 0.15
        pose_goal.orientation.w = 0.535960768954  #1.0
        pose_goal.orientation.x = -0.415964133446
        pose_goal.orientation.y = 0.352373748554
        pose_goal.orientation.z = 0.644633721705

        self.group.set_pose_target(pose_goal)
        plan = self.group.go(wait=True)
        self.group.stop()
        self.group.clear_pose_targets()

    def get_pose(self):
        group = self.group
        current_pose = group.get_current_pose().pose
        # print("Current pose: ", current_pose)
        return current_pose

    def clean_shutdown(self):
        print("\nExiting example.")
        # if not init_state:
        #     print("Disabling robot...")
            # rs.disable()


def distance_between_commands(j1, j2):
    a = []
    b = []
    for joint in j1:
        a.append(j1[joint])
        b.append(j2[joint])
    return np.array(a) - np.array(b)
