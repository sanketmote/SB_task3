#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math


class Ur5Moveit:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "arm_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')

        return flag_plan

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


def main():

    ur5 = Ur5Moveit()

    # ur5_pose_1 = geometry_msgs.msg.Pose()
    # ur5_pose_1.position.x = 0.451200162399
    # ur5_pose_1.position.y = 0.224974782038
    # ur5_pose_1.position.z = 0.903036171172
    # ur5_pose_1.orientation.x = -0.685179452927
    # ur5_pose_1.orientation.y = -0.223635419344
    # ur5_pose_1.orientation.z = 0.213043225003
    # ur5_pose_1.orientation.w = 0.659643010106
    #
    # ur5_pose_2 = geometry_msgs.msg.Pose()
    # ur5_pose_2.position.x = 0.540224332581
    # ur5_pose_2.position.y = -0.0221500888139
    # ur5_pose_2.position.z = 0.903913383151
    # ur5_pose_2.orientation.x = -0.333429739176
    # ur5_pose_2.orientation.y = -0.636615812955
    # ur5_pose_2.orientation.z = 0.62861366727
    # ur5_pose_2.orientation.w = 0.297304175972
    #
    # ur5_pose_3 = geometry_msgs.msg.Pose()
    # ur5_pose_3.position.x = 0.5871240135
    # ur5_pose_3.position.y = -0.320272279267
    # ur5_pose_3.position.z = 0.950909551716
    # ur5_pose_3.orientation.x = -0.6140305786
    # ur5_pose_3.orientation.y = 0.0480110593442
    # ur5_pose_3.orientation.z = 0.0504672567326
    # ur5_pose_3.orientation.w = 0.78620254561

    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.569201132988
    ur5_pose_1.position.y = -0.0203906697051
    ur5_pose_1.position.z = 0.913318335053
    ur5_pose_1.orientation.x = -0.280065095859
    ur5_pose_1.orientation.y = -0.639910027336
    ur5_pose_1.orientation.z = 0.656934879419
    ur5_pose_1.orientation.w = 0.283752115763

    ur5_pose_1_down = geometry_msgs.msg.Pose()
    ur5_pose_1_down.position.x = 0.569201132988
    ur5_pose_1_down.position.y = -0.0203906697051
    ur5_pose_1_down.position.z = 0.843318335053
    ur5_pose_1_down.orientation.x = -0.280065095859
    ur5_pose_1_down.orientation.y = -0.639910027336
    ur5_pose_1_down.orientation.z = 0.656934879419
    ur5_pose_1_down.orientation.w = 0.283752115763

    box_1 = geometry_msgs.msg.Pose()
    box_1.position.x = -0.0235100083399
    box_1.position.y = 0.704612729528
    box_1.position.z = 1.10936524976
    box_1.orientation.x = -0.494624899543
    box_1.orientation.y = -0.487877752732
    box_1.orientation.z = 0.50323358423
    box_1.orientation.w = 0.513884682438

    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = 0.461778524531
    ur5_pose_2.position.y = 0.233456734134
    ur5_pose_2.position.z = 0.926453425062
    ur5_pose_2.orientation.x = -0.17662108756
    ur5_pose_2.orientation.y = 0.683487618981
    ur5_pose_2.orientation.z = -0.688681530753
    ur5_pose_2.orientation.w = 0.165430998695

    box_2_1 = geometry_msgs.msg.Pose()
    box_2_1.position.x = 0.146988715212
    box_2_1.position.y = -0.720151876755
    box_2_1.position.z = 1.08271081781
    box_2_1.orientation.x = -0.491669335072
    box_2_1.orientation.y =-0.490729925634
    box_2_1.orientation.z = 0.509055429188
    box_2_1.orientation.w = 0.508240076195

    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.569644946384
    ur5_pose_3.position.y = -0.252547808473
    ur5_pose_3.position.z = 0.958383313877
    ur5_pose_3.orientation.x = 0.0189170904343
    ur5_pose_3.orientation.y = -0.712048158862
    ur5_pose_3.orientation.z = 0.701629263234
    ur5_pose_3.orientation.w = 0.0185994656884

    box_2_2 = geometry_msgs.msg.Pose()
    box_2_2.position.x = -0.0934119546798
    box_2_2.position.y = -0.72197476859
    box_2_2.position.z = 1.08004566798
    box_2_2.orientation.x = -0.493479404405
    box_2_2.orientation.y = -0.489511926242
    box_2_2.orientation.z = 0.501638162456
    box_2_2.orientation.w = 0.514990587742

    while not rospy.is_shutdown():
        ur5.go_to_pose(ur5_pose_1)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_1_down)
        rospy.sleep(2)
        ur5.go_to_pose(box_1)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_2)
        rospy.sleep(2)
        ur5.go_to_pose(box_2_1)
        rospy.sleep(2)
        ur5.go_to_pose(ur5_pose_3)
        rospy.sleep(2)
        ur5.go_to_pose(box_2_2)
        rospy.sleep(2)

    del ur5


if __name__ == '__main__':
    main()
