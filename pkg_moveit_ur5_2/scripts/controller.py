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


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')


class Ur5Gripper:

    # Constructor
    def __init__(self):

        rospy.init_node('node_eg3_set_joint_angles', anonymous=True)

        self._planning_group = "gripper_group"
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


        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Gripper init done." + '\033[0m')

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
        self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    # Destructor

    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Gripper Deleted." + '\033[0m')


# Main function
def main():
    ur5 = Ur5Moveit()
    gripper = Ur5Gripper()

    # gripper open and close
    open = [math.radians(0)]

    close = [math.radians(11)]

    #object 1
    lst_joint_angles_1_up = [math.radians(190),
                          math.radians(-115),
                          math.radians(-108),
                          math.radians(-48),
                          math.radians(89),
                          math.radians(-214)]

    lst_joint_angles_1_down = [math.radians(189),
                          math.radians(-125),
                          math.radians(-111),
                          math.radians(-35),
                          math.radians(89),
                          math.radians(-214)]

    #object 2
    lst_joint_angles_2_up = [math.radians(219),
                          math.radians(-107),
                          math.radians(-116),
                          math.radians(-48),
                          math.radians(-270),
                          math.radians(101)]

    #lst_joint_angles_2_down =




    #object 3
    lst_joint_angles_3_up = [math.radians(-34),
                          math.radians(296),
                          math.radians(96),
                          math.radians(-119),
                          math.radians(269),
                          math.radians(56)]

    #lst_joint_angles_3_down =


    #box 1
    box_1 = [math.radians(-277),
              math.radians(-60),
              math.radians(67),
              math.radians(-97),
              math.radians(272),
              math.radians(-96)]

    #box 2
    box_2_1 = [math.radians(-87),
              math.radians(305),
              math.radians(63),
              math.radians(-98),
              math.radians(268),
              math.radians(93)]

    box_2_2 = [math.radians(-106),
              math.radians(304),
              math.radians(65),
              math.radians(-99),
              math.radians(268),
              math.radians(75)]


    while not rospy.is_shutdown():
        gripper.set_joint_angles(open)
        rospy.sleep(0.1)

        # object 1 -> box 1
        ur5.set_joint_angles(lst_joint_angles_1_up)
        rospy.sleep(1)
        ur5.set_joint_angles(lst_joint_angles_1_down)
        rospy.sleep(1)
        gripper.set_joint_angles(close)
        rospy.sleep(1)
        ur5.set_joint_angles(box_1)
        rospy.sleep(1)
        gripper.set_joint_angles(open)
        rospy.sleep(0.5)

        #continue same process for other 2 objects

        # object 2 -> box_2_1



        # object 3 -> box_2_2




    del ur5
    del gripper



if __name__ == '__main__':
    main()
