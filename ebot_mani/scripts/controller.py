#! /usr/bin/env python
'''
SB_Task3 Moveit! Arm Manipulation Controller Node
Created by Team eYRC#SB#871
Reference:
1) Python files provided in the MD Book for SB theme
'''
import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

class Ur5Moveit:
    '''
    This is a class for the Arm Manipulation. An object of this class is created in the main()
    function later. It is used to send set of joint angles to the arm in order to move it.
    The code is self explainatory.
    The planning group "arm_group" was created using moveit setup assistant and it consists of all
    active joints of arm.
    '''
    # Constructor
    def __init__(self):
        self._planning_group = "arm_group"      # the planning group for arm control
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
    '''
    This is a class for the Gripper Control. An object of this class is created in the main()
    function later. It is used to open and close the gripper using gripper_finger1_joint, by sending
    joint angles to the gripper.
    The code is self explainatory.
    The planning group "gripper_group" was created using moveit setup assistant and it consists of
    gripper_finger1_joint active joint for gripper control.
    '''
    # Constructor
    def __init__(self):
        self._planning_group = "gripper_group"      # planning group for gripper
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
    rospy.init_node('node_controller_joint_angles', anonymous=True)
    arm = Ur5Moveit()   # object for arm manipulation
    gripper = Ur5Gripper()  # object for gripper manipulation

    # gripper open and close
    open = [math.radians(0)]

    close1 = [math.radians(14)] # for object 1
    close2 = [math.radians(25)] # for object 2
    close3 = [math.radians(13.5)] # for object 3

    # Here, we have predefined all the joint angles necessary for the arm
    # to perform pick and place function.

    # object 1
    object_1_up = [math.radians(-14),
                  math.radians(-72),
                  math.radians(108),
                  math.radians(-126),
                  math.radians(-86),
                  math.radians(-54)]

    object_1_down = [math.radians(-13),
                      math.radians(-56),
                      math.radians(110),
                      math.radians(-143),
                      math.radians(-89),
                      math.radians(-54)]

    #object 2
    object_2_up = [math.radians(-141),
                  math.radians(-107),
                  math.radians(-116),
                  math.radians(-48),
                  math.radians(90),
                  math.radians(101)]

    object_2_down = [math.radians(-141),
                      math.radians(-119),
                      math.radians(-120),
                      math.radians(-32),
                      math.radians(89),
                      math.radians(101)]

    #object 3
    object_3_up = [math.radians(-193),
                  math.radians(-117),
                  math.radians(-92),
                  math.radians(-66),
                  math.radians(90),
                  math.radians(267)]

    object_3_down = [math.radians(-193),
                      math.radians(-125),
                      math.radians(-97),
                      math.radians(-54),
                      math.radians(91),
                      math.radians(268)]

    #box 1
    box_1 = [math.radians(84),
              math.radians(-59),
              math.radians(66),
              math.radians(-97),
              math.radians(-88),
              math.radians(-96)]

    #box 2 - position 1
    box_2_1 = [math.radians(-87),
              math.radians(-53),
              math.radians(72),
              math.radians(-109),
              math.radians(-92),
              math.radians(93)]

    #box 2 - position 2
    box_2_2 = [math.radians(-106),
              math.radians(-55),
              math.radians(65),
              math.radians(-99),
              math.radians(-92),
              math.radians(255)]

    # Actual Control Start !!!

    gripper.set_joint_angles(open)

    # object 1 -> box 1
    arm.set_joint_angles(object_1_up)
    arm.set_joint_angles(object_1_down)
    gripper.set_joint_angles(close1)
    arm.set_joint_angles(object_1_up)
    arm.set_joint_angles(box_1)
    gripper.set_joint_angles(open)

    # object 2 -> box_2_1
    arm.set_joint_angles(object_2_up)
    arm.set_joint_angles(object_2_down)
    gripper.set_joint_angles(close2)
    arm.set_joint_angles(box_2_1)
    gripper.set_joint_angles(open)

    # object 3 -> box_2_2
    arm.set_joint_angles(object_3_up)
    arm.set_joint_angles(object_3_down)
    gripper.set_joint_angles(close3)
    arm.set_joint_angles(box_2_2)
    gripper.set_joint_angles(open)

    #After completing all process
    rospy.sleep(0.1)
    del arm
    del gripper



if __name__ == '__main__':
    main()
