#!/usr/bin/env python
'''
Task 2 Controller Script by SB#871
Reference: https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/
Sending a sequence of Goals to ROS NavStack with Python by Fiorella Sibona

Here, instead of taking waypoint completion feedback from "statuses" in "end_cb"
we decided to take the actual "odom" position of robot in "running_cb"
and compare it with the goal.

Note: We have used "global_planner" instead of default "navfn". You can see that
in "move_base_params" and "global_planner_params". Also we have used
"Trajectory_Planner" (which is default local planner - base_local_planner).
'''

import time
import math
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from matplotlib.pyplot import plot


class MoveBaseSeq():
    '''
    Move Base Class. Initiated in the __main__ function.
    Sends continuous goals using actionlib
    '''

    def __init__(self):
        rospy.init_node('move_base_sequence', disable_signals=True)

        # The waypoints / goals are defined here as co-ordinates [x,y,z]
        self.points = [[-9.1, -1.2, 0], [10.7, 10.5, 0],
                       [12.6, -1.8, 0], [18.2, -1.4, 0], [-2.0, 4.0, 0]]
        # NOTE: in 3rd waypoint, we changed -1.9 to -1.8, because that point was so close to the map's border
        # that our global planner was not able to define any path for it, causing an error.
        # We made sure that the output is within tolerance range of 0.5

        # the goal sequence which we will send to the move_base
        self.pose_seq = list()
        # completed goals count
        self.goal_cnt = 0

        head_seq = list()
        # this is the expected final heading of the bot at each waypoint.
        # Not mentioned in the md book so we have just put
        # 0 degree heading for all waypoints

        headings = [0, 0, 0, 0, 0]
        # coverting from eular to quaternion
        for angle in headings:
            head_seq.append(Quaternion(
                *(quaternion_from_euler(0, 0, angle*math.pi/180, axes='sxyz'))))

        # counter
        n = 0
        for point in self.points:
            self.pose_seq.append(Pose(Point(*point), head_seq[n]))
            n += 1

        # Create action client
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        wait = self.client.wait_for_server(rospy.Duration(20.0))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
            return
        rospy.loginfo("Connected to move base server")
        rospy.loginfo("Starting goals achievements ...")

        # the main function
        self.movebase_client()

    def start_cb(self):
        # this callback is called once at the start of each waypoint
        rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                      " is now being processed by the Action Server...")

    def running_cb(self, data):
        '''
        This function is called recursively. Here we compare the current bot position with the waypoint's
        co-ordinates. If the diff in both x and y axis is less than threshold value (0.3), then the goal is
        reached and we move on to next goal
        '''

        # To print current pose at each feedback:
        #rospy.loginfo("Feedback for goal "+str(self.goal_cnt+1)+": "+str(data))

        x_pose = data.base_position.pose.position.x
        y_pose = data.base_position.pose.position.y
        GOAL_X = self.points[self.goal_cnt][0]
        GOAL_Y = self.points[self.goal_cnt][1]
        inc_x = GOAL_X - x_pose
        inc_y = GOAL_Y - y_pose
        print "x diff: "+str(inc_x) + "   y diff: " + str(inc_y)

        if abs(inc_x) < 0.395 and abs(inc_y) < 0.395:
            self.goal_cnt += 1
            rospy.loginfo("Goal pose "+str(self.goal_cnt)+" reached")
            if self.goal_cnt < len(self.pose_seq):
                next_goal = MoveBaseGoal()
                next_goal.target_pose.header.frame_id = "map"
                next_goal.target_pose.header.stamp = rospy.Time.now()

                next_goal.target_pose.pose = self.pose_seq[self.goal_cnt]
                rospy.loginfo("Sending goal pose " +
                              str(self.goal_cnt+1)+" to Action Server")
                rospy.loginfo(str(self.pose_seq[self.goal_cnt]))

                self.client.send_goal(
                    next_goal, self.end_cb, self.start_cb, self.running_cb)
            else:
                rospy.loginfo("Final goal pose reached!")
                rospy.signal_shutdown("Final goal pose reached!")
                plot(self.xdata, self.ydata)
                return

    def end_cb(self, status, result):
        '''
        This feedback gives the final status of navigation. All the mentioned status are conditions where
        robot stops navigating.
        (status == 3 corresponds to robot reaching it's current waypoint successfully, but we were getting errors
        using that method. So we decided to directly use the feedback location data in the "running_cb" function)
        '''
        # Reference for terminal status values: http://docs.ros.org/diamondback/api/actionlib_msgs/html/msg/GoalStatus.html
        if status == 2:
            rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                          " received a cancel request after it started executing, completed execution!")

        elif status == 4:
            rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                          " was aborted by the Action Server")
            rospy.signal_shutdown(
                "Goal pose "+str(self.goal_cnt+1)+" aborted, shutting down!")
            return

        elif status == 5:
            rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                          " has been rejected by the Action Server")
            rospy.signal_shutdown(
                "Goal pose "+str(self.goal_cnt+1)+" rejected, shutting down!")
            return

        elif status == 8:
            rospy.loginfo("Goal pose "+str(self.goal_cnt+1) +
                          " received a cancel request before it started executing, successfully cancelled!")

        else:
            # this else condition is mostly called to force send goal to the move_base
            print "Force restarting navigation. Sending current goal again.!!"
            time.sleep(2)
            self.movebase_client()

    def movebase_client(self):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose = self.pose_seq[self.goal_cnt]
        rospy.loginfo("Sending goal pose " +
                      str(self.goal_cnt+1)+" to Action Server")
        rospy.loginfo(str(self.pose_seq[self.goal_cnt]))

        self.client.send_goal(
            goal, self.end_cb, self.start_cb, self.running_cb)

        rospy.spin()


if __name__ == '__main__':
    try:
        MoveBaseSeq()
    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation finished.")
