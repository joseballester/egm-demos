#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
import helpers.services as services
from geometry_msgs.msg import PoseStamped
from robot_comm.srv import robot_SetCartesian, robot_ActivateEGM, robot_StopEGM

# EGM demo: Time-piecewise pose function
# -------------------------------
# This script calls SetCartesian to move the robot to initial_pos,
# and then activates EGM and uses it to keep the robot moving in a circle
# centered at initial_pos with radius R, with a period T.

# First period will be used to move from the center to the circle path.
# Orientation will remain constant, as defined in initial_orient.

# Robot name
robot_name = 'robot2'

# Initial position in mm
initial_pos = [300.0, 0.0, 278.2]
# Initial orientation in quaternion (qx, qy, qz, qw)
initial_orient = [0.0, -1.0, 0.0, 0.0]

# Period length in seconds
T = 10.0
# Circle radius in mm
R = 50
# Publishing frequency in hz
hz = 248.0

# Motion functions
def go_to_circle(t):
    return [
        initial_pos[0] + R*t/T,
        initial_pos[1],
        initial_pos[2]
    ]

def keep_in_circle(t):
    return [
        initial_pos[0] + R*np.cos(t/T),
        initial_pos[1] + R*np.sin(t/T),
        initial_pos[2]
    ]

# Motion function temporization
periods = [
    # [motion_function, time_length]
    [go_to_circle, T],
    [keep_in_circle, 1.0e9]
]


if __name__ == '__main__':
    rospy.init_node('piecewise_pose', anonymous=True)
    command_pose_pub = rospy.Publisher(robot_name + '_EGM/SetCartesian', PoseStamped, queue_size = 100, latch=True)

    # First we move to the initial position
    ret = services.SetCartesian_client(initial_pos[0], initial_pos[1], initial_pos[2], initial_orient[3], initial_orient[0], initial_orient[1], initial_orient[2])
    if not ret:
        rospy.loginfo('Translation to initial position was not successful.')
        exit()

    # Then we activate EGM in position mode and a high timeout (e.g. a day)
    ret = services.ActivateEGM_client(false, 86400)
    if not ret:
        rospy.loginfo('EGM activation was not successful.')
        exit()

    rate = rospy.Rate(hz)
    start_time = rospy.Time.now().to_sec()
    i = 0

    while (not rospy.is_shutdown()) and i < len(periods):
        now = rospy.Time.now()
        t = now.to_sec()-start_time
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        # Position in mm or velocity in mm/s
        pos = periods[i][0](t)
        pose.pose.position.x = pos[0]
        pose.pose.position.y = pos[1]
        pose.pose.position.z = pos[2]
        # Orientation or angular velocity in xyzw
        pose.pose.orientation.x = initial_orient[0]
        pose.pose.orientation.y = initial_orient[1]
        pose.pose.orientation.z = initial_orient[2]
        pose.pose.orientation.w = initial_orient[3]
        command_pose_pub.publish(pose)

        if rospy.Time.now().to_sec()-start_time >= periods[i][1]:
            i += 1
            start_time = rospy.Time.now().to_sec()

        rate.sleep()

    # Finally, stop EGM
    ret = services.StopEGM_client()
    if not ret:
        rospy.loginfo('EGM deactivation was not successful.')
        exit()
