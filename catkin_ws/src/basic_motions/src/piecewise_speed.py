#!/usr/bin/env python
import os, sys
import numpy as np
import rospy
import helpers.services as services
from geometry_msgs.msg import PoseStamped
from robot_comm.srv import robot_SetCartesian, robot_ActivateEGM, robot_StopEGM

# EGM demo: Time-piecewise speed function
# -------------------------------
# This script activates EGM and uses it to keep the robot moving in
# different potentially time-dependent speeds among time, defined as functions.

# Robot name
robot_name = 'robot2'
# Publishing frequency in hz
hz = 248.0

# Speed functions (in mm/s)
# In this case: (v_x, v_y, v_z, omega_x, omega_y, omega_z)
def speed1(t):
    return [
        0.0, 0.0, 5.0, 0.0, 0.0, 0.0
    ]

def speed2(t):
    return [
        0.0, 0.0, -5.0, 0.0, 0.0, 0.0
    ]

# Speed function temporization
periods = [
    # [speed_function, time_length]
    [speed1, 10],
    [speed2, 10],
    [speed1, 10],
    [speed2, 10]
]


if __name__ == '__main__':
    rospy.init_node('piecewise_speed', anonymous=True)
    command_pose_pub = rospy.Publisher(robot_name + '_EGM/SetCartesian', PoseStamped, queue_size = 100, latch=True)

    # Then we activate EGM in velocity mode and a high timeout (e.g. a day)
    ret = services.ActivateEGM_client(true, 86400)
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
        vel = periods[i][0](t)
        pose.pose.position.x = vel[0]
        pose.pose.position.y = vel[1]
        pose.pose.position.z = vel[2]
        # Orientation or angular velocity in xyzw
        pose.pose.orientation.x = vel[3]
        pose.pose.orientation.y = vel[4]
        pose.pose.orientation.z = vel[5]
        pose.pose.orientation.w = 0.0
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
