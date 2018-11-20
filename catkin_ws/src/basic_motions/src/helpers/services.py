#!/usr/bin/env python
import os, sys
import rospy
from robot_comm.srv import robot_SetCartesian, robot_ActivateEGM, robot_StopEGM

# SetCartesian service proxy
def SetCartesian_client(x, y, z, q0, qx, xy, qz):
    rospy.wait_for_service(robot_name + '_SetCartesian')
    try:
        SetCartesian = rospy.ServiceProxy(robot_name + '_SetCartesian', robot_SetCartesian)
        resp = SetCartesian(x, y, z, q0, qx, xy, qz)
        return resp.ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# ActivateEGM service proxy
def ActivateEGM_client(mode, timeout):
    rospy.wait_for_service(robot_name + '_ActivateEGM')
    try:
        ActivateEGM = rospy.ServiceProxy(robot_name + '_ActivateEGM', robot_ActivateEGM)
        resp = ActivateEGM(mode, timeout)
        return resp.ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

# StopEGM service proxy
def StopEGM_client():
    rospy.wait_for_service(robot_name + '_StopEGM')
    try:
        StopEGM = rospy.ServiceProxy(robot_name + '_StopEGM', robot_ActivateEGM)
        resp = StopEGM()
        return resp.ret
    except rospy.ServiceException, e:
        print "Service call failed: %s"%e
