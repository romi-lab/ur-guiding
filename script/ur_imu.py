#!/usr/bin/env python
# -*- coding: UTF-8 -*-



#"/home/rick/.local/lib/python2.7/site-packages/urx/robot.py"

import sys  
sys.path.append('../urx/')  
import urx

######################################
# for moving ur with imu orientation

import cv2
import numpy as np
from numpy import pi
import time
import sys
import copy
import rospy
import logging
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose
from sensor_msgs.msg import Imu
import math
from scipy.spatial.transform import Rotation as R

#angle
Angle = [0.0]*3
#angle_in_quat
Angle_quat = [0.0]*4

def callback_raw_imu(raw_imu):
    
    global Angle_quat, Angle, ith, print_count, i

    Angle_quat[0] = raw_imu.orientation.x
    Angle_quat[1] = raw_imu.orientation.y
    Angle_quat[2] = raw_imu.orientation.z
    Angle_quat[3] = raw_imu.orientation.w

    r = R.from_quat(Angle_quat)
    rot_vector = r.as_rotvec()

    pose = []
    robot_current_pose = robot.getl()

    pose.append(robot_current_pose[0])              #px
    pose.append(robot_current_pose[1])              #py
    pose.append(robot_current_pose[2])              #pz

    r = R.from_quat(Angle_quat)
    rot_vector = r.as_rotvec()
    rot_vector_offset = [0, 0, 0]

    pose.append(rot_vector[0]+rot_vector_offset[0]) #rx
    pose.append(-(rot_vector[1]+rot_vector_offset[1])) #z to y >> ry
    pose.append(rot_vector[2]+rot_vector_offset[2]) #y to z >> rz
    
    print "==================================================="
    print "orientation is: "
    print pose[3:]
    print "+++++++++++++++++++++++++++++++++++++++++++++++++++="
    
    publish_pen_pose(pose)

    print("start moving...")
    robot.movel(pose, acc=0.2, vel=0.4, wait=True)

def publish_pen_pose(pose):

    pen_pose = PoseStamped()
    # header
    pen_pose.header.frame_id = "base_link"
    pen_pose.header.stamp = rospy.Time.now()

    # pose
    pen_pose.pose.position.x = pose[0]
    pen_pose.pose.position.y = pose[1]
    pen_pose.pose.position.z = pose[2]

    r = R.from_rotvec(pose[3:])
    quat = r.as_quat()

    pen_pose.pose.orientation.x = quat[0]
    pen_pose.pose.orientation.y = quat[1]
    pen_pose.pose.orientation.z = quat[2]
    pen_pose.pose.orientation.w = quat[3]

    pen_pose_pub.publish(pen_pose)


if __name__ == "__main__":
    try:  
        logging.basicConfig(level=logging.WARN)
        robot = urx.Robot("192.168.0.2")
        r_tcp = R.from_euler("xyz", [-60,0,0], degrees=True)
        tcp_rotvec = r_tcp.as_rotvec()
        robot.set_tcp((0, 0.048, 0.227, tcp_rotvec[0], tcp_rotvec[1], tcp_rotvec[2])) #unit: m
        robot.set_payload(1.2, [0, 0.03, 0.03]) #unit: kg, m
        time.sleep(1)
        # robot = urx.Robot("localhost")

        print("Going to start")
        startj = [-1.0369790236102503, -1.0823825041400355, -2.211877171193258, 0.15503239631652832, 1.0424913167953491, -0.0012467543231409195]
        robot.movej(startj, acc=0.8, vel=0.4, wait=True)
        print("Finish initialization")
        raw_input("Press any key to start")

        rospy.init_node('ur_imu', anonymous=True)
        rospy.Subscriber("raw_imu", Imu, callback_raw_imu, queue_size=1)
        pen_pose_pub = rospy.Publisher("imu_orientation", PoseStamped, queue_size=1)

        rospy.spin()

    finally:
        robot.close()
