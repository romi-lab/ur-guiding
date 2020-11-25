#!/usr/bin/env python
# encoding: utf-8

# combine ar position and imu orientation
# ur moves after trajectory is generated

import sys  
sys.path.append('../urx/')  
import urx

import rospy
import logging
import numpy as np
from std_msgs.msg import Bool
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
import math
import time
import csv
from scipy.spatial.transform import Rotation as R

#Global variables
#record ur pose list for execution
ur_pose_list_tip = []
ur_pose_list_cam = []


def callback_execute_trajectory(execution):

    global ur_pose_list_tip, ur_pose_list_cam, robot
    print "execution is : ", execution

    if execution.data == True:

        print("start scanning...")
        robot.set_tcp((0, -0.120, 0.070,0,0,0)) #unit: m
        robot.set_payload(1.2, [0, 0.03, 0.03]) #unit: kg, m
        time.sleep(1)
        # robot.movep(ur_pose_list_cam[0], acc=0.05, vel=0.1, wait=True)
        # robot.movels(ur_pose_list_cam, acc=0.1, vel=0.2, wait=True)
        # # # for ur_pose in ur_pose_list_tip:
        # # #     robot.movep(ur_pose, acc=0.1, vel=0.2, wait=False)

        
        robot.movel(ur_pose_list_tip[0], acc=0.05, vel=0.1, wait=True)
        robot.movels(ur_pose_list_tip, acc=0.1, vel=0.2, wait=True)
        raw_input("mission complete, press any key to continue for torch")

        # with open('Trajectory2.csv','w') as file: # open the file, if it exists overwrite it, if it doesn't create it.
        #     writer = csv.writer(file)
        #     # writer = csv.writer(file, delimiter=',')
        #     for ur_pose in ur_pose_list_tip: # iterate through all the data
        #         writer.writerow(ur_pose) # write data line to the open file 
        #     # with closes file automatically on exiting block
        # print "file written success!"


    else:
        print "mission cancelled"

def callback_pen_tip_pose_trajectory(pen_tip_pose):

    print "recording trajectory"
    global ur_pose_list_tip

    ur_pose = [0.0]*6

    ur_pose[0] = pen_tip_pose.position.x
    ur_pose[1] = pen_tip_pose.position.y
    ur_pose[2] = pen_tip_pose.position.z
    #pen tip orientation
    Angle_quat = [0.0]*4
    Angle_quat[0] = pen_tip_pose.orientation.x
    Angle_quat[1] = pen_tip_pose.orientation.y
    Angle_quat[2] = pen_tip_pose.orientation.z
    Angle_quat[3] = pen_tip_pose.orientation.w

    r = R.from_quat(Angle_quat)
    rot_vector = r.as_rotvec()
    rot_vector_offset = [0,0,0]
    ur_pose[3] = (rot_vector[0]+rot_vector_offset[0]) #rx
    ur_pose[4] = -(rot_vector[1]+rot_vector_offset[1]) #z to y >> ry
    ur_pose[5] = (rot_vector[2]+rot_vector_offset[2]) #y to z >> rz

    ur_pose_list_tip.append(ur_pose)
    # print "ur pose list", ur_pose_list_tip

def callback_cam_pose_trajectory(cam_pose):
    
    print "recording cam trajectory"
    global ur_pose_list_cam

    ur_pose = [0.0]*6

    ur_pose[0] = cam_pose.position.x
    ur_pose[1] = cam_pose.position.y
    ur_pose[2] = cam_pose.position.z
    #pen tip orientation
    Angle_quat = [0.0]*4
    Angle_quat[0] = cam_pose.orientation.x
    Angle_quat[1] = cam_pose.orientation.y
    Angle_quat[2] = cam_pose.orientation.z
    Angle_quat[3] = cam_pose.orientation.w

    r = R.from_quat(Angle_quat)
    rot_vector = r.as_rotvec()
    rot_vector_offset = [0,0,0]
    ur_pose[3] = (rot_vector[0]+rot_vector_offset[0]) #rx
    ur_pose[4] = -(rot_vector[1]+rot_vector_offset[1]) #z to y >> ry
    ur_pose[5] = (rot_vector[2]+rot_vector_offset[2]) #y to z >> rz

    ur_pose_list_cam.append(ur_pose)

if __name__=='__main__': 
    try:  
        logging.basicConfig(level=logging.WARN)
        
        # robot = urx.Robot("localhost")

        robot = urx.Robot("192.168.0.2")
        # r_tcp = R.from_euler("xyz", [-60,0,0], degrees=True)
        # tcp_rotvec = r_tcp.as_rotvec()
        # robot.set_tcp((0, 0.048, 0.227, tcp_rotvec[0], tcp_rotvec[1], tcp_rotvec[2])) #unit: m
        robot.set_payload(1.2, [0, 0.03, 0.03]) #unit: kg, m
        # time.sleep(1)

        # print("Going to start")
        # startj = [-1.0369790236102503, -1.0823825041400355, -2.211877171193258, 0.15503239631652832, 1.0424913167953491, -0.0012467543231409195]
        # robot.movej(startj, acc=0.8, vel=0.4, wait=True)
        # print("Finish initialization")
        # raw_input("Press any key to start")

        rospy.init_node('execute_tracjectory', anonymous=True)

        sub_pen_tip_pose = rospy.Subscriber("pen_tip_pose", Pose, callback_pen_tip_pose_trajectory, queue_size=1)
        sub_pen_pose = rospy.Subscriber("pen_pose", Pose, callback_cam_pose_trajectory, queue_size=1)
        sub_execute_trajectory = rospy.Subscriber("magic", Bool, callback_execute_trajectory, queue_size=1)

        rospy.spin()

    finally:
        robot.close()
        print("Shutting down")
