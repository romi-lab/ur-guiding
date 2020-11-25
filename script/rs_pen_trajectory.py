#!/usr/bin/env python
# encoding: utf-8

# combine ar position and imu orientation
# ur moves simutiouly with pen pose

import sys  
sys.path.append('../urx/')  
import urx

import rospy
import logging
import numpy as np
from numpy import pi
from sensor_msgs.msg import Imu
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from visualization_msgs.msg import Marker, MarkerArray
import math
import time
from scipy.spatial.transform import Rotation as R
# import serial
# from serial import Serial

#Global variables

#angle_in_quat
Angle_quat = [1.0]*4

#ensure imu is record first
record_imu = False

#record pose to a list
# ar marker side
#for real execution
PoseList = geometry_msgs.msg.PoseArray()
#for visualization
PoseList_rviz = geometry_msgs.msg.PoseArray()

# pen tip side
#for real execution
PoseList_tip = geometry_msgs.msg.PoseArray()
#for visualization
PoseList_tip_rviz = geometry_msgs.msg.PoseArray()

PoseList_cam_rviz = geometry_msgs.msg.PoseArray()

#publish current pen tip pose marker array for visualization only
tip_marker_array_rviz = MarkerArray()

#record ur pose list for execution
ur_pose_list = []

def callback_raw_imu(raw_imu):

    # print "in imu"
    
    global Angle_quat, record_imu

    Angle_quat[0] = raw_imu.orientation.x
    Angle_quat[1] = raw_imu.orientation.y
    Angle_quat[2] = raw_imu.orientation.z
    Angle_quat[3] = raw_imu.orientation.w
    record_imu = True


def callback_VisuoGuding_Pose(ar_initial_pose):

    print "in visual guiding pose"
    global record_imu, PoseList, PoseList_rviz, PoseList_tip, PoseList_tip_rviz, tip_marker_array_rviz, ur_pose_list, PoseList_cam_rviz

    if math.isnan(ar_initial_pose.position.x) == False and record_imu == True:

        #fix the position erro
        # ar_pose_offset = [-0.06,-0.12,0.11] unit:m
        ar_pose_offset = [0,0,0]
        ar_pose = geometry_msgs.msg.Pose()
        ar_pose.position.x = ar_initial_pose.position.x + ar_pose_offset[0]
        ar_pose.position.y = ar_initial_pose.position.y + ar_pose_offset[1]
        ar_pose.position.z = ar_initial_pose.position.z + ar_pose_offset[2]

        global Angle_quat

        #publish penpose for visualization 
        pen_pose_rviz = geometry_msgs.msg.Pose()
        #rviz pen position
        pen_pose_rviz.position.x = ar_pose.position.x
        pen_pose_rviz.position.y = ar_pose.position.y
        pen_pose_rviz.position.z = ar_pose.position.z
        #rviz pen orientation >> rotation angle in x and y axis should be inverse
        r = R.from_quat(Angle_quat)
        Angle = r.as_euler("zyx", degrees=True) #zyx
        Angle_quat_rviz = euler_to_quaternion(Angle[2], Angle[1], Angle[0]+180)
        pen_pose_rviz.orientation.x = Angle_quat_rviz[0]
        pen_pose_rviz.orientation.y = Angle_quat_rviz[1]
        pen_pose_rviz.orientation.z = Angle_quat_rviz[2]
        pen_pose_rviz.orientation.w = Angle_quat_rviz[3]
        #publish current penpose
        pub_pen_pose_rviz.publish(pen_pose_rviz)
        #publish penpose trajectory for visualization
        PoseList_rviz.poses.append(pen_pose_rviz)
        PoseList_rviz.header.frame_id = 'base_link'
        PoseList_rviz.header.stamp = rospy.Time.now()
        pub_pen_pose_trajectory_rviz.publish(PoseList_rviz)

        #publihs pose for execution
        pen_pose = geometry_msgs.msg.Pose()
        #pen position
        pen_pose.position.x = -ar_pose.position.x
        pen_pose.position.y = -ar_pose.position.y
        pen_pose.position.z = ar_pose.position.z
        #pen orientation
        pen_pose.orientation.x = Angle_quat[0]
        pen_pose.orientation.y = Angle_quat[1]
        pen_pose.orientation.z = Angle_quat[2]
        pen_pose.orientation.w = Angle_quat[3]
        #publish current penpose for moving
        pub_pen_pose.publish(pen_pose)
        #publish penpose trajectory
        PoseList.poses.append(pen_pose)
        PoseList.header.frame_id = 'base_link'
        PoseList.header.stamp = rospy.Time.now()
        pub_pen_pose_trajectory.publish(PoseList)


        #publish current penpose marker for visualization
        ar_marker = Marker()
        ar_marker.header.frame_id = "base_link"
        ar_marker.header.stamp = rospy.Time.now()
        #ar_marker.ns = "my_namespace"
        ar_marker.id = 0
        ar_marker.type = Marker.CUBE
        ar_marker.action = Marker.ADD
        ar_marker.pose.position.x = ar_pose.position.x
        ar_marker.pose.position.y = ar_pose.position.y
        ar_marker.pose.position.z = ar_pose.position.z
        ar_marker.pose.orientation.x = Angle_quat_rviz[0]
        ar_marker.pose.orientation.y = Angle_quat_rviz[1]
        ar_marker.pose.orientation.z = Angle_quat_rviz[2]
        ar_marker.pose.orientation.w = Angle_quat_rviz[3]
        ar_marker.scale.x = 0.02
        ar_marker.scale.y = 0.02
        ar_marker.scale.z = 0.02
        ar_marker.color.a = 1
        ar_marker.color.r = 0
        ar_marker.color.g = 0
        ar_marker.color.b = 0
        pub_pen_ar_marker.publish(ar_marker)


        #transform position from ar cube side to pen tip based on current 6dof info
        #get pen tip position
        r = R.from_quat(Angle_quat)

        # for testing , this is wrong, euler angle is the rotation of axis, not angle of axis
        # Angle_tip = r.as_euler("zyx", degrees=True) #zyx
        # print "Angle tip xyz", Angle_tip[2], Angle_tip[1], Angle_tip[0]
        # # Angle_tip_quat = euler_to_quaternion(-Angle_tip[2]+180, -Angle_tip[1], Angle_tip[0])
        # Angle_tip_quat = euler_to_quaternion(Angle_tip[2]-180, Angle_tip[1], Angle_tip[0])
        # print "New Angle tip xyz", Angle_tip[2]-180, Angle_tip[1], Angle_tip[0]
        # r_tip = R.from_quat(Angle_tip_quat)

        tip_rot_vector = r.as_rotvec()
        # for debugging
        # tip_rot_vector_offset = [0,0,0]
        # tip_rot_vector = np.add(tip_rot_vector, tip_rot_vector_offset)
        r_tip = R.from_rotvec(tip_rot_vector)
        Rot_matrix = r_tip.as_dcm()
        new_z = Rot_matrix[:,2]
        #the length of pen 0.22m, cube dim: 0.02*0.02 unit:m
        # pen_length = 0.19 #planer
        pen_length = 0.22 #yshape
        offset = 0
        # offset = 0.22 for setting camera as tcp
        pen_length = pen_length - offset

        displacement = pen_length*new_z
        pen_tip_position = np.zeros(3)
        pen_tip_position[0] = pen_pose.position.x - displacement[0]
        pen_tip_position[1] = pen_pose.position.y - displacement[1]
        pen_tip_position[2] = pen_pose.position.z + displacement[2]




        # new_y = Rot_matrix[:,1]
        new_y = np.array([0,1,0])
        distance = 0.5 #distance

        y_displacement = distance*new_y
        cam_position = np.zeros(3)
        cam_position[0] = pen_pose.position.x - y_displacement[0]
        cam_position[1] = pen_pose.position.y + y_displacement[1]
        cam_position[2] = pen_pose.position.z - y_displacement[2]

        # print cam_position


        #publish to cam_pose
        cam_pose = geometry_msgs.msg.Pose()
        #cam position
        cam_pose.position.x = cam_position[0]
        cam_pose.position.y = cam_position[1]
        cam_pose.position.z = cam_position[2]
        #cam orientation
        r_cam = R.from_rotvec([0, 2.2, -2.2])
        cam_quat = r_cam.as_quat()
        print cam_quat

        cam_pose.orientation.x = cam_quat[0]
        cam_pose.orientation.y = cam_quat[1]
        cam_pose.orientation.z = cam_quat[2]
        cam_pose.orientation.w = cam_quat[3]
        #publish cam pose for moving
        pub_cam_pose.publish(cam_pose)

        
        #publish to cam_pose
        cam_pose_rviz = geometry_msgs.msg.Pose()
        #cam position
        cam_pose_rviz.position.x = -cam_pose.position.x
        cam_pose_rviz.position.y = -cam_pose.position.y
        cam_pose_rviz.position.z = cam_pose.position.z
        #cam orientation
        cam_pose_rviz.orientation.x = cam_quat[0]
        cam_pose_rviz.orientation.y = cam_quat[1]
        cam_pose_rviz.orientation.z = cam_quat[2]
        cam_pose_rviz.orientation.w = cam_quat[3]
        #publish cam pose for moving
        pub_cam_pose_rviz.publish(cam_pose_rviz)

        PoseList_cam_rviz.poses.append(cam_pose_rviz)
        PoseList_cam_rviz.header.frame_id = '/base_link'
        PoseList_cam_rviz.header.stamp = rospy.Time.now()
        pub_cam_pose_array_rviz.publish(PoseList_cam_rviz)




        # print "new_z is: ", new_z
        # print "displacement", displacement

        #publish to pen_tip_pose
        pen_tip_pose = geometry_msgs.msg.Pose()
        #pen tip position
        pen_tip_pose.position.x = pen_tip_position[0]
        pen_tip_pose.position.y = pen_tip_position[1]
        pen_tip_pose.position.z = pen_tip_position[2]
        #pen tip orientation
        pen_tip_pose.orientation.x = Angle_quat[0]
        pen_tip_pose.orientation.y = Angle_quat[1]
        pen_tip_pose.orientation.z = Angle_quat[2]
        pen_tip_pose.orientation.w = Angle_quat[3]
        #publish current pen tip pose for moving
        pub_pen_tip_pose.publish(pen_tip_pose)
        #publish pen tip pose trajectory
        PoseList_tip.poses.append(pen_tip_pose)
        PoseList_tip.header.frame_id = '/base_link'
        PoseList_tip.header.stamp = rospy.Time.now()
        pub_pen_tip_pose_trajectory.publish(PoseList_tip)

        # print "ar position:\n", ar_pose.position
        # print "pen tip position:\n", pen_tip_pose.position



        #publish pen tip pose for visualization  >> the position is inverted in rviz, so manually invert back to achieve visualization
        #publish to pen_tip_pose
        pen_tip_pose_rviz = geometry_msgs.msg.Pose()
        #pen tip position
        pen_tip_pose_rviz.position.x = -pen_tip_position[0]
        pen_tip_pose_rviz.position.y = -pen_tip_position[1]
        pen_tip_pose_rviz.position.z = pen_tip_position[2]
        #pen tip orientation
        pen_tip_pose_rviz.orientation.x = Angle_quat_rviz[0]
        pen_tip_pose_rviz.orientation.y = Angle_quat_rviz[1]
        pen_tip_pose_rviz.orientation.z = Angle_quat_rviz[2]
        pen_tip_pose_rviz.orientation.w = Angle_quat_rviz[3]
        #publish current pen tip pose for moving
        pub_pen_tip_pose_rviz.publish(pen_tip_pose_rviz)
        #publish pen tip pose trajectory
        PoseList_tip_rviz.poses.append(pen_tip_pose_rviz)
        PoseList_tip_rviz.header.frame_id = '/base_link'
        PoseList_tip_rviz.header.stamp = rospy.Time.now()
        pub_pen_tip_pose_trajectory_rviz.publish(PoseList_tip_rviz)

        
        
        #publish current pen tip pose marker for visualization
        tip_marker = Marker()
        tip_marker.header.frame_id = "base_link"
        tip_marker.header.stamp = rospy.Time.now()
        #tip_marker.ns = "my_namespace"
        tip_marker.id = 0
        tip_marker.type = Marker.CUBE
        tip_marker.action = Marker.ADD
        tip_marker.pose.position.x = -pen_tip_position[0]
        tip_marker.pose.position.y = -pen_tip_position[1]
        tip_marker.pose.position.z = pen_tip_position[2]
        tip_marker.pose.orientation.x = Angle_quat_rviz[0]
        tip_marker.pose.orientation.y = Angle_quat_rviz[1]
        tip_marker.pose.orientation.z = Angle_quat_rviz[2]
        tip_marker.pose.orientation.w = Angle_quat_rviz[3]
        tip_marker.scale.x = 0.01
        tip_marker.scale.y = 0.01
        tip_marker.scale.z = 0.01
        tip_marker.color.a = 1
        tip_marker.color.r = 1
        tip_marker.color.g = 1
        tip_marker.color.b = 0
        pub_pen_tip_marker.publish(tip_marker)

        #publish current pen tip pose marker array for visualization
        tip_marker_array_rviz.markers.append(tip_marker)
        id = 0
        for m in tip_marker_array_rviz.markers:
            m.id = id
            id += 1
        pub_pen_tip_marker_array.publish(tip_marker_array_rviz)



        #publish current whole pen marker for visualization
        pen_marker = Marker()
        pen_marker.header.frame_id = "base_link"
        pen_marker.header.stamp = rospy.Time.now()
        #pen_marker.ns = "my_namespace"
        pen_marker.id = 0
        pen_marker.type = Marker.CUBE
        pen_marker.action = Marker.ADD
        pen_marker.pose.position.x = -(pen_tip_position[0]+pen_pose.position.x)/2
        pen_marker.pose.position.y = -(pen_tip_position[1]+pen_pose.position.y)/2
        pen_marker.pose.position.z = (pen_tip_position[2]+pen_pose.position.z)/2
        pen_marker.pose.orientation.x = Angle_quat_rviz[0]
        pen_marker.pose.orientation.y = Angle_quat_rviz[1]
        pen_marker.pose.orientation.z = Angle_quat_rviz[2]
        pen_marker.pose.orientation.w = Angle_quat_rviz[3]
        pen_marker.scale.x = 0.02
        pen_marker.scale.y = 0.02
        pen_marker.scale.z = 0.22
        pen_marker.color.a = 1
        pen_marker.color.r = 1
        pen_marker.color.g = 1
        pen_marker.color.b = 1
        pub_pen_marker.publish(pen_marker)



        #make a target pose for ur to move
        # ur_pose = []

        # ur_pose.append(pen_tip_pose.position.x)              #px
        # ur_pose.append(pen_tip_pose.position.y)              #py
        # ur_pose.append(pen_tip_pose.position.z)              #pz

        # r = R.from_quat(Angle_quat)
        # rot_vector = r.as_rotvec()
        # rot_vector_offset = [0,0,0]
        # ur_pose.append(rot_vector[0]+rot_vector_offset[0]) #rx
        # ur_pose.append(-(rot_vector[1]+rot_vector_offset[1])) #z to y >> ry
        # ur_pose.append(rot_vector[2]+rot_vector_offset[2]) #y to z >> rz

        # ur_pose_list.append(ur_pose)


        #make a target pose for scan following
        ur_pose = []

        ur_pose.append(cam_pose.position.x)              #px
        ur_pose.append(cam_pose.position.y)              #py
        ur_pose.append(cam_pose.position.z)              #pz
        
        current_pose = robot.getl()
        ur_pose.append(current_pose[3]) #rx
        ur_pose.append(current_pose[4]) #z to y >> ry
        ur_pose.append(current_pose[5]) #y to z >> rz

        # r = R.from_quat(cam_quat)
        # rot_vector = r.as_rotvec()
        # rot_vector_offset = [0,0,0]
        # ur_pose.append(rot_vector[0]+rot_vector_offset[0]) #rx
        # ur_pose.append(rot_vector[1]+rot_vector_offset[1]) #z to y >> ry
        # ur_pose.append(rot_vector[2]+rot_vector_offset[2]) #y to z >> rz

        ur_pose_list.append(ur_pose)

        print "==================================================="
        print "pen pose is: "
        print ur_pose

        print("start moving...")
        robot.movep(ur_pose, acc=0.01, vel=0.02, wait=False)  #set wait=false to for realtime tracking, wait=true will create delay
        # safe vel 0.05
        # safe acc 0.02 

    #     # robot.movel(ur_pose, acc=0.4, vel=0.8, wait=True) # for movel, if wait = false bad performance >> create mutiple pause in motion
    #     rospy.sleep(1)
    
    rate = rospy.Rate(10) # 5 Hz for following, 10 Hz for filter trajecotry, change the publihs frequency of imu to make it compatabile
    #10 Hz only for drawing
    # Do stuff, maybe in a while loop
    rate.sleep() # Sleeps for 1/rate sec


def euler_to_quaternion(roll, pitch, yaw):
    
    # change to radius
    yaw = yaw*pi/180
    pitch = pitch*pi/180
    roll = roll*pi/180

    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)

    return qx, qy, qz, qw

if __name__=='__main__': 
    try:  
        logging.basicConfig(level=logging.WARN)
        
        # robot = urx.Robot("localhost")

        robot = urx.Robot("localhost")
        r_tcp = R.from_euler("xyz", [-60,0,0], degrees=True)
        tcp_rotvec = r_tcp.as_rotvec()

        # set tcp as end for testing
        robot.set_tcp((0, 0, 0, 0, 0, 0)) #unit: m

        #set tcp as torch
        # robot.set_tcp((0, 0.048, 0.227, tcp_rotvec[0], tcp_rotvec[1], tcp_rotvec[2])) #unit: m\

        #set tcp as camera for scanning
        # robot.set_tcp((0, -0.120, 0.070,0,0,0)) #unit: m
        robot.set_payload(1.2, [0, 0.03, 0.03]) #unit: kg, m
        time.sleep(1)

        # print robot.getj()
        # print robot.getl()
        # raw_input("Press any key to start reocrd pose")

        print("Going to start")
        startj = [-0.5432379881488245, -0.7106049696551722, -2.332104269658224, -0.09274894395937139, 0.547268271446228, -0.006056133900777638]
        # startj = [-1.0369790236102503, -1.0823825041400355, -2.211877171193258, 0.15503239631652832, 1.0424913167953491, -0.0012467543231409195]
        robot.movej(startj, acc=0.8, vel=0.4, wait=False)
        print("Finish initialization")
        raw_input("Press any key to start reocrd pose")

        rospy.init_node('record_tracjectory', anonymous=True)

        sub_pen_orientation = rospy.Subscriber("raw_imu", Imu, callback_raw_imu, queue_size=1)
        sub_pen_position = rospy.Subscriber("ar_transformed_pose", Pose, callback_VisuoGuding_Pose, queue_size=1)

        pub_pen_pose = rospy.Publisher("pen_pose", Pose, queue_size=1)
        pub_pen_pose_rviz = rospy.Publisher("pen_pose_rviz", Pose, queue_size=1)
        pub_pen_tip_pose = rospy.Publisher("pen_tip_pose", Pose, queue_size=1)
        pub_pen_tip_pose_rviz = rospy.Publisher("pen_tip_pose_rviz", Pose, queue_size=1)

        pub_pen_pose_trajectory = rospy.Publisher("pen_pose_trajectory", PoseArray, queue_size=1)
        pub_pen_pose_trajectory_rviz = rospy.Publisher("pen_pose_trajectory_rviz", PoseArray, queue_size=1)
        pub_pen_tip_pose_trajectory = rospy.Publisher("pen_tip_pose_trajectory", PoseArray, queue_size=1)
        pub_pen_tip_pose_trajectory_rviz = rospy.Publisher("pen_tip_pose_trajectory_rviz", PoseArray, queue_size=1)

        pub_pen_ar_marker = rospy.Publisher("pen_ar_marker", Marker, queue_size=1) #vislaizse ar marker as a cube
        pub_pen_tip_marker = rospy.Publisher("pen_tip_marker", Marker, queue_size=1) #vislaizse pen tip as a cube
        pub_pen_marker = rospy.Publisher("pen_marker", Marker, queue_size=1) #vislaizse whole pen as
        pub_pen_tip_marker_array = rospy.Publisher("pen_tip_marker_array", MarkerArray, queue_size=1) #vislaizse pen tip trajectory for drawing

        pub_cam_pose = rospy.Publisher("cam_pose", Pose, queue_size=1) 
        pub_cam_pose_rviz = rospy.Publisher("cam_pose_rviz", Pose, queue_size=1) 
        pub_cam_pose_array_rviz  = rospy.Publisher("cam_pose_array_rviz", PoseArray, queue_size=1)

        rospy.spin()

    finally:
        robot.close()
        print("Shutting down")
