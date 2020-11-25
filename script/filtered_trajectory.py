#!/usr/bin/env python
# encoding: utf-8

# read csv, filtered it by voxel grid and publish it in rviz
import sys  
sys.path.append('../urx/')  
import urx

import rospy
import logging
import numpy as np
import math
import time
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Bool
import geometry_msgs
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from scipy.spatial.transform import Rotation as R

#Global variables
#record ur pose list for execution
#pls commnet it when using realtime filter
PoseList_tip = geometry_msgs.msg.PoseArray()
PoseList_tip_filtered = geometry_msgs.msg.PoseArray()

#number to publish the trajecotry   
pub_num = 0
ur_pose_list = []
count = 1

# this function is used to represnt point cloud in voxel cube manner modified for 6dof trajecoty
# voxel grid works based on psoiton only the orientation information remain
# input parameters are: pc: point cloud numpy array each row corresponds to a xyz position
#                       pstart: min xyz value in voxel grid cube unit:m
#                       pend: max xyz value in voxel grid cube
#                       grid_sz: grid size
#                       verbose: for testing    
# output values are:    voxel_mat: binary 3d array with TF indicates if that grid is occupied or not
#                       pc_voxel: convert it back to point cloud
def pc_to_voxel_array(pc,
                      grid_sz = 1e-3,
                      verbose = False):
        
        pc = np.array(pc)
        #find the boundary of given trajectory, only using the first three col >> position info
        pstart = np.min(pc[:,:3], axis=0)
        pend = np.max(pc[:,:3], axis=0)
        if verbose: print pstart, pend

        # devide the cube along xyz based of grid_sz
        num_x = int(math.ceil((pend[0]-pstart[0])/grid_sz))
        num_y = int(math.ceil((pend[1]-pstart[1])/grid_sz))
        num_z = int(math.ceil((pend[2]-pstart[2])/grid_sz))

        # transform point cloud xyz to voxel grid
        if verbose: print('[INF] Quantization')
        pc0 = pc[:,:3]-np.array(pstart)                   # get the position of point could w.r.t. pstart
        pc_q = np.round(pc0/grid_sz).astype(int)          # transform it to voxel grid numbers

        # filtered out the points that are not included in voxel cube
        if verbose: print('[INF] Trimming')
        valid = (pc_q[:,0]>=0)*(pc_q[:,0]<num_x)*\
                (pc_q[:,1]>=0)*(pc_q[:,1]<num_y)*\
                (pc_q[:,2]>=0)*(pc_q[:,2]<num_z)
        pc_q_valid = pc_q[valid,:]

        # construct 3d array that stores voxel cube data
        voxel_mat = np.zeros((num_x,num_y,num_z), dtype=bool)
        if verbose: print('[INF] Constructing 3D array')
        for x,y,z in pc_q_valid: voxel_mat[x,y,z]=True

        # construct 3d array that stores the new converted poit cloud
        if verbose: print('[INF] Converting to point cloud')
        pc_voxel = pc_q*grid_sz + np.array(pstart)
        trajectory_voxel = np.append(pc_voxel, pc[:,3:], axis=1)
        # pc_voxel = np.unique(pc_voxel, axis=0)             # remove duplicated grid

        # # merge duplicated grid, average orientation
        # if verbose: print('[INF] Merge dupliations')
        # position, counts = np.unique(trajectory_voxel[:,:3], axis=0, return_counts=True)
        # dup_positions = position[counts>1]
        # while dup_positions.size!=0:
        #     dup_indice = np.where(trajectory_voxel[:,:3]==dup_positions[0])[0]
        #     ind_position, ind_counts = np.unique(dup_indice, axis=0, return_counts=True)
        #     dup_indice = ind_position[ind_counts==3]
        #     avg_pose = np.mean(trajectory_voxel[dup_indice,:], axis=0)
        #     trajectory_voxel = np.delete(trajectory_voxel, dup_indice, axis=0)
        #     trajectory_voxel = np.vstack((trajectory_voxel, avg_pose))
        #     position, counts = np.unique(trajectory_voxel[:,:3], axis=0, return_counts=True)
        #     dup_positions = position[counts>1]
        #     print dup_positions     
        #     print trajectory_voxel   
        #     print "==========" 

        return trajectory_voxel, voxel_mat

# write a function to filter it(smooth it)





def read_csv(path):
    csv_info = []

    csvFile = open(path, "r")
    reader = csv.reader(csvFile)

    for item in reader:
        csv_info.append(item)

    csvFile.close()

    pen_trajectory = []
    for i in range(len(csv_info)):
        if len(csv_info[i]) == 0:
            print "empty"
            continue

        p = []
        p.append(float(csv_info[i][0]))
        p.append(float(csv_info[i][1]))
        p.append(float(csv_info[i][2]))
        p.append(float(csv_info[i][3]))
        p.append(float(csv_info[i][4]))
        p.append(float(csv_info[i][5]))
        pen_trajectory.append(p)
        # print p

    return pen_trajectory

def write_csv(PC_path, trajectoryInfo):
    csvFile = open(PC_path, "w")
    writer = csv.writer(csvFile)
    for i in range(len(trajectoryInfo)):
        writer.writerow(trajectoryInfo[i])
    csvFile.close()

# def quat_traj_to_rot_traj(quat_trajectory):
# def quat_traj_to_rot_traj(quat_trajectory):

def callback_pen_tip_pose_trajectory(pen_tip_pose_trajectory):

    global count, ur_pose_list
    # do not do it everytime, the buffer is too large
    # filter it every twenty times
    ur_pose_list = []

    if count%5 == 0:

        for pen_tip_pose in pen_tip_pose_trajectory.poses:
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
            
            ur_pose_list.append(ur_pose)

        trajectory_voxel = pc_to_voxel_array(ur_pose_list,
                                                grid_sz=0.03,
                                                verbose=False)


        PoseList_tip_filtered = geometry_msgs.msg.PoseArray()
        for i in trajectory_voxel:
        
            pen_tip_pose_filtered = geometry_msgs.msg.Pose()
            pen_tip_pose_filtered.position.x = i[0] #unit:m
            pen_tip_pose_filtered.position.y = i[1]
            pen_tip_pose_filtered.position.z = i[2]

            r = R.from_rotvec(i[3:])
            Angle_quat = r.as_quat()
            pen_tip_pose_filtered.orientation.x = Angle_quat[0]
            pen_tip_pose_filtered.orientation.y = Angle_quat[1]
            pen_tip_pose_filtered.orientation.z = Angle_quat[2]
            pen_tip_pose_filtered.orientation.w = Angle_quat[3]
            PoseList_tip_filtered.poses.append(pen_tip_pose_filtered)
        
        PoseList_tip_filtered.header.frame_id = '/base_link'
        PoseList_tip_filtered.header.stamp = rospy.Time.now()
        pub_pen_pose_trajectory_filetered_realtime.publish(PoseList_tip_filtered)

    else:
        pass

    count += 1
        
    rate = rospy.Rate(1)
    rate.sleep()

def set_axes_equal(ax):
    """Set 3D plot axes to equal scale.

    Make axes of 3D plot have equal scale so that spheres appear as
    spheres and cubes as cubes.  Required since `ax.axis('equal')`
    and `ax.set_aspect('equal')` don't work on 3D.
    """
    limits = np.array([
        ax.get_xlim3d(),
        ax.get_ylim3d(),
        ax.get_zlim3d(),
    ])
    origin = np.mean(limits, axis=1)
    radius = 0.5 * np.max(np.abs(limits[:, 1] - limits[:, 0]))
    _set_axes_radius(ax, origin, radius)

def _set_axes_radius(ax, origin, radius):
    x, y, z = origin
    ax.set_xlim3d([x - radius, x + radius])
    ax.set_ylim3d([y - radius, y + radius])
    ax.set_zlim3d([z - radius, z + radius])

def publish_trajectory_static():

    # with open("/home/maggie/rosprojects/tool/src/guide-pen/Trajectory/Trajectory.csv") as csvfile:
    #     readCSV = csv.reader(csvfile, delimiter=',')
    #     print readCSV
    #     for row in readCSV:
    #         print row

   
    global pub_num
    print "using read csv function"
    pen_trajectory = read_csv("/home/rick/Documents/ros-projects/ur_guide/src/guide-pen/Trajectory/Trajectory5.csv")
    trajectory_voxel, voxel_mat = pc_to_voxel_array(pen_trajectory,
                                                    grid_sz=0.03,
                                                    verbose=True)
 
    # plot the voxel grid in 3d
    fig = plt.figure()
    ax = fig.gca(projection='3d')
    ax.voxels(voxel_mat, facecolors='orange', edgecolor='k')
    ax.set_aspect('equal')
    set_axes_equal(ax)
    plt.show()

    while not rospy.is_shutdown():
        if pub_num < 3:
            # row, col = np.shape(pen_trajectory)
            # print row, col

            # publish original trajectory in rviz
            for i in pen_trajectory:

                pen_tip_pose = geometry_msgs.msg.Pose()
                pen_tip_pose.position.x = i[0] #unit:m
                pen_tip_pose.position.y = i[1]
                pen_tip_pose.position.z = i[2]

                r = R.from_rotvec(i[3:])
                Angle_quat = r.as_quat()
                pen_tip_pose.orientation.x = Angle_quat[0]
                pen_tip_pose.orientation.y = Angle_quat[1]
                pen_tip_pose.orientation.z = Angle_quat[2]
                pen_tip_pose.orientation.w = Angle_quat[3]
                PoseList_tip.poses.append(pen_tip_pose)
            
            PoseList_tip.header.frame_id = '/base_link'
            PoseList_tip.header.stamp = rospy.Time.now()
            pub_pen_pose_trajectory.publish(PoseList_tip)

            # change each rotvec to quat for visualiation in rviz
            for i in trajectory_voxel:

                pen_tip_pose_filtered = geometry_msgs.msg.Pose()
                pen_tip_pose_filtered.position.x = i[0] #unit:m
                pen_tip_pose_filtered.position.y = i[1]
                pen_tip_pose_filtered.position.z = i[2]

                r = R.from_rotvec(i[3:])
                Angle_quat = r.as_quat()
                pen_tip_pose_filtered.orientation.x = Angle_quat[0]
                pen_tip_pose_filtered.orientation.y = Angle_quat[1]
                pen_tip_pose_filtered.orientation.z = Angle_quat[2]
                pen_tip_pose_filtered.orientation.w = Angle_quat[3]
                PoseList_tip_filtered.poses.append(pen_tip_pose_filtered)
            
            PoseList_tip_filtered.header.frame_id = '/base_link'
            PoseList_tip_filtered.header.stamp = rospy.Time.now()
            pub_pen_pose_trajectory_filetered.publish(PoseList_tip_filtered)
            
            rate = rospy.Rate(1)
            rate.sleep()
            
            pub_num += 1
            print pub_num
        else:
            rospy.loginfo("end publishing")
            rospy.signal_shutdown("shut down")

 
if __name__=='__main__': 

    logging.basicConfig(level=logging.WARN)
    rospy.init_node('filter_tracjectory', anonymous=True)
    # sub_pen_tip_pose_trajectory = rospy.Subscriber("pen_tip_pose_trajectory", PoseArray, callback_pen_tip_pose_trajectory, queue_size=1)

    pub_pen_pose_trajectory = rospy.Publisher("pen_tip_pose_trajectory", PoseArray, queue_size=1)
    pub_pen_pose_trajectory_filetered = rospy.Publisher("pen_tip_pose_trajectory_filtered", PoseArray, queue_size=1)
    # pub_pen_pose_trajectory_filetered_realtime = rospy.Publisher("pen_tip_pose_trajectory_filtered_realtime", PoseArray, queue_size=1)
    publish_trajectory_static()
    rospy.spin()