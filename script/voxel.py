
import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

# this function is used to represnt point cloud in voxel cube manner
# input parameters are: pc: point cloud numpy array each row corresponds to a xyz position
#                       pstart: min xyz value in voxel grid cube
#                       pend: max xyz value in voxel grid cube
#                       grid_sz: grid size
#                       verbose: for testing    
# output values are:    voxel_mat: binary 3d array with TF indicates if that grid is occupied or not
#                       pc_voxel: convert it back to point cloud
def pc_to_voxel_array(pc,
                      pstart = (0.0,0.0,0.0),
                      pend = (1.0,1.0,1.0),
                      grid_sz = 0.5e-2,
                      verbose = False):
                
        # devide the cube along xyz based of grid_sz
        num_x = int(math.ceil((pend[0]-pstart[0])/grid_sz))
        num_y = int(math.ceil((pend[1]-pstart[1])/grid_sz))
        num_z = int(math.ceil((pend[2]-pstart[2])/grid_sz))

        # transform point cloud xyz to voxel grid
        if verbose: print('[INF] Quantization')
        pc0 = pc-np.array(pstart)                         # get the position of point could w.r.t. pstart
        pc_q = np.round(pc0/grid_sz).astype(int)          # transform it to voxel grid numbers

        # filtered out the points that are not included in voxel cube
        if verbose: print('[INF] Trimming')
        valid = (pc_q[:,0]>=0)*(pc_q[:,0]<num_x)*\
                (pc_q[:,1]>=0)*(pc_q[:,1]<num_y)*\
                (pc_q[:,2]>=0)*(pc_q[:,2]<num_z)
        pc_q = pc_q[valid,:]

        # construct 3d array that stores voxel cube data
        voxel_mat = np.zeros((num_x,num_y,num_z), dtype=bool)
        if verbose: print('[INF] Constructing 3D array')
        for x,y,z in pc_q: voxel_mat[x,y,z]=True

        # construct 3d array that stores the new converted poit cloud
        if verbose: print('[INF] Converting to point cloud')
        pc_voxel = pc_q*grid_sz + np.array(pstart)
        pc_voxel = np.unique(pc_voxel, axis=0)             # remove duplicated grid

        return voxel_mat, pc_voxel


# for testing
N = 5000
pc = np.random.rand(N,3)*2.0-1.0
pc = pc/np.linalg.norm(pc,axis=1).reshape(N,1)*0.5

# plot the point cloud in 3d
ax = plt.figure(1).gca(projection='3d')
ax.plot(pc[:,0],pc[:,1],pc[:,2],'b.',markersize=0.5)
plt.show()

# construct voxel grid matrix
voxel_mat, pc_voxel=pc_to_voxel_array(pc,
                                      pstart=(-1,-1,-1),
                                      pend  =( 1, 1, 1),
                                      grid_sz=0.05,
                                      verbose=True)

# plot the voxel grid in 3d
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.voxels(voxel_mat, facecolors='b', edgecolor='k')
plt.show()

# plot the point cloud in 3d after voxel grid convertion
fig = plt.figure()
ax = fig.gca(projection='3d')
ax.plot(pc_voxel[:,0],pc_voxel[:,1],pc_voxel[:,2],'r.',markersize=2)
plt.show()