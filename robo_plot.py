import numpy as np
import matplotlib.pyplot as plt
import modern_robotics as mr
from mpl_toolkits import mplot3d


#Each joint will be defined by a point, this is also the case for the 90deg angles that are present in the robot.
def robo_plot(thetalist):
    S = np.array([[0, 0 ,1, 0, 0, 0],
                 [0, 1, 0, -0.505, 0, 0.150],
                 [0, 1, 0, -1.265, 0, 0.150],
                 [1, 0, 0, 0, 1.465, 0],
                 [0, 1, 0, -1.465, 0, 0.945],
                 [1, 0, 0, 0, 1.465, 0]]).T

    
    M1 = np.identity(4)
    M2 = np.array([[1, 0, 0, 0.150],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.505],
                   [0, 0, 0, 1]])

    M3 = np.array([[1, 0, 0, 0.150],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.760+0.505],
                   [0, 0, 0, 1]])

    M4 = np.array([[1, 0, 0, 0.150+0.238],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.505+0.760+0.200],
                   [0, 0, 0, 1]])
    
    M5 = np.array([[1, 0, 0, 0.150+0.238+0.557],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.505+0.760+0.200],
                   [0, 0, 0, 1]])

    M6 = np.array([[1, 0, 0, 0.150+0.238+0.557+0.1],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.505+0.760+0.200],
                   [0, 0, 0, 1]])
    
    joint_frames = [M1, M2, M3, M4, M5, M6]
    trans_mat = [M1, M2, M3, M4, M5, M6]
    exp_trans = np.identity(4)

    for i in range(len(S)):
        trans_mat[i] = exp_trans@joint_frames[i]
        exp_trans = exp_trans@mr.MatrixExp6(mr.VecTose3(S[:,i]*thetalist[i])) 



    #Garther the x,y,z coorinates in arrays
    x_list = np.zeros(len(trans_mat))
    y_list = np.zeros(len(trans_mat))
    z_list = np.zeros(len(trans_mat))
    for i in range(len(trans_mat)):
        x_list[i] = trans_mat[i][0][3]
        y_list[i] = trans_mat[i][1][3]
        z_list[i] = trans_mat[i][2][3]


    #Plot the points in a 3d-coordinate system
    ax = plt.axes(projection='3d')
    ax.plot(x_list, y_list ,z_list ,'-') #Plot the robot links
    ax.plot(x_list, y_list, z_list, 'r.') #Plot the robot joints
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d(-2.1,2.1)
    ax.set_ylim3d(-2.1,2.1)
    ax.set_zlim3d(0,2.4) 
    plt.show()