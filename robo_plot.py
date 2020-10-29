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
    
    M7 = np.array([[1, 0, 0, 0.150+0.238+0.557+0.1],
                   [0, 1, 0, 0],
                   [0, 0, 1, 0.505+0.760+0.200],
                   [0, 0, 0, 1]])
    
    joint_frames = [M1, M2, M3, M4, M5, M6, M7] #M1 - M& is a frame attached to each joint, M7 is the end effector frame
    trans_mat = [M1, M2, M3, M4, M5, M6, M7]
    exp_trans = np.identity(4)

    for i in range(len(S)):
        trans_mat[i] = exp_trans@joint_frames[i]
        exp_trans = exp_trans@mr.MatrixExp6(mr.VecTose3(S[:,i]*thetalist[i]))
        if i == range(len(S))[-1]:
            trans_mat[i+1] = exp_trans@joint_frames[i+1] 

    Tr = np.radians(54)
    T = np.array([[np.cos(Tr), 0, np.sin(Tr), 0.450], #The transformation matrix between the end frame and the tool-tip
                  [0, 1 ,0, 0],
                  [-np.sin(Tr), 0, np.cos(Tr), -0.084],
                  [0, 0, 0, 1]])

    trans_mat[-1] = trans_mat[-1]@T

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
    plt.title('YASKAWA Motoman GP25')
    ax.plot(x_list, y_list ,z_list ,'-k') #Plot the robot links
    ax.plot(x_list[0:len(x_list)-1], y_list[0:len(y_list)-1], z_list[0:len(z_list)-1        ], 'c.') #Plot the robot joints
    color_list= ['-b', '-r', '-g']
    c_scale = 0.3 #Used to scale the size of the coordinate axis
    I = np.identity(3)*c_scale 
    for i in range(3): #Plots coordinate axis for the base frame and end effector frame
        ax.plot([0, I[0][i]], [0, I[1][i]], [0, I[2][i]], color_list[i])
        ax.plot([x_list[-1], x_list[-1] + trans_mat[-1][0][i]*c_scale], [y_list[-1], y_list[-1] + trans_mat[-1][1][i]*c_scale], [z_list[-1], z_list[-1] + trans_mat[-1][2][i]*c_scale], color_list[i])


    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim3d(-2.1,2.1)
    ax.set_ylim3d(-2.1,2.1)
    ax.set_zlim3d(0,2.4) 
    plt.show()