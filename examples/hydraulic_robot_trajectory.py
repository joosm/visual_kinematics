#!/usr/bin/env python3

from visual_kinematics.RobotSerial import *
from visual_kinematics.RobotTrajectory import *
import numpy as np
from math import pi
import csv
import robot_kinematics_tools as RKT
from ctypes import cdll, c_int, POINTER, c_double

def get_data_from_csv_20layers(path_to_csv_file):
    f = open(path_to_csv_file, 'r', newline = '') #, encoding='utf-8')
    csv_reader = csv.reader(f)
    temp_data = []    
    line_no = 0
    for line in csv_reader:
        if line_no == 0:
            line_no += 1
        else:
            temp_data.append([float(f) for f in line[1:23]])
            line_no += 1
    print(path_to_csv_file,' ',len(temp_data),' ',line_no-1) 

    temp_data_array = np.asarray(temp_data, dtype=np.float32)

    X = temp_data_array[:,6:12] #x,y,z,rx,ry,rz
    Y = temp_data_array[:,0:6] #theta1, ... , theta6

    return X, Y

def get_data_from_csv(path_to_csv_file):
    f = open(path_to_csv_file, 'r', newline = '') #, encoding='utf-8')
    csv_reader = csv.reader(f)
    temp_data = []    
    line_no = 0
    for line in csv_reader:
        if line_no == 0:
            line_no += 1
        else:
            temp_data.append([float(f) for f in line[1:13]])
            line_no += 1
    print(path_to_csv_file,' ',len(temp_data),' ',line_no-1) 

    temp_data_array = np.asarray(temp_data,dtype=np.float32)

    X = temp_data_array[:,6:12] #x,y,z,rx,ry,rz
    Y = temp_data_array[:,0:6] #theta1, ... , theta6

    return X, Y

#data_file = "/home/joosm/Workplace/Hydraulic_robot/hybrid_calibration/training_data_joo/simulated_training_data_theta_quaternion_test_20210930_20layers.csv"
#X,Y = get_data_from_csv_20layers(data_file)

data_file = "/home/joosm/Workplace/Hydraulic_robot/hybrid_calibration/training_data_hyun_simulation/HydRobotLearningPt_csv.csv"
#data_file = "/home/joosm/Workplace/Hydraulic_robot/hybrid_calibration/training_data_joo/simulated_training_data_test.csv"
X,Y = get_data_from_csv(data_file)
#X: x,y,z,rx,ry,rz
#Y: theta1, ... , theta6 -- in degree

X_data_temp = X #[]

T_Base_0 = [[ -0.04626181,  -0.99892327,  -0.00348464,  1346.71285/1000],
            [  0.99892232,  -0.04627428,   0.00358895, -1561.24376/1000],
            [ -0.00374633,  -0.00331485,   0.99998749,   830.66027/1000],
            [  0.        ,   0.        ,   0.        ,     1.     ]]
trGripper = RKT.transform_from_translation_only([0.0, 160.0/1000, 340.0/1000])
T_6_TCP = trGripper

#a,alpha,beta,d,theta
HR_MDH_parameter_nominal = [[ 0,   0, 0, 0,  0], #T_0_1
[245.5, -np.pi/2.0, 0, 0,      0], #T_1_2
[1300,   0,         0, 0,      0], #T_2_3
[-300,   np.pi/2.0, 0, 800.5,  0], #T_3_4
[ 0,    -np. pi/2.0, 0, 0,      0], #T_4_5
[ 0,     np.pi/2.0, 0, 457.3,  0]] #T_5_6
d = np.asarray(HR_MDH_parameter_nominal)[:,3]/1000
a = np.asarray(HR_MDH_parameter_nominal)[:,0]/1000
alpha = np.asarray(HR_MDH_parameter_nominal)[:,1]
theta = np.asarray(HR_MDH_parameter_nominal)[:,4]

#a,alpha,beta,d,theta
'''
HR_DH_parameter_nominal = [[245.5, -np.pi/2.0,  0,  0,      0], #T_1_2
[1300,   0,          0,  0,      0], #T_2_3
[-300,   np.pi/2.0,  0,  0,      0], #T_3_4
[ 0,    -np.pi/2.0,  0,  800.5,  0], #T_4_5
[ 0,     np.pi/2.0,  0,  0,      0],
[ 0,     0        ,  0,  457.3,  0]] #T_5_6

d = np.asarray(HR_DH_parameter_nominal)[:,3]/1000
a = np.asarray(HR_DH_parameter_nominal)[:,0]/1000
alpha = np.asarray(HR_DH_parameter_nominal)[:,1]
theta = np.asarray(HR_DH_parameter_nominal)[:,4]
'''
print("d ",d)
print("a ",a)
print("alpha: ",alpha)
print("theta: ",theta)

mdh_params = np.array([[d[0],a[0],alpha[0],theta[0]],
[d[1],a[1],alpha[1],theta[1]],
[d[2],a[2],alpha[2],theta[2]],
[d[3],a[3],alpha[3],theta[3]],
[d[4],a[4],alpha[4],theta[4]],
[d[5],a[5],alpha[5],theta[5]]])

'''
dh_params = np.array([[d[0],a[0],alpha[0],theta[0]],
[d[1],a[1],alpha[1],theta[1]],
[d[2],a[2],alpha[2],theta[2]],
[d[3],a[3],alpha[3],theta[3]],
[d[4],a[4],alpha[4],theta[4]],
[d[5],a[5],alpha[5],theta[5]]])
'''

#IK = cdll.LoadLibrary("/home/joosm/Workplace/cpp_project/hydraulic_robot_kinematics/IK_test_visual_kinematics.so")    
IK = cdll.LoadLibrary("/home/joosm/Workplace/cpp_project/hydraulic_robot_kinematics/IK_test.so") 
#IK = cdll.LoadLibrary("/home/joosm/Workplace/cpp_project/hydraulic_robot_kinematics/IK_test_hyun.so") 
IK.InverseKinematics.restype = POINTER(c_double)
IK.InverseKinematics.argtypes = [POINTER(c_double),POINTER(c_double),POINTER(c_double),POINTER(c_double),POINTER(c_double),POINTER(c_double)]
A = a #0.001*HR_MDH_parameter_nominal[:,0]
#print(A)
#alpha = HR_MDH_parameter_nominal[:,1]
beta = np.asarray(HR_MDH_parameter_nominal)[:,2]
D = d# 0.001*HR_MDH_parameter_nominal[:,3]
#print(D)
#theta = HR_MDH_parameter_nominal[:,4]
#print(len(A))
A = (c_double * len(A))(*A)
alpha = (c_double * len(alpha))(*alpha)
beta = (c_double * len(beta))(*beta)
D = (c_double * len(D))(*D)
theta = (c_double * len(theta))(*theta)

'''
def HR_analytical_IK(dh_params,TCP_target):
    #TCP_target_position = T_0_6[0:3,3].flatten() # unit: mm
    #TCP_target_orientation = np.rad2deg(RKT.RotationMatrixToRPYeulerAngles(T_0_6[0:3,0:3]))
    #print(TCP_target_position, " ", np.rad2deg(TCP_target_orientation))
    #TCP_target = np.array([TCP_target_position[0],TCP_target_position[1],TCP_target_position[2],TCP_target_orientation[0],TCP_target_orientation[1],TCP_target_orientation[2] ])
    #TCP_target = np.hstack((TCP_target_position,TCP_target_orientation))
    #print(TCP_target)
    #print(TCP_target.shape)

    #TCP_target = (c_double * len(TCP_target))(*TCP_target)    

    q = IK.InverseKinematics(A,alpha,beta,D,theta,(c_double * len(TCP_target))(*TCP_target))[0:6]
    return True, q
'''

def main():
    np.set_printoptions(precision=3, suppress=True)
     #|  d  |  a  |  alpha  |  theta  |
    
    #dh_params = np.array([[0.163, 0., 0.5 * pi, 0.],
    #                      [0., 0.632, pi, 0.5 * pi],
    #                      [0., 0.6005, pi, 0.],
    #                      [0.2013, 0., -0.5 * pi, -0.5 * pi],
    #                      [0.1025, 0., 0.5 * pi, 0.],
    #                      [0.094, 0., 0., 0.]])
    def HR_analytical_IK(mdh_params,f):
        #TCP_target_position = T_0_6[0:3,3].flatten() # unit: mm
        #TCP_target_orientation = np.rad2deg(RKT.RotationMatrixToRPYeulerAngles(T_0_6[0:3,0:3]))
        #print(TCP_target_position, " ", np.rad2deg(TCP_target_orientation))
        #TCP_target = np.array([TCP_target_position[0],TCP_target_position[1],TCP_target_position[2],TCP_target_orientation[0],TCP_target_orientation[1],TCP_target_orientation[2] ])
        #TCP_target = np.hstack((TCP_target_position,TCP_target_orientation))
        #print(TCP_target)
        #print(TCP_target.shape)

        #TCP_target = (c_double * 6)(*TCP_target)    

        #q = IK.InverseKinematics(A,alpha,beta,D,theta,TCP_target)[0:6]
        #print(f)
        TCP_target_position = f[0:3,3].flatten() # unit: mm
        TCP_target_orientation = np.rad2deg(RKT.RotationMatrixToRPYeulerAngles(f[0:3,0:3]))
        TCP_target = np.asarray([TCP_target_position[0]*1000, TCP_target_position[1]*1000, TCP_target_position[2]*1000, TCP_target_orientation[0], TCP_target_orientation[1], TCP_target_orientation[2]])
        q = IK.InverseKinematics(A,alpha,beta,D,theta,(c_double * 6)(*TCP_target))[0:6]
        #print(q)
        return True, q


    robot = RobotSerial(mdh_params,dh_type="modified",analytical_inv = HR_analytical_IK, plot_xlim=[-1.5, 1.5], plot_ylim=[-1.5, 1.5], plot_zlim=[-0.5, 1.0] )
    #robot = RobotSerial(mdh_params,dh_type="modified",plot_xlim=[-1.5, 1.5], plot_ylim=[-1.5, 1.5], plot_zlim=[-0.5, 1.0] )
    #robot = RobotSerial(dh_params,dh_type="normal" )
    # =====================================
    # trajectory
    # =====================================

    #  construct a frame using ZYX euler angle and translation vector
    #@staticmethod
    #def from_euler_3(euler_3, t_3_1):
    #    r_3_3 = Rotation.from_euler("ZYX", euler_3, degrees=False).as_matrix()
    #    return Frame.from_r_3_3(r_3_3, t_3_1)
    frames = []
    #for i in range(int(len(X)/2)):
    for i in range(100,len(X)):        
    #for i in range(10000): 
        T_Base_TCP_given = RKT.transform_from_pose(X_data_temp[i]) #T_0_TCP
        #print(T_Base_TCP_given)
        T_Base_TCP_given[0,3] = T_Base_TCP_given[0,3]/1000
        T_Base_TCP_given[1,3] = T_Base_TCP_given[1,3]/1000
        T_Base_TCP_given[2,3] = T_Base_TCP_given[2,3]/1000
        #print(T_Base_TCP_given)
        #print(T_Base_TCP_given)
        T_0_TCP = np.dot(np.linalg.inv(T_Base_0),T_Base_TCP_given)
        #print(T_temp)
        T_0_6 = np.dot(T_0_TCP,np.linalg.inv(T_6_TCP))
        TCP_target_position = T_0_6[0:3,3].flatten() # unit: mm
        TCP_target_orientation = RKT.RotationMatrixToRPYeulerAngles(T_0_6[0:3,0:3])     
        #print("TCP_target_position: ",TCP_target_position,  TCP_target_position[0]," ", TCP_target_position[1]," ", TCP_target_position[2])  
        #print("TCP_target_orientation: ",TCP_target_orientation)                
        #frames.append(Frame.from_euler_3(np.array([TCP_target_orientation[0],TCP_target_orientation[1],TCP_target_orientation[2]]), np.array([[TCP_target_position[0]], [TCP_target_position[1]], [TCP_target_position[2]]])))
        #print("TCP x,y,z: ", TCP_target_position[0], " ", TCP_target_position[1]," ",TCP_target_position[2])
        #if TCP_target_position[0]>=0:
        frames.append(Frame.from_euler_3(np.array([TCP_target_orientation[2],TCP_target_orientation[1],TCP_target_orientation[0]]), np.array([[TCP_target_position[0]], [TCP_target_position[1]], [TCP_target_position[2]]])))
        #HR_analytical_IK(mdh_params,Frame.from_euler_3(np.array([TCP_target_orientation[2],TCP_target_orientation[1],TCP_target_orientation[0]]), np.array([[TCP_target_position[0]], [TCP_target_position[1]], [TCP_target_position[2]]])))        
        #frames = [Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.28127], [0.], [1.13182]])),
        #      Frame.from_euler_3(np.array([0.25 * pi, 0., 0.75 * pi]), np.array([[0.48127], [0.], [1.13182]])),
        #      Frame.from_euler_3(np.array([0.5 * pi, 0., pi]), np.array([[0.48127], [0.], [0.63182]]))]
    print("frame set built")        
    trajectory = RobotTrajectory(robot, frames)
    print("trajectory built")        
    trajectory.show(num_segs=100, motion="lin", method="linear") #motion="p2p" or "lin", method="linear")
    print("show end") 

if __name__ == "__main__":
    main()
