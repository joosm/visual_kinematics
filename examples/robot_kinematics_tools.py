import numpy as np
import csv
import math

def transform_from_pose(pose): #orientation in degree
    translation = np.asarray([pose[0],pose[1],pose[2]]).reshape(3,1)
    R = RPYeulerAnglesToRotationMatrix([np.deg2rad(pose[3]),np.deg2rad(pose[4]),np.deg2rad(pose[5])])
    T_temp = np.hstack((R,translation))
    T4 = [0.0, 0.0, 0.0, 1.0]
    T = np.vstack((T_temp,T4))

    return T


def transform_from_translation_only(translation):
    #translation in millimeter
    x = translation[0]
    y = translation[1]
    z = translation[2]
    T1 = [1.0, 0.0, 0.0, x]
    T2 = [0.0, 1.0, 0.0, y]
    T3 = [0.0, 0.0, 1.0, z]
    T4 = [0.0, 0.0, 0.0, 1.0]
    T = np.array(np.vstack((T1,T2,T3,T4)))

    return T

def transformation_matrix_to_rotation(T,key): #from transformation matrix (or rotation matrix)to Euler angles or Quaternions
    c_11 = T[0][0]
    c_12 = T[0][1]
    c_13 = T[0][2]    

    c_21 = T[1][0]
    c_22 = T[1][1]
    c_23 = T[1][2] 

    c_31 = T[2][0]
    c_32 = T[2][1]
    c_33 = T[2][2]   

    if key == 'XYZ_euler_angles': #return in rad
        Rx = np.arctan2(-c_23,c_33)
        Ry = np.arctan2(c_13,np.sqrt(c_11**2 + c_12**2))
        Rz = np.arctan2(-c_12,c_11)
        rotation = [Rx,Ry,Rz]
    elif key == 'unit_quaternion':
        tr = c_11 + c_22 + c_33
        #print("tr: ",tr)
        if tr > 0:
            #print("case:1")         
            S = np.sqrt(tr + 1.0)*2.0
            q_0 = 0.25*S #0.5*np.sqrt(tr + 1) #qw   
            q_1 = (c_32 - c_23)/S #0.5*np.sign(c_32 - c_23)*np.sqrt(c_11 - c_22 - c_33 + 1) #qx
            q_2 = (c_13 - c_31)/S #0.5*np.sign(c_13 - c_31)*np.sqrt(c_22 - c_33 - c_11 + 1) #qy
            q_3 = (c_21 - c_12)/S # 0.5*np.sign(c_21 - c_12)*np.sqrt(c_33 - c_11 - c_22 + 1) #qz   

            #q_0_ = 0.5*np.sqrt(tr + 1)
            #q_1_ = 0.5*np.sign(c_32 - c_23)*np.sqrt(c_11 - c_22 - c_33 + 1)
            #q_2_ = 0.5*np.sign(c_13 - c_31)*np.sqrt(c_22 - c_33 - c_11 + 1)
            #q_3_ = 0.5*np.sign(c_21 - c_12)*np.sqrt(c_33 - c_11 - c_22 + 1)

            #print("q0: ",q_0, " ",q_0_)
            #print("q1: ",q_1, " ",q_1_)
            #print("q2: ",q_2, " ",q_2_)
            #print("q3: ",q_3, " ",q_3_)
        elif (c_11 > c_22) and (c_11 > c_33):
            #print("case:2") 
            S = np.sqrt(1.0 + c_11 - c_22 - c_33) * 2.0 #S = 4*qx
            q_0 = (c_32 - c_23)/S
            q_1 = 0.25*S
            q_2 = (c_12 + c_21)/S
            q_3 = (c_13 + c_31)/S          
        elif c_22 > c_33:
            #print("case:3") 
            S = np.sqrt(1.0 + c_22 - c_11 - c_33) * 2.0 #S = 4*qy
            q_0 = (c_13 - c_31)/S
            q_1 = (c_12 + c_21)/S
            q_2 = 0.25*S
            q_3 = (c_23 + c_32)/S           
        else:
            #print("case:4") 
            S = np.sqrt(1.0 + c_33 - c_11 - c_22) * 2.0 #S = 4*qz
            q_0 = (c_21 - c_12)/S
            q_1 = (c_13 + c_31)/S
            q_2 = (c_23 + c_32)/S
            q_3 = 0.25*S
    # https://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
    # https://math.stackexchange.com/questions/893984/conversion-of-rotation-matrix-to-quaternion 
    # https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation                         
        rotation = [q_0,q_1,q_2,q_3]
    else:
        print("unidentified key")
        rotation  = [0,0,0,0]           
        
    return rotation

#def RotationMatrixToQuaternions(R) : #rotation matrix to q0,q1,q2,q3

#    tr = m00 + m11 + m22

def transform_from_MDH_parameters_analytical(alpha,beta,a,d,theta):
    #alpha, theta, beta: in radian
	cos_theta = np.cos(theta)
	sin_theta = np.sin(theta)

	cos_alpha = np.cos(alpha)
	sin_alpha = np.sin(alpha)

	T1 = [cos_theta, -sin_theta, 0.0, a]
	T2 = [cos_alpha*sin_theta, cos_alpha*cos_theta, -sin_alpha, -d*sin_alpha]
	T3 = [sin_alpha*sin_theta, sin_alpha*cos_theta, cos_alpha, d*cos_alpha]
	T4 = [0.0,0.0,0.0,1.0]
	T = np.array(np.vstack((T1,T2,T3,T4)))

	return T

def transform_from_DH_parameters_analytical(alpha,beta,a,d,theta):

	cos_theta = np.cos(theta)
	sin_theta = np.sin(theta)

	cos_alpha = np.cos(alpha)
	sin_alpha = np.sin(alpha)

	T1 = [cos_theta, -sin_theta*cos_alpha, sin_theta*sin_alpha, a*cos_theta]
	T2 = [sin_theta, cos_theta*cos_alpha, -cos_theta*sin_alpha, a*sin_theta]
	T3 = [0        , sin_alpha,            cos_alpha, d]
	T4 = [0.0,0.0,0.0,1.0]
	T = np.array(np.vstack((T1,T2,T3,T4)))

	return T


def transform_from_MDH_parameters_numerical(alpha,beta,a,d,theta):

    Rx_alpha = rotation_about_axis('x',alpha)
    Dx_a = translation_along_axis('x',a)

    Ry_beta = rotation_about_axis('y',beta)
    Dz_d = translation_along_axis('z',d)
    Rz_theta = rotation_about_axis('z',theta)


    #T = np.dot(Rx_alpha,np.dot(Dx_a,np.dot(Dz_d, np.dot(Ry_beta,Rz_theta))))
    T = np.dot(Rx_alpha,np.dot(Dx_a,np.dot(Ry_beta,np.dot(Rz_theta,Dz_d))))
    return T


def rotation_about_axis(axis_name, rotation_angle):
	#rotation_angle in radian
	theta = rotation_angle
	cos_theta = np.cos(theta)
	sin_theta = np.sin(theta)
	if axis_name == 'x':
		T1 = [1.0,        0.0,        0.0, 0.0]
		T2 = [0.0,  cos_theta, -sin_theta, 0.0]
		T3 = [0.0,  sin_theta,  cos_theta, 0.0]
		T4 = [0.0,        0.0,        0.0, 1.0]
		T = np.vstack((T1,T2,T3,T4))

	elif axis_name == 'y':
		T1 = [cos_theta ,    0.0, sin_theta, 0.0]
		T2 = [0.0       ,    1.0,       0.0, 0.0]
		T3 = [-sin_theta,    0.0, cos_theta, 0.0]
		T4 = [0.0       ,    0.0,       0.0, 1.0]
		T = np.vstack((T1,T2,T3,T4))

	elif axis_name == 'z':
		T1 = [cos_theta, -sin_theta, 0.0, 0.0]
		T2 = [sin_theta,  cos_theta, 0.0, 0.0]
		T3 = [0.0,              0.0, 1.0, 0.0]
		T4 = [0.0,              0.0, 0.0, 1.0]
		T = np.vstack((T1,T2,T3,T4))
	else:
		print('wrong axis name in rotation!')
		T = np.zeros((4,4))
	return T

def translation_along_axis(axis_name, translation):
	#translation in millimeter
	d = translation
	if axis_name == 'x':
		T1 = [1.0, 0.0, 0.0, d]
		T2 = [0.0, 1.0, 0.0, 0.0]
		T3 = [0.0, 0.0, 1.0, 0.0]
		T4 = [0.0, 0.0, 0.0, 1.0]
		T = np.vstack((T1,T2,T3,T4))

	elif axis_name == 'y':
		T1 = [1.0, 0.0, 0.0, 0.0]
		T2 = [0.0, 1.0, 0.0, d]
		T3 = [0.0, 0.0, 1.0, 0.0]
		T4 = [0.0, 0.0, 0.0, 1.0]
		T = np.vstack((T1,T2,T3,T4))

	elif axis_name == 'z':
		T1 = [1.0, 0.0, 0.0, 0.0]
		T2 = [0.0, 1.0, 0.0, 0.0]
		T3 = [0.0, 0.0, 1.0, d]
		T4 = [0.0, 0.0, 0.0, 1.0]
		T = np.vstack((T1,T2,T3,T4))

	else:
		print('wrong axis name in translation!')
		T = np.zeros((4,4))

	return T


def DH_T_analytical(DH, idx, joint_angle):
    #a,alpha,beta,d,theta
    a = DH[idx][0]
    alpha = DH[idx][1]
    beta = DH[idx][2]
    d = DH[idx][3]
    theta = DH[idx][4]+ joint_angle
    #print(a," ",alpha, " ", beta," ",d," ",theta)
    return transform_from_DH_parameters_analytical(alpha,beta,a,d,theta)

'''
def DH_T_numerical(DH, idx, joint_angle):
    #a,alpha,beta,d,theta
    a = DH[idx][0]*1000.0
    alpha = DH[idx][1]
    beta = DH[idx][2] 
    d = DH[idx][3]*1000.0
    theta = DH[idx][4] + joint_angle
    #print("a, alpha,beta,d,theta, joint angle, theta + joint angle: ",a," ",alpha, " ", beta," ", d," ",DH[idx][4]," ",joint_angle," ",theta)
    return transform_from_DH_parameters_numerical(alpha,beta,a,d,theta)
'''

def MDH_T_analytical(DH, idx, joint_angle):
    #a,alpha,beta,d,theta
    a = DH[idx][0]
    alpha = DH[idx][1]
    beta = DH[idx][2]
    d = DH[idx][3]
    theta = DH[idx][4]+ joint_angle
    #print(a," ",alpha, " ", beta," ",d," ",theta)
    return transform_from_MDH_parameters_analytical(alpha,beta,a,d,theta)

def MDH_T_numerical(DH, idx, joint_angle):
    #a,alpha,beta,d,theta
    a = DH[idx][0]
    alpha = DH[idx][1]
    beta = DH[idx][2]
    d = DH[idx][3]
    theta = DH[idx][4]+ joint_angle
    #print("a, alpha,beta,d,theta, joint angle, theta + joint angle: ",a," ",alpha, " ", beta," ", d," ",DH[idx][4]," ",joint_angle," ",theta)
    return transform_from_MDH_parameters_numerical(alpha,beta,a,d,theta)    


# Checks if a matrix is a valid rotation matrix.
def isRotationMatrix(R) :
    Rt = np.transpose(R)
    shouldBeIdentity = np.dot(Rt, R)
    I = np.identity(3, dtype = R.dtype)
    n = np.linalg.norm(I - shouldBeIdentity)
    return n < 1e-6


def RPYeulerAnglesToRotationMatrix(theta=[0,0,0]) : #ZYX euler angle
    roll  = theta[0]
    pitch = theta[1]
    yaw   = theta[2]
    R_x = np.array([[1,                0,               0               ],
                    [0,                math.cos(roll), -math.sin(roll) ],
                    [0,                math.sin(roll),  math.cos(roll) ]])
    R_y = np.array([[math.cos(pitch),  0,               math.sin(pitch)],
                    [0,                1,               0              ],
                    [-math.sin(pitch), 0,               math.cos(pitch)]])
    R_z = np.array([[math.cos(yaw),   -math.sin(yaw),   0],
                    [math.sin(yaw),    math.cos(yaw),   0],
                    [0,                0,               1]])
    
    R = np.dot(R_z, np.dot( R_y, R_x ))
    
    assert(isRotationMatrix(R))

    return R

def RotationMatrixToRPYeulerAngles(R) : #rotation matrix to roll, pitch, yaw
    assert(isRotationMatrix(R))
    sy = math.sqrt(R[0,0] * R[0,0] +  R[1,0] * R[1,0])
    singular = sy < 1e-6
    if  not singular :
        x = math.atan2(R[2,1] , R[2,2]) #psi, rotation about x
        y = math.atan2(-R[2,0], sy) # theta, rotation about y
        z = math.atan2(R[1,0], R[0,0]) #phi, rotation about z
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0

    return np.array([x, y, z]) #psi,theta, phi, R = R_z(phi)*R_y(theta)*R_x(psi), return in radian


def RPYeulerAnglesToQuaternion(theta=[0,0,0]) : #ZYX euler angles to Quaternion
    roll  = theta[0] #phi
    pitch = theta[1] #theta 
    yaw   = theta[2] #psi
    cos_phi_2   = math.cos(roll * 0.5)
    cos_theta_2 = math.cos(pitch * 0.5)
    cos_psi_2   = math.cos(yaw * 0.5)
    sin_phi_2   = math.sin(roll * 0.5)
    sin_theta_2 = math.sin(pitch * 0.5)
    sin_psi_2   = math.sin(yaw * 0.5)
    
    q0 = cos_phi_2 * cos_theta_2 * cos_psi_2 + sin_phi_2 * sin_theta_2 * sin_psi_2 
    q1 = sin_phi_2 * cos_theta_2 * cos_psi_2 - cos_phi_2 * sin_theta_2 * sin_psi_2
    q2 = cos_phi_2 * sin_theta_2 * cos_psi_2 + sin_phi_2 * cos_theta_2 * sin_psi_2
    q3 = cos_phi_2 * cos_theta_2 * sin_psi_2 - sin_phi_2 * sin_theta_2 * cos_psi_2

    return np.array([q0,q1,q2,q3]) 

def QuaternionToRPYeulerAngles(q=[0,0,0,0]) : #Quaternion to ZYX euler angles (radian)
    q_norm = np.linalg.norm(q)
    if abs(1-q_norm) > 0.0001:
        print("norm of quaternion is not 1")
    
    q0 = q[0]
    q1 = q[1] 
    q2 = q[2]
    q3 = q[3]
    
    phi = math.atan2(2.0*(q0*q1 + q2*q3), 1.0 - 2.0*(q1*q1 + q2*q2)) 
    theta = math.asin(2.0*(q0*q2 - q3*q1))
    psi = math.atan2(2.0*(q0*q3 + q1*q2), 1.0 - 2.0*(q2*q2 + q3*q3))

    return np.array([phi,theta,psi]) 

def QuaternionToRotationMatrix(Q=[0,0,0,0]) : #Quaternion to Rotation matrix
    """
    Covert a quaternion into a full three-dimensional rotation matrix.
 
    Input
    :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
 
    Output
    :return: A 3x3 element matrix representing the full 3D rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
    """
    q_norm = np.linalg.norm(Q)
    if abs(1-q_norm) > 0.0001:
        print("norm of quaternion is not 1."," The norm of q is ",q_norm)

    # Extract the values from Q
    q0 = Q[0]
    q1 = Q[1]
    q2 = Q[2]
    q3 = Q[3]
     
    # First row of the rotation matrix
    r00 = 2 * (q0 * q0 + q1 * q1) - 1
    r01 = 2 * (q1 * q2 - q0 * q3)
    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
    r10 = 2 * (q1 * q2 + q0 * q3)
    r11 = 2 * (q0 * q0 + q2 * q2) - 1
    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
    r20 = 2 * (q1 * q3 - q0 * q2)
    r21 = 2 * (q2 * q3 + q0 * q1)
    r22 = 2 * (q0 * q0 + q3 * q3) - 1
     
    # 3x3 rotation matrix
    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])

    assert(isRotationMatrix(R))

    return R

#https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
def RotationMatrixFromAxisAngle(u,theta): #u = unit direction vector, theta = rotation angle in radian

    u_norm = np.linalg.norm(u,ord=2)
    if np.abs(u_norm - 1.0) > 1e-7:
        print("rotation axis vector is not a unit vector!")
        print("norm of the rotation axis vector is ",u_norm)
        u = np.multiply(1/u_norm,u)
    c_theta = np.cos(theta)
    s_theta = np.sin(theta)
    ux = u[0]
    uy = u[1]
    uz = u[2]
    alpha = 1-c_theta

    r00 = c_theta + ux*ux*alpha
    r01 = ux*uy*alpha - uz*s_theta
    r02 = ux*uz*alpha + uy*s_theta

    r10 = uy*ux*alpha + uz*s_theta
    r11 = c_theta + uy*uy*alpha
    r12 = uy*uz*alpha - ux*s_theta

    r20 = uz*ux*alpha - uy*s_theta
    r21 = uz*uy*alpha + ux*s_theta
    r22 = c_theta + uz*uz*alpha

    # 3x3 rotation matrix
    R = np.array([[r00, r01, r02],
                  [r10, r11, r12],
                  [r20, r21, r22]])

    assert(isRotationMatrix(R))

    return R
                
