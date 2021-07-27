#!/usr/bin/env python
from numpy.core.defchararray import array, endswith
from numpy.core.fromnumeric import take, transpose
import rospy #import rospy 
import numpy as np #import numpy 
from numpy.lib.utils import info
from sensor_msgs.msg import Imu #subscribe imu data
from sensor_msgs.msg import MagneticField #subscribe mag data
import time
from geometry_msgs.msg  import PoseStamped

q0 = 1 # assume these q are correct
q1 = 0 #
q2 = 0 #
q3 = 0 #
q = np.array([q0 ,q1 ,q2, q3]) #quaternion input rostopic echo augular position (gyro)
q_k = q.transpose() # qT
q_k1 = q.transpose() # qT
pub_q = PoseStamped()

P_k = np.array([[0.125, 0.0003, 0.0003, 0.0003],[0.003 , 0.125 , 0.0003 , 0.0003],[0.0003, 0.0003 , 0.125 , 0.0003],[0.0003, 0.0003 , 0.0003 ,0.125]])
# record time
T = -1

#gryo subscriber stage1 
def gyro_callback(data,pub):
    global T, q_k , q_k1,P_k
    Ax = data.linear_acceleration.x
    Ay = data.linear_acceleration.y
    Az = data.linear_acceleration.z
    Wx = data.angular_velocity.x
    Wy = data.angular_velocity.y
    Wz = data.angular_velocity.z

    # solve time problem
    if (T == -1):
        T = data.header.stamp.to_sec()
        dt = 0
        pass
    else: #T != -1 , T will be updated T 
        t = data.header.stamp.to_sec() #updates time
        dt = t - T
        T=t
        print( 'T now is :' , dt)
        
    #prediction    
    omega_nb = np.array([[0 ,-Wx ,-Wy ,-Wz],[Wx, 0, Wz , -Wy],[Wy , -Wz, 0 ,Wx],[Wz, Wy, -Wx, 0]])
    A_k = np.eye(4) + (1/2) * omega_nb*dt
    q_k = np.matmul(A_k , q_k)
    q_k = (q_k[0]**2 + q_k[1]**2 + q_k[2]**2 +q_k[3]**2)/( q_k[0]+q_k[1] +q_k[2]+q_k[3])* q_k #Normalize
    Q_k = 0.000001*np.eye(4)
    P_k =np.matmul( np.matmul(A_k,P_k),np.transpose(A_k)) + Q_k
    print("P_k is : ",P_k)

    pub_q.pose.orientation.w = q_k[0]
    pub_q.pose.orientation.x = q_k[1]
    pub_q.pose.orientation.y = q_k[2]
    pub_q.pose.orientation.z = q_k[3]
    pub.publish(pub_q)



    #start of the coreection stage 1 (gyro callback)
    H_k = np.array([[-2*q2 , 2*q3 , -2*q0 , 2*q1] , [2*q1 , 2*q0 , 2*q3 , 2*q2] , [2*q0 , -2*q1 , -2*q2 , 2*q3]])
    R_k =2* np.eye(3)
    K_k = np.matmul(np.matmul(P_k , H_k.transpose()) , np.linalg.inv(np.matmul(np.matmul(H_k , P_k) , H_k.transpose()) + R_k)) # 4x3
    Z_k = np.array([[Ax],[Ay],[Az]]) # 3x1
    h1 = np.array([[2*q1*q3 - 2*q0*q2] , [2*q0*q1 + 2*q2*q3 ] , [q0^2 - q1^2 -q2^2 + q3^2]]) 
    q_1 = np.matmul(K_k , (Z_k - h1))
    q_1[2] = 0
    q_k1= q_k + q_1
    P_k =  np.matmul((np.eye(4) - np.matmul(K_k , H_k) ) ,P_k)

#mag subscriber stage 2
def mag_callback(data):

    global P_k , q_k1 , q_k
    Mx = data.magnetic_field.x
    My = data.magnetic_field.y
    Mz = data.magnetic_field.z

    #start of the correctoin stage 2
    H_k2 = np.array([[ 2*q3, 2*q2 , 2*q1, 2*q0] , [ 2*q0, -2*q1, -2*q2, -2*q3] , [ -2*q1, -2*q0 , 2*q3 , 2*q2 ]])
    R_k2 = np.eye(3)
    K_k2 = np.matmul(np.matmul(P_k , H_k2.transpose()) ,np.linalg.inv(np.matmul(np.matmul(H_k2 , P_k) , H_k2.transpose()) +  R_k2))
    Z_k2 = np.array([[Mx],[My],[Mz]])
    h2 = np.array([[2*q1*q2 + 2*q0*q3] , [q0^2 - q1^2 - q2^2 - q3^2] , [ 2*q2*q3 - 2*q0*q1]])
    q_2 = np.matmul(K_k2 , (Z_k2 - h2))
    q_2[1] = 0
    q_2[2] = 0
    q_k = q_k1 +q_2
    P_k = np.matmul((np.eye(4) - np.matmul(K_k2,H_k2)),P_k)

def listener():
    rospy.init_node('doubleKF', anonymous=True)
    pub = rospy.Publisher("QQ",PoseStamped,queue_size=1 )
    rospy.Subscriber("/mavros/imu/data_raw", Imu , gyro_callback,pub)
    rospy.Subscriber("/mavros/imu/mag", MagneticField, mag_callback)
    rospy.spin()
 
if __name__ == '__main__': #main function
    listener()
