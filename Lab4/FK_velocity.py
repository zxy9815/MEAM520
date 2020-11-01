#!/usr/bin/python2
import numpy as np


def FK_velocity (q, dq, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration
    :param dq: 1 x 6 vector corresponding to the robot's current joint velocities
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    v     - The resulting linear velocity in the world frame
    omega - The resulting angular velocity in the world frame
    """
    d1 = 76.2                      # Distance between joint 0 and joint 1
    a2 = 146.05                    # Distance between joint 1 and joint 2
    a3 = 187.325                   # Distance between joint 2 and joint 3
    d4 = 34                        # Distance between joint 3 and joint 4
    d5 = 68                        # Distance between joint 3 and joint 5
    lg = 0                         # Distance between joint 5 and end effector (gripper length)

    v = np.array([0, 0, 0])
    omega = np.array([0, 0, 0])

    #Case trcking world or base frame
    if(joint == 0 or joint == 1):
        return v, omega
    
    #Get Jacobians
    J = calcJacobians(q, joint)

    #Ignore q6 as it does not affector velocities
    Joint_v = np.zeros((5,1))
    Joint_v[:,0] = dq[0:5]

    velocities = np.matmul(J, Joint_v)

    v = velocities[0:3,0]
    omega = velocities[3:6,0]

    #units: v = mm/s, omega = rad/s
    return v, omega


def calcJacobians (q, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    J: 6 x 5 Linear and Angular Jacobian of the joint at the configuration q.
    """

    #Case trcking world or base frame
    if(joint == 0 or joint == 1):
        return np.zeros((6,5))
    
    #Get Cross Product Components Jvi = [Z_i-1 X (O_frame - O_i-1)]
    frame = joint - 1
    t_frame = getTransformMat(q,frame)
    O_frame = t_frame[0:3,-1]

    #Linear Jacobian
    Jv = np.zeros((3,5))

    for i in range(frame):
        t_prev = getTransformMat(q,i)
        O_prev = t_prev[0:3,-1]
        Z_prev = t_prev[0:3,-2]

        #Geometric Approach Jvi = [Z_i-1 X (O_frame - O_i-1)]
        Jv[:,i] = np.cross(Z_prev, (O_frame - O_prev))

    #Angular Jacobian
    Jw = np.zeros((3,5))

    #Jwi = [Z_i-1]
    for i in range(frame):
        t_prev = getTransformMat(q,i)
        Z_prev = t_prev[0:3,-2]
        Jw[:,i] = Z_prev

    #Output
    J = np.append(Jv, Jw, axis=0)
    return J

def getTransformMat(q, frame):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration
    :param frame: an integer in [0,5] corresponding to which frame you are tracking
    :return:
    t_frame: Transformation Matrix of frame to base frame
    """
    L1 = 76.2    # distance between joint 0 and joint 1
    L2 = 146.05  # distance between joint 1 and joint 2
    L3 = 187.325 # distance between joint 2 and joint 3
    L4 = 34      # distance between joint 3 and joint 4
    L5 = 34      # distance between joint 4 and center of gripper

    T0 = np.array([[1, 0, 0, 0],
                    [0, 1, 0, 0],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Frame 1 w.r.t Frame 0
    T1 = np.array([[np.cos(q[0]), -np.sin(q[0])*np.cos(-np.pi/2), np.sin(q[0])*np.sin(-np.pi/2), 0],
                    [np.sin(q[0]), np.cos(q[0])*np.cos(-np.pi/2), -np.cos(q[0])*np.sin(-np.pi/2), 0],
                    [0, np.sin(-np.pi/2), np.cos(-np.pi/2), L1],
                    [0, 0, 0, 1]])

    # Frame 2 w.r.t Frame 1
    T2 = np.array([[np.cos(q[1]-(np.pi/2)), -np.sin(q[1]-(np.pi/2)), 0, L2*np.cos(q[1]-(np.pi/2))],
                    [np.sin(q[1]-(np.pi/2)), np.cos(q[1]-(np.pi/2)), 0, L2*np.sin(q[1]-(np.pi/2))],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Frame 3 w.r.t Frame 2
    T3 = np.array([[np.cos(q[2]+(np.pi/2)), -np.sin(q[2]+(np.pi/2)), 0, L3*np.cos(q[2]+(np.pi/2))],
                    [np.sin(q[2]+(np.pi/2)), np.cos(q[2]+(np.pi/2)), 0, L3*np.sin(q[2]+(np.pi/2))],
                    [0, 0, 1, 0],
                    [0, 0, 0, 1]])

    # Frame 4 w.r.t Frame 3
    T4 = np.array([[np.cos(q[3]-(np.pi/2)), -np.sin(q[3]-(np.pi/2))*np.cos(-np.pi/2), np.sin(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                    [np.sin(q[3]-(np.pi/2)), np.cos(q[3]-(np.pi/2))*np.cos(-np.pi/2), -np.cos(q[3]-(np.pi/2))*np.sin(-np.pi/2), 0],
                    [0, np.sin(-np.pi/2), np.cos(-np.pi/2), 0],
                    [0, 0, 0, 1]])
    # Frame 5 w.r.t Frame 4
    T5 = np.array([[np.cos(q[4]), -np.sin(q[4]), 0, 0],
                    [np.sin(q[4]), np.cos(q[4]), 0, 0],
                    [0, 0, 1, L4 + L5],
                    [0, 0, 0, 1]])
    
    switcher = {
        0: T0,
        1: T1,
        2: T1.dot(T2),
        3: (T1.dot(T2)).dot(T3),
        4: ((T1.dot(T2)).dot(T3)).dot(T4),
        5: ((((T1.dot(T2)).dot(T3)).dot(T4)).dot(T5))
    }

    return switcher.get(frame)


if __name__=='__main__':

    q = [0,0,0,0,0,0]
    dq = [1.,1.,1.,1.,0,0]
    joint = 4

    v, omega = FK_velocity(q, dq, joint)
    print(v)
    print(omega)