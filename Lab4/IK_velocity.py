import numpy as np
from FK_velocity import *


def IK_velocity (q, v, omega, joint):
    """
    :param q: 1 x 6 vector corresponding to the robot's current configuration.
    :param v: The desired linear velocity in the world frame. If any element is Nan, then that velocity can be anything
    :param omega: The desired angular velocity in the world frame. If any element is Nan, then that velocity can be
                  anything
    :param joint: an integer in [0,6] corresponding to which joint you are tracking
    :return:
    dq - 1 x 6 vector corresponding to the joint velocities. If v and omega
         are infeasible, then dq should minimize the least squares error.

    """
    d1 = 76.2                      # Distance between joint 0 and joint 1
    a2 = 146.05                    # Distance between joint 1 and joint 2
    a3 = 187.325                   # Distance between joint 2 and joint 3
    d4 = 34                        # Distance between joint 3 and joint 4
    d5 = 68                        # Distance between joint 3 and joint 5
    lg = 0                         # Distance between joint 5 and end effector (gripper length)

    # handle joints 0 and 1:
    if ((joint == 1) or (joint == 0)):
        dq = np.array([0,0,0,0,0,0])
        return dq

    # Test if it's possible to achieve the exact input velocities v and omega
    # If so, dq should be the joint velocities to achieve that.
    
    
    xi = np.concatenate((v,omega),axis=0)
    #TODO: implement test
    J = calcJacobians(q, joint)
    
    
    if not(np.linalg.matrix_rank(J) == np.linalg.matrix_rank(np.append(J,np.array([xi]).T,axis=1))):
        print('Infeasible velocities.')
    
    Jplus = np.linalg.pinv(J.T @ J) @ J.T # dq = Jplus @ [v; omega]
    dq = np.linalg.pinv(J) @ xi
    
    # If not possible to get exact input velocities v and omega:
    # dq should be joint velocities that minimize least squared error btwn result and desired
    
    # If any inputs in v or omega contain NaN, then the velocity can be anything
    
    return dq

if __name__=='__main__':
    q = np.array([0,0,0,0,0,0])
    velocity = np.array([0,255.325,0])
    omega = np.array([0,0,1])
    joint = 6
    np.set_printoptions(suppress=True)
    print(IK_velocity(q,velocity,omega,joint))
