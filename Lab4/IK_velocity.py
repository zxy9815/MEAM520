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

    dq = np.array([0,0,0,0,0,0])
    
    # handle joints 0 and 1:
    if ((joint == 1) or (joint == 0)):
        return dq

    
    
    
    xi = np.concatenate((v,omega),axis=0)
    #TODO: implement test
    J = calcJacobians(q, joint)
    
    # If any inputs in v or omega contain NaN, then the velocity can be anything
    
    # Test if it's possible to achieve the exact input velocities v and omega
    # If so, dq should be the joint velocities to achieve that.
    # if not, it should be the least squares solution
    if not(np.linalg.matrix_rank(J) == np.linalg.matrix_rank(np.append(J,np.array([xi]).T,axis=1))):
        print('Infeasible velocities.')
        dq = np.linalg.pinv(J) @ xi
    else:
        dq = np.linalg.inv(J) @ xi
    # Jplus = np.linalg.pinv(J.T @ J) @ J.T # dq = Jplus @ [v; omega]
    
    return dq

if __name__=='__main__':
    # do the tests in the zero position because it's easy.
    q = np.array([0,0,0,0,0,0])
    
    # for all joint movements, IK should give a unit output for that joint
    # joint 1 moves #############################
    # end effector (joint 6) velocity:
    v = np.array([0,255.325,0])
    omega = np.array([0,0,1])
    
    # joint 5 velocity
    # v = np.array([0,187.325+34,0])
    # omega = np.array([0,0,1])
    
    # joint 4 velocity
    # v = np.array([0,187.325,0])
    # omega = np.array([0,0,1])
    
    # joint 3 velocity 
    # v = np.array([0,0,0])
    # omega = np.array([0,0,1])
    
    # joint 2 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,1])
    
    # joint 1 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 2 moves #############################
    # end effector (joint 6) velocity:
    # v = np.array([146.05,0,-255.325])
    # omega = np.array([0,1,0])
    
    # joint 5 velocity
    # v = np.array([146.05,0,-187.325-34])
    # omega = np.array([0,1,0])
    
    # joint 4 velocity
    # v = np.array([146.05,0,-187.325])
    # omega = np.array([0,1,0])
    
    # joint 3 velocity 
    # v = np.array([146.05,0,0])
    # omega = np.array([0,1,0])
    
    # joint 2 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 1 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 3 moves #############################
    # end effector (joint 6) velocity:
    # v = np.array([0,0,-255.325])
    # omega = np.array([0,1,0])
    
    # joint 5 velocity
    # v = np.array([0,0,-187.325-34])
    # omega = np.array([0,1,0])
    
    # joint 4 velocity
    # v = np.array([0,0,-187.325])
    # omega = np.array([0,1,0])
    
    # joint 3 velocity 
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 2 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 1 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 4 moves #############################
    # end effector (joint 6) velocity:
    # v = np.array([0,0,-68])
    # omega = np.array([0,1,0])
    
    # joint 5 velocity
    # v = np.array([0,0,-34])
    # omega = np.array([0,1,0])
    
    # joint 4 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 3 velocity 
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 2 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 1 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 5 moves #############################
    # end effector (joint 6) velocity:
    # v = np.array([0,0,0])
    # omega = np.array([1,0,0])
    
    # joint 5 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 4 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 3 velocity 
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 2 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    # joint 1 velocity
    # v = np.array([0,0,0])
    # omega = np.array([0,0,0])
    
    
    joint = 6
    np.set_printoptions(suppress=True)
    print(IK_velocity(q,v,omega,joint))
