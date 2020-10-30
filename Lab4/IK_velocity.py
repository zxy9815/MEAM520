import numpy as np


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

    dq = np.array([0, 0, 0, 0, 0, 0])
    return dq
