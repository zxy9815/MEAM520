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

    return v, omega
