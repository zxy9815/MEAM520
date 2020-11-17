import numpy as np
from potentialFieldStep import potentialFieldStep

def potentialFieldPath(map, qStart, qGoal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param qStart:       start pose of the robot (1x6).
    :param qGoal:        goal pose of the robot (1x6).
    :return:            returns an Nx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix.
    """
