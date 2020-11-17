import numpy as np
from potentialFieldStep import potentialFieldStep
from loadmap import loadmap

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
    isDone = False
    qCurr = qStart.copy()
    path = np.array([qStart])
    
    while (not(isDone)):
        print(qCurr)
        qCurr,isDone = potentialFieldStep(qCurr,map,qGoal)
        path = np.vstack((path,qCurr))
    
    return path

if __name__=='__main__':

    start = np.array([0,  0, 0, 0, 0, 0])
    goal = np.array([0, 0, 1.1, 0, 0, 0])
    map = loadmap("maps/map2.txt")
    path = potentialFieldPath(map,start,goal)
    
    print(path)
        