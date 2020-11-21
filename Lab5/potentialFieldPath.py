import numpy as np
from numpy import random
from potentialFieldStep import potentialFieldStep
from loadmap import loadmap
from calculateFK import calculateFK
from distPointToBox import distPointToBox
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

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
    v = 0.01 # the initial random step size
    
    localMinTol = 1.1e-2#9e-3 # minimum step for a local min detection
    
    fk = calculateFK()
    
    obstacles = map.obstacles
    
    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)
    
    while (not(isDone)):
        print(qCurr)
        qCurr,isDone = potentialFieldStep(qCurr,map,qGoal)
        path = np.vstack((path,qCurr))
        
        # local minima detection, code inspired by lecture 18 slide 27
        s = path.shape[0] # size of the history
        if (s > 4): # only look back at most 4 steps
            step = np.empty((4))
            for i in range(4): # find the norms of 
                step[i] = np.linalg.norm(path[-1]-path[-1-i])
            
            # detect the local min
            if ((step<localMinTol).all()):
                print('Local minimum detected. Attempting local walk.')
                
                localmin = path[-1] # store the local min
                
                # hacked together do-while loop
                qNext = np.ones_like(path[-1])*v
                for i in range(qNext.shape[0]-2):
                    direction = random.randint(3)-1
                    qNext[i] = qNext[i] * direction + localmin[i] # step in a random direction
                qNext[-1] = localmin[-1]
                qNext[-2] = localmin[-2]
                    
                # check if the new position collides
                jointpos_next, t0i_next = fk.forward(qNext)
                dists = np.zeros((6*len(obstacles)))
                for k in range(len(obstacles)):
                    dists[k*6:(k+1)*6], unitVec = distPointToBox(jointpos_next,obstacles[k])
                
                # do this until this new position doesn't collide with anything
                while (np.logical_not(dists).any()):
                    print('collision detected while stepping')
                    qNext = np.ones_like(path[-1])*v
                    print('qNext')
                    print(qNext)
                    for i in range(qNext.shape[0]-2): # ignore final two joints
                        direction = random.randint(3)-1
                        qNext[i] = np.clip(qNext[i] * direction + localmin[i],\
                                           lowerLim[0,i],upperLim[0,i]) # step in a random direction
                    qNext[-1] = localmin[-1]
                    qNext[-2] = localmin[-2]
                    # check if the new position collides
                    jointpos_next, t0i_next = fk.forward(qNext)
                    dists = np.zeros((6*len(obstacles)))
                    for k in range(len(obstacles)):
                        dists[k*6:(k+1)*6], unitVec = distPointToBox(jointpos_next,obstacles[k])
                
                qCurr = qNext
                
                path = np.vstack((path,qNext))
                if (v < 0.3): # cap v
                    v = v*1.02 # increase step size for next time
                
                
    return path

if __name__=='__main__':

    # Map 1 test from lab 3
    # start = np.array([0,0,0,0,0,0])
    # goal = np.array([0,0,1.1,0,0,0])
    # map = loadmap("maps/map1.txt")    

    # bare bones go down test
    # start = np.array([0,0, -1.3, 0, 0, 0])
    # goal = np.array([0,0, 1.3,0, 0, 0])
    # map = loadmap("maps/map2.txt")
    
    # go down and induce local min
    # start = np.array([0,0, -1.3, 0, 0, 0])
    # goal = np.array([0,0, 1.3,0, 0, 0])
    # map = loadmap("maps/twoPosts.txt")
        
    # Map 3 test from lab 3
    start = np.array([0,0,0,0,0,0])
    goal = np.array([1.4,-0.2,0,0,0,0])
    map = loadmap("maps/map3.txt")
    
    # go down and induce local min with a flat trap
    # start = np.array([0,0, -1.3, 0, 0, 0])
    # goal = np.array([0,0, 1.3,0, 0, 0])
    # map = loadmap("maps/uTrap.txt")
    
    # go down and induce local min with a u trap
    # start = np.array([0,0, 1.3, 0, 0, 0])
    # goal = np.array([0,0, -1.3,0, 0, 0])
    # map = loadmap("maps/uTrap.txt")
    
    
    path = potentialFieldPath(map,start,goal)
    
    #Add Plots
    plt.close('all')
    fk = calculateFK()
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    #ax.axes.set_xlim3d(left=-400, right=400)
    ax.axes.set_ylim3d(bottom=-400, top=400)
    #ax.axes.set_zlim3d(bottom=-400, top=400)
    
    x = []
    y = []
    z = []
    for q in path:
        jointPositions, t0e = fk.forward(q)
        x.append(jointPositions[-1,0])
        y.append(jointPositions[-1,1])
        z.append(jointPositions[-1,2])
    ax.plot(x,y,z)
    ax.set_xlabel("x axis")
    ax.set_ylabel("y axis")
    ax.set_zlabel("z axis")
    plt.show()
    
    # print(path)