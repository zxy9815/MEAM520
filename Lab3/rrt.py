
import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision
from loadmap import loadmap
import matplotlib.pyplot as plt
from mpl_toolkits import mplot3d

def rrt(map, start, goal):
    """
    Implement RRT algorithm in this file.
    :param map:         the map struct
    :param start:       start pose of the robot (1x6).
    :param goal:        goal pose of the robot (1x6).
    :return:            returns an mx6 matrix, where each row consists of the configuration of the Lynx at a point on
                        the path. The first row is start and the last row is goal. If no path is found, PATH is a 0x6
                        matrix..
    """

    #joint limits, gripper constant
    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, -15]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 30]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    #Max iterations
    n_iter = 1000
    isPath = False
    fk = calculateFK()

    #################################
    #Initialize 2 graphs in C-space
    obstacles = map.obstacles

    #Initialize Nodes
    V_start = start.reshape((1, 6))
    V_goal = goal.reshape((1, 6))

    #Initialize Edges
    E_start = np.array([-1,-1]).reshape((1,2))
    E_goal = np.array([-1,-1]).reshape((1,2))

    ##################################
    #Build up RRT
    for iter_ in range(n_iter):
        #Pick a random point btw limits
        q_rand = np.random.uniform(low=lowerLim, high=upperLim, size=(1,6))
        q_rand = np.reshape(q_rand, -1)
        q_rand[5] = 0.0

        #Check if random q is valid
        num_rand = 1
        while not isValidConfig(q_rand, obstacles):
            q_rand = np.random.uniform(low=lowerLim, high=upperLim, size=(1,6))
            q_rand = np.reshape(q_rand, -1)
            q_rand[5] = 0.0
            num_rand += 1
            if(num_rand > n_iter):
                print("cannot find a valid config")
                return

        ###########################################
        #Find closest node from start tree
        dist_s = []
        for node in V_start:
            #measure by Linf distance in C-space
            d = np.max(np.abs(q_rand - node))
            dist_s.append(d)

        ind_a = np.argmin(dist_s)
        q_a = V_start[ind_a,:]

        #Check if (q_rand, q_a) has collision
        isCollide_a = False

        #set 20 waypoints in C-space btw the 2 configs
        waypoints = np.zeros((10,len(q_a)))
        for i in range(len(q_a)):
          waypoints[:,i] = np.linspace(q_a[i], q_rand[i], 10, endpoint=True)

        for config in range(len(waypoints)):
            isCollide = not(isValidConfig(waypoints[config],obstacles))
            if(isCollide):
                print("qa qrand collide with obstacle")
                isCollide_a = True
                break


        #if not collide, add new edge and node
        if not isCollide_a:
            new_edge = np.array([ind_a, len(V_start)]).reshape((1,2))
            E_start = np.append(E_start, new_edge, axis=0)
            V_start = np.append(V_start, q_rand.reshape((1,6)), axis=0)

        ##################################################
        #Find closest node from end tree
        dist_s = []
        for node in V_goal:
            #measure by L2 distance in C-space
            d = np.max(np.abs(q_rand - node))
            dist_s.append(d)

        ind_b = np.argmin(dist_s)
        q_b = V_goal[ind_b,:]

        #Check if (q_rand, q_b) has collision
        isCollide_b = False

        #set 10 waypoints in C-space btw the 2 configs
        waypoints_b = np.zeros((10,len(q_b)))
        for i in range(len(q_b)):
          waypoints_b[:,i] = np.linspace(q_rand[i], q_b[i], 10, endpoint=True)
        

        for config in range(len(waypoints_b)):
            isCollide = not(isValidConfig(waypoints_b[config],obstacles))
            if(isCollide):
                print("qb qrand collide with obstacle")
                isCollide_b = True
                break

        #if not collide, add new edge and node
        if not isCollide_b:
            new_edge = np.array([ind_b, len(V_goal)]).reshape((1,2))
            E_goal = np.append(E_goal, new_edge, axis=0)
            V_goal = np.append(V_goal, q_rand.reshape((1,6)), axis=0)

        #if connect to both trees, then a path is found!
        if not isCollide_a and not isCollide_b:
            print("Path found!")
            isPath = True
            break

    ###################################
    #Backtrack
    if isPath:
        #init path as mid node
        path = V_goal[-1,:].reshape((1,6))

        #Add path from start to mid
        #get parent of mid
        ind1 = E_start[-1, 0]
        while ind1 != 0:
            node = V_start[ind1, :].reshape((1,6))
            path = np.insert(path,0,node,axis=0)
            for row in range(len(E_start)):
                if(E_start[row,1] == ind1):
                    ind1 = E_start[row, 0]
                    break

        #insert start
        path = np.insert(path,0,start.reshape((1, 6)),axis=0)

        #Add path from mid to goal
        #get parent of mid
        ind2 = E_goal[-1, 0]
        while ind2 != 0:
            node = V_goal[ind2, :].reshape((1,6))
            path = np.append(path,node,axis=0)
            for row in range(len(E_goal)):
                if(E_goal[row,1] == ind2):
                    ind2 = E_goal[row, 0]
                    break

        #append goal
        path = np.append(path,goal.reshape((1, 6)),axis=0)
        print("RRT Path")
        print(path)
        return path
    else:
        raise Exception("No Path Found. ")


def makeRectangle(points,q,joint,delta,w,rot=0):
    """
    Find the vertices of a rectangle centered around point p given
    axes u and v perpendicular to its normal and edges
    points:     6x3 array, joint positions
    q:          1x6 array, joint values
    joint:      integer, which link to find the rectangle for. NOT ZERO-INDEXED
    delta:      float, the depth of the link
    w:          float, the width of the link
    rot:        extra rotation for link 5 and end effector

    return:     8x3 array, each row contains the coordinates of a vertex for the box


    """

    l = points[joint,:] - points[joint-1,:]

    # rotate 90 degrees about z_1 to get u
    x = np.sin(q[0])*np.sin(-np.pi/2)
    y = -np.cos(q[0])*np.sin(-np.pi/2)
    z = np.cos(-np.pi/2)
    c = np.cos(np.pi/2)
    s = np.sin(np.pi/2)
    C = 1 - c

    R = np.array([[x*x*C + c, x*y*C - z*s, x*z*C + y*s],
                  [y*x*C+z*s, y*y*C + c, y*z*C - x*s],
                  [z*x*C-y*s, z*y*C+x*s, z*z*C + c]])
    u = np.matmul(R, l.T)
    u = u / np.linalg.norm(u) * delta

    v = np.cross(u,l)
    v = v / np.linalg.norm(v) * w


    if (rot): # if it's a joint that rotates with theta5:
        normal = np.linalg.norm(l)
        x = l[0] / normal
        y = l[1] / normal
        z = l[2] / normal
        c = np.cos(rot)
        s = np.sin(rot)
        C = 1 - c

        R = np.array([[x*x*C + c, x*y*C - z*s, x*z*C + y*s],
                      [y*x*C+z*s, y*y*C + c, y*z*C - x*s],
                      [z*x*C-y*s, z*y*C+x*s, z*z*C + c]])
        u = np.matmul(R, u.T)
        v = np.matmul(R, v.T)

    # Credit to stackexchange for this algorithm in 2D space:
    # https://math.stackexchange.com/questions/2518607/how-to-find-vertices-of-a-rectangle-when-center-coordinates-and-angle-of-tilt-is
    vert = np.zeros((8,3))
    p1  = points[joint,:]
    vert[0,:] = p1 - u + v
    vert[1,:] = p1 + u + v
    vert[2,:] = p1 + u - v
    vert[3,:] = p1 - u - v

    p2 = points[joint-1,:]
    vert[4,:] = p2 - u + v
    vert[5,:] = p2 + u + v
    vert[6,:] = p2 + u - v
    vert[7,:] = p2 - u - v

    return vert



def isValidConfig(q, obstacles):
    """
    Check if a configuration is in free C-space
    :q:                 Configuration of the robot (1x6)
    :obstacles          location of all boxes (Nx6) [xmin, ymin, zmin, xmax, ymax, zmax]
    :return:            True if valid configuration, else false
    """
    fk = calculateFK()
    q = np.reshape(q,-1)
    points, T0e = fk.forward(q)

    buffer = 5 # buffer to add to all links

    # Define the space of link 1: ####################################
    delta1  = 50 + buffer # half the depth of link 1, mm
    w1      = 50 + buffer # half the width of link 1, mm
    link1 = makeRectangle(points,q,1,delta1,w1) # rectangle at joint 1

    # Define the space of link 2: ####################################
    delta2  = 10.3 + buffer
    w2      = 33.3 + buffer
    link2 = makeRectangle(points,q,2,delta2,w2)

    # Define the space of link 3: ####################################
    delta3  = 15.5 + buffer
    w3      = 24.5 + buffer
    # define special case for the bit of the lynx sticking out its back:
    l = points[3,:] - points[2,:]
    l = l / np.linalg.norm(l) # unit vector pointing in direction of link 3
    l = l * (-28.5) # rough distance to the back end
    link3Points = np.array([points[2,:]+l,points[3]])
    link3 = makeRectangle(link3Points,q,1,delta3,w3)

    # Define the space of link 4: ####################################
    delta4  = 15.5 + buffer
    w4      = 30.1 + buffer
    link4 = makeRectangle(points,q,4,delta4,w4)

    # Define the space of link 5: ####################################
    delta5  = 27.3 + buffer
    w5      = 10.5 + buffer
    link5 = makeRectangle(points,q,5,delta5,w5,rot=q[4])

    # Define the space of end effector: ##############################
    if (q[5] >=0):
        deltaEE = 5.2 + q[5] + buffer
    else:
        deltaEE = 5.2 + buffer
    wEE = 10.5 + buffer
    # have to make a new point for the tip of the end effector:
    l = points[-1,:] - points[4,:]
    l = l / np.linalg.norm(l) # the unit vector pointing in the direction of the ee
    l = l * 12.5 + buffer# the length of the end effector tips
    eePoints = np.array([points[-1,:],points[-1,:]+l])
    linkEE = makeRectangle(eePoints,q,1,deltaEE,wEE,rot=q[4]) # use end effector points


    # the index of the points in the rectangles that begin and end a line
    # the first four pairs are the lines along the length of the rectangle
    # the last eight pairs are the rectangles at the end of the prism
    startPtIdx = np.array([0,1,2,3,0,1,2,3,4,5,6,7],dtype=np.int8)
    endPtIdx   = np.array([4,5,6,7,1,2,3,0,5,6,7,4],dtype=np.int8)

    # Test to see if any links collide with an obstacle
    for link in np.array([link1,link2,link3,link4,link5,linkEE]):
        for obs in range(len(obstacles)):
            isCollide = detectCollision(link[startPtIdx],link[endPtIdx],obstacles[obs])
            if(any(isCollide)):
                return False
    for link in np.array([link3,link4,link5,linkEE]):
        isCollide = detectCollision(link[startPtIdx],link[endPtIdx],np.array([-50, -50, 0, 50, 50, 76.2]))
        if(any(isCollide)):
            print("Lynx collides with base frame")
            return False

    return True

