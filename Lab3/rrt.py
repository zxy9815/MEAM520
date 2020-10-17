import numpy as np
import random
from calculateFK import calculateFK
from detectCollision import detectCollision

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
    lowerLim = np.array([-1.4, -1.2, -1.8, -1.9, -2.0, 0]).reshape((1, 6))    # Lower joint limits in radians (grip in mm (negative closes more firmly))
    upperLim = np.array([1.4, 1.4, 1.7, 1.7, 1.5, 0]).reshape((1, 6))          # Upper joint limits in radians (grip in mm)

    #Max iterations
    n_iter = 1000
    isPath = False

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

        #Check if random q is valid
        num_rand = 1
        while not isValidConfig(q_rand, obstacles):
            q_rand = np.random.uniform(low=lowerLim, high=upperLim, size=(1,6))
            num_rand += 1
            if(num_rand > n_iter):
                print("cannot find a valid config")
                return
        
        ###########################################
        #Find closest node from start tree
        dist_s = []
        for node in V_start:
            #measure by L2 distance in C-space
            d = np.sum(np.power((q_rand - node),2))
            dist_s.append(d)
        
        ind_a = np.argmin(dist_s)
        q_a = V_start[ind_a,:]

        #Check if (q_rand, q_a) has collision
        isCollide_a = False
        pt_rand, ~ = calculateFK.forward(q_rand)
        pt_a, ~ = calculateFK.forward(q_a)

        for i in range(1,6):
            for j in range(len(obstacles)):
                begin_pt = pt_a[i,:]
                end_pt = pt_rand[i,:]
                isCollide = detectCollision(begin_pt, end_pt, obstacles[j,:])

                if(any(isCollide)):
                    print("qa qrand collide with ", j)
                    isCollide_a = True
                    break
        
        #if not collide, add new edge and node
        if not isCollide_a:
            new_edge = np.array([ind_a, len(V_start)]).reshape((1,2))
            E_start = np.append(E_start, new_edge, axis=0)
            V_start = np.append(V_start, q_rand, axis=0)
        
        ##################################################
        #Find closest node from end tree
        dist_s = []
        for node in V_goal:
            #measure by L2 distance in C-space
            d = np.sum(np.power((q_rand - node),2))
            dist_s.append(d)
        
        ind_b = np.argmin(dist_s)
        q_b = V_goal[ind_b,:]

        #Check if (q_rand, q_b) has collision
        isCollide_b = False
        pt_b, ~ = calculateFK.forward(q_b)

        for i in range(1,6):
            for j in range(len(obstacles)):
                begin_pt = pt_rand[i,:]
                end_pt = pt_b[i,:]
                isCollide = detectCollision(begin_pt, end_pt, obstacles[j,:])

                if(any(isCollide)):
                    print("qb qrand collide with ", j)
                    isCollide_b = True
                    break
        
        #if not collide, add new edge and node
        if not isCollide_b:
            new_edge = np.array([ind_b, len(V_goal)]).reshape((1,2))
            E_goal = np.append(E_goal, new_edge, axis=0)
            V_goal = np.append(V_goal, q_rand, axis=0)

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
        return path
    else:
        raise Exception("No Path Found. ")
            





def isValidConfig(q, obstacles):
    """
    Check if a configuration is in free C-space
    :q:                 Configuration of the robot (1x6)
    :obstacles          location of all boxes (Nx6) [xmin, ymin, zmin, xmax, ymax, zmax]
    :return:            True if valid configuration, else false
    """

    points, T0e = calculateFK.forward(q)

    beg_ind = [0,1,2,3,4]
    end_ind = [1,2,3,4,5]

    #Check for all links and obstacles
    for i in range(len(beg_ind)):
        for j in range(len(obstacles)):

            #Get Link coordinates and check if collide with box[j]
            begin_pt = points[beg_ind[i],:]
            end_pt = points[end_ind[i],:]
            isCollide = detectCollision(begin_pt, end_pt, obstacles[j,:])

            if(any(isCollide)):
                return False
    
    #check if gripper finger is in collision


    return True



