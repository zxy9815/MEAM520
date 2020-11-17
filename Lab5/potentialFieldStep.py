import numpy as np
from calculateFK import calculateFK
from distPointToBox import distPointToBox
from loadmap import loadmap

def potentialFieldStep(qCurr, map, qGoal):
    """
    This function plans a path through the map using a potential field planner
    :param qCurr:       current pose of the robot (1x6).
    :param map:         the map struct
    :param qGoal:       goal pose of the robot (1x6).
    :return:
            qNext - 1x6 vector of the robots next configuration
            isDone - a boolean that is true when the robot has reached the goal or is stuck. false otherwise

    """

    #Parameters
    d0 = 1.0  #attractive field conic to parabolic threshold
    xi = 1.0  #attractive field strength
    eta = 1.0  #repulsive field strength
    rho0 = 1.0  #repulsive field influence distance
    alpha = 1.0  #step size


    tau = np.zeros((1,6))

    obstacles = map.obstacles

    #compute joint positions
    fk = calculateFK()
    jointpos_curr, t0i = fk.forward(qCurr)

    jointpos_goal, t0iGoal = fk.forward(qGoal)

    #Start computing tau of each joint
    for j in range(6):

        ###############################################
        #Compute attractive forces on j
        Oj_curr = jointpos_curr[j, :].reshape((3,1))
        Oj_goal = jointpos_goal[j, :].reshape((3,1))

        F_att = np.array((3,1))

        #Case conic well
        if (np.linalg.norm(Oj_curr - Oj_goal) > d0):
            F_att = - (Oj_curr - Oj_goal) / np.linalg.norm(Oj_curr - Oj_goal)
        
        #Case parabolic
        else:
            F_att = - xi * (Oj_curr - Oj_goal)
        
        ###############################################
        #compute repulsive force on j
        F_rep = np.zeros((3,1))

        for k in range(len(obstacles)):
            dis2obs, unitVec = distPointToBox(Oj_curr.reshape((1,3)), obstacles[k])
            
            F_rep_k = np.zeros((3,1))
            
            if (dis2obs[0] < rho0):
                force = eta * (1.0/dis2obs[0] - 1.0/rho0) * (1.0/dis2obs[0]**2) * unitVec[0]
                F_rep_k = force.reshape((3,1))

            F_rep = F_rep + F_rep_k
        
        ################################################
        #total force
        F_total = F_att + F_rep

        #Compute joint effort tau for joint j
    

    ##########################################
    #Update the next configuration





    

if __name__=='__main__':

    start = np.array([0,  0, 0, 0, 0, 0])
    goal = np.array([0, 0, 1.1, 0, 0, 0])
    map = loadmap("maps/map2.txt")
    potentialFieldStep(start,map,goal)