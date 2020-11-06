import numpy as np
from FK_velocity import *
from calculateFK import calculateFK

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D


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
    
    # remove rows of the Jacobian and xi when there are nan values
    notNanIdx = np.logical_not(np.isnan(xi))
    J = J[notNanIdx,:]
    xi = xi[notNanIdx]
    
    # Test if it's possible to achieve the exact input velocities v and omega
    # If so, dq should be the joint velocities to achieve that.
    # if not, it should be the least squares solution
    if not(np.linalg.matrix_rank(J) == np.linalg.matrix_rank(np.append(J,np.array([xi]).T,axis=1))):
        print('Infeasible velocities.')
        dq = np.linalg.pinv(J) @ xi
    else:
        dq = np.linalg.pinv(J) @ xi
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
    
    # NaN testing ###############################
    # end effector (joint 6) velocity:
    # q = np.array([ np.pi/4,0,-np.pi/6,np.pi/6,0,0])
    # v = np.array([1,np.nan,np.nan])
    # omega = np.array([1,0,0])
    
    # Trajectories ##############################
    # follow a line in the negative y direction, ignore orientation
    q0 = np.array([ np.pi/4,0,-np.pi/4,np.pi/4,0,0]) # start
    v = np.array([0,-1,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[\pi/4,0,-\pi/4,\pi/4,0,0]$' + \
        '\n' + r' and Moving in the Negative $y$-Direction'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in the Negative $y$-Direction'
    fileName = 'Y_rot.png'
    trajName = 'Y_q_rot.png'
    
    # follow a line in the positive x direction, ignore orientation
    q0 = np.array([ 0,0,-np.pi/4,np.pi/4,0,0]) # start
    v = np.array([1,0,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,0,-\pi/4,\pi/4,0,0]$' + \
        '\n' + r' and Moving in the $x$-Direction'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in the $x$-Direction'
    fileName = 'X_rot.png'
    trajName = 'X_q_rot.png'
    
    # follow a line in the positive z direction, ignore orientation
    q0 = np.array([ 0,-np.pi/4,np.pi/4,0,0,0]) # start
    v = np.array([0,0,1])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-\pi/4,\pi/4,0,0,0]$' + \
        '\n' + r' and Moving in the $z$-Direction'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in the $z$-Direction'
    fileName = 'Z_rot.png'
    trajName = 'Z_q_rot.png'
    
    # follow a line in the negative y direction, maintain orientation
    q0 = np.array([ np.pi/4,0,-np.pi/4,np.pi/4,0,0]) # start
    v = np.array([0,-1,0])
    omega = np.array([0,0,0]) # maintain orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[\pi/4,0,-\pi/4,\pi/4,0,0]$' + \
        '\n' + r' and Moving in the Negative $y$-Direction, Maintaining Orientation'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in the Negative $y$-Direction, Maintaining Orientation'
    errTitleStr = r'Euclidean Norm of Orientation Deviation from Initial Orientation' + \
        '\n' + r'Moving the End Effector in the Negative $y$-Direction, Maintaining Orientation'
    fileName = 'Y_noRot.png'
    trajName = 'Y_q_noRot.png'
    errName = 'Y_noRot_err.png'
    
    # follow a line in the positive x direction, maintain orientation
    # q0 = np.array([ 0,0,-np.pi/4,np.pi/4,0,0]) # start
    # v = np.array([1,0,0])
    # omega = np.array([0,0,0]) # maintain orientation
    # titleStr = r'End Effector Trajectory Beginning at $q=[0,0,-\pi/4,\pi/4,0,0]$' + \
    #     '\n' + r' and Moving in the $x$-Direction, Maintaining Orientation'
    # qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
    #     '\n' + r'in the $x$-Direction, Maintaining Orientation'
    # errTitleStr = r'Euclidean Norm of Orientation Deviation from Initial Orientation' + \
    #     '\n' + r'Moving the End Effector in the $x$-Direction, Maintaining Orientation'
    # fileName = 'X_noRot.png'
    # trajName = 'q_X_noRot.png'
    # errName = 'X_noRot_err.png'
    
    # follow a line in the positive z direction, maintain orientation
    q0 = np.array([ 0,-np.pi/4,np.pi/4,0,0,0]) # start
    v = np.array([0,0,1])
    omega = np.array([0,0,0]) # maintain orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-\pi/4,\pi/4,0,0,0]$' + \
        '\n' + r' and Moving in the $z$-Direction, Maintaining Orientation'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in the $z$-Direction, Maintaining Orientation'
    errTitleStr = r'Euclidean Norm of Orientation Deviation from Initial Orientation' + \
        '\n' + r'Moving the End Effector in the $z$-Direction, Maintaining Orientation'
    fileName = 'Z_noRot.png'
    trajName = 'Z_q_noRot.png'
    errName = 'Z_noRot_err.png'
    
    # Circular Trajectories #####################
    # follow a circle in the yz plane
    q0 = np.array([ 0,-np.pi/4,np.pi/4,0,0,0]) # start
    v = np.array([0,-1,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-\pi/4,\pi/4,0,0,0]$' + \
        '\n' + r'and Moving in a Circle in the $yz$-Plane'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in a Circle in the $yz$-Plane'
    fileName = 'YZ_rot.png'
    trajName = 'YZ_q_rot.png'
    
    # follow a circle in the xz plane
    q0 = np.array([ 0,-np.pi/4,np.pi/4,0,0,0]) # start
    v = np.array([1,0,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-\pi/4,\pi/4,0,0,0]$' + \
        '\n' + r'and Moving in a Circle in the $xz$-Plane'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in a Circle in the $xz$-Plane'
    fileName = 'XZ_rot.png'
    trajName = 'XZ_q_rot.png'
    rot = lambda theta: np.array([[np.cos(theta),0,np.sin(theta)],
                                  [0,1,0],
                                  [-np.sin(theta),0,np.cos(theta)]])
    
    # follow a circle in the xz plane, but closer to joint limits
    q0 = np.array([ 0,-1.4,1.4,0,0,0]) # start
    v = np.array([-1,0,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-1.4,1.4,0,0,0]$' + \
        '\n' + r'and Moving in a Circle in the $xz$-Plane'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in a Circle in the $xz$-Plane Beginning Close to Joint Limits'
    fileName = 'XZ_JL_rot.png'
    trajName = 'XZ_JL_q_rot.png'
    rot = lambda theta: np.array([[np.cos(theta),0,np.sin(theta)],
                                  [0,1,0],
                                  [-np.sin(theta),0,np.cos(theta)]])
    
    # follow a circle in the xz plane, but better?
    q0 = np.array([0,-np.pi/4,0,0,0,0]) # start
    v = np.array([1,0,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-\pi/4,0,0,0,0]$' + \
        '\n' + r'and Moving in a Real Circle in the $xz$-Plane'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in a Real Circle in the $xz$-Plane'
    fileName = 'XZ_better_rot.png'
    trajName = 'XZ_better_q_rot.png'
    rot = lambda theta: np.array([[np.cos(theta),0,np.sin(theta)],
                                  [0,1,0],
                                  [-np.sin(theta),0,np.cos(theta)]])
    
    # follow a circle in the xz plane, but rotated
    q0 = np.array([-np.pi/4,0,0,0,0,0]) # start
    v = np.array([1,0,0])
    omega = np.array([np.nan,np.nan,np.nan]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[-\pi/4,9,0,0,0,0]$' + \
        '\n' + r'and Moving in a Circle in the $xz$-Plane'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in a Circle in the $xz$-Plane'
    fileName = 'XZ_angleCirc_rot.png'
    trajName = 'XZ_angleCirc_q_rot.png'
    rot = lambda theta: np.array([[np.cos(theta),0,np.sin(theta)],
                                  [0,1,0],
                                  [-np.sin(theta),0,np.cos(theta)]])
    
    # Circles with Orientation ##################
    # follow a circle in the xz plane, but better?
    q0 = np.array([0,-np.pi/4,0,0,0,0]) # start
    v = np.array([1,0,0])
    omega = np.array([0,0,0]) # don't care about orientation
    titleStr = r'End Effector Trajectory Beginning at $q=[0,-\pi/4,0,0,0,0]$' + \
        '\n' + r'and Moving in a Real Circle in the $xz$-Plane, Maintaining Orientation'
    qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
        '\n' + r'in a Real Circle in the $xz$-Plane, Maintaining Orientation'
    errTitleStr = r'Euclidean Norm of Orientation Deviation from Initial Orientation' + \
        '\n' + r'Moving the End Effector in an $xz$ Circle, Maintaining Orientation'
    fileName = 'XZ_better_noRot.png'
    trajName = 'XZ_better_q_noRot.png'
    errName = 'XZ_better_noRot_err'
    rot = lambda theta: np.array([[np.cos(theta),0,np.sin(theta)],
                                  [0,1,0],
                                  [-np.sin(theta),0,np.cos(theta)]])
    
    # follow a circle in the xz plane, but rotated
    # q0 = np.array([-np.pi/4,0,0,0,0,0]) # start
    # v = np.array([1,0,0])
    # omega = np.array([0,0,0]) # don't care about orientation
    # titleStr = r'End Effector Trajectory Beginning at $q=[-\pi/4,9,0,0,0,0]$' + \
    #     '\n' + r'and Moving in a Circle in the $xz$-Plane, Maintaining Orientation'
    # qTitleStr = r'Joint Variable Changes Over Time for Moving the End Effector' + \
    #     '\n' + r'in a Circle in the $xz$-Plane, Maintaining Orientation'
    # errTitleStr = r'Euclidean Norm of Orientation Deviation from Initial Orientation' + \
    #     '\n' + r'Moving the End Effector in an $xz$ Circle out of the $xz$-Plane, Maintaining Orientation'
    # fileName = 'XZ_angleCirc_noRot.png'
    # trajName = 'XZ_angleCirc_q_noRot.png'
    # errName = 'XZ_angleCirc_noRot_err.png'
    # rot = lambda theta: np.array([[np.cos(theta),0,np.sin(theta)],
    #                               [0,1,0],
    #                               [-np.sin(theta),0,np.cos(theta)]])
    
    
    
    # Calculations ##############################
    joint = 6
    
    # Linear Trajectories ######
    # n_sec = 5     # number of seconds to run the bot
    # T = 0.1 # timestep
    # N = int(np.ceil(n_sec / T)) # number of steps the bot will run
    # new_q = q0.copy()
    # q = np.empty((N,6)) # array to plot the q values
    # q[0,:] = new_q
    # pos = np.empty((N,3)) # position of the end effector to check if the trajectory is proper
    # # calculate the initial position
    # fkCalc = calculateFK()
    # fk = fkCalc.forward(q[0,:])[0]
    # pos[0,:] = fk[5,:]
    
    # for t in range(1,N):
    #     dq = IK_velocity(new_q,v,omega,joint)
    #     q[t,:] = q[t-1,:].copy() + dq*T
    #     fk = fkCalc.forward(q[t,:])[0]
    #     pos[t,:] = fk[5,:]
    #     new_q = q[t,:].copy()
    # x = pos[:,0].copy()
    # y = pos[:,1].copy()
    # z = pos[:,2].copy()
        
    # Circular Trajectories ####
    # always start at the top of the circle
    degStep = 5
    step = degStep * np.pi / 180
    T = 1
    N = int(np.ceil(360/degStep))+1
    n_sec = N * T
    r = 5 # radius of the circle
    new_q = q0.copy()
    q = np.empty((N,6)) # array to plot the q values
    q[0,:] = new_q
    pos = np.empty((N,3)) # position of the end effector to check if the trajectory is proper
    fkCalc = calculateFK()
    fk = fkCalc.forward(q[0,:])[0]
    pos[0,:] = fk[5,:]
    new_v = v.copy()
    vs = np.empty((N,3))
    vs[0,:] = new_v
    for t in range(1,N):
        dq = IK_velocity(new_q,new_v,omega,joint)
        q[t,:] = q[t-1,:].copy() + dq*T
        fk = fkCalc.forward(q[t,:])[0]
        pos[t,:] = fk[5,:]
        
        # find new velocity:
        new_q = q[t,:].copy()
        new_v = rot(step) @ new_v
        vs[t,:] = new_v
    x = pos[:,0].copy()
    y = pos[:,1].copy()
    z = pos[:,2].copy()
    
    
    
    # Plotting ##################################
    plt.close('all')
    
    # Credit to stackoverflow user Remy F for square axes in 3d plots: https://stackoverflow.com/a/13701747    
    plt.rc('text',usetex=True)
    plt.rc('font',family='serif')
    
    fig = plt.figure()
    ax = fig.add_subplot(111)
    plt.plot(x,z,'k')
    ax.set_aspect('equal',adjustable='box')
    
    fig = plt.figure()
    ax = Axes3D(fig)
    # ax.set_aspect('equal')
    plt.subplots_adjust(top=0.88)
    
    scat = ax.plot(x,y,z,'k')
    
    # cubic bounding box with equal aspect ratio
    max_range = np.array([x.max()-x.min(),y.max()-y.min(),z.max()-z.min()]).max()
    Xb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][0].flatten() + 0.5*(x.max()+x.min())
    Yb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][1].flatten() + 0.5*(y.max()+y.min())
    Zb = 0.5*max_range*np.mgrid[-1:2:2,-1:2:2,-1:2:2][2].flatten() + 0.5*(z.max()+z.min())
    for xb, yb, zb in zip(Xb, Yb, Zb):
        ax.plot([xb], [yb], [zb], 'w')
    
    ax.set_xlabel(r'$x$')
    ax.set_ylabel(r'$y$')
    ax.set_zlabel(r'$z$')
    ax.set_title(titleStr,pad=4.0)
    ax.dist=11
    
    plt.savefig('./figs/3dTraj/'+fileName,format='png',dpi=196)
    plt.show()
    
    
    # Plot the joint variables
    fig = plt.figure()
    t = np.arange(0,n_sec,T)
    style = ('k-','k--','k-.','k:','ko:')
    for i in range(q.shape[1]-1):
        line = plt.plot(t,q[:,i]-q[0,i],style[i],markevery=5)
    plt.plot(t,np.zeros_like(t),'r:')
        
    
    plt.xlabel(r'Time $t$ [seconds]')
    plt.ylabel(r'Joint Angle $\theta$ [radians]')
    plt.title(qTitleStr)
    plt.legend([r'$\theta_1$',r'$\theta_2$',r'$\theta_3$',r'$\theta_4$',r'$\theta_5$',r'$t$-axis'])
    
    plt.savefig('./figs/'+trajName,format='png',dpi=196)
    plt.show()
    
    # plot error of orientation
    if all(np.logical_not(omega)):
        # find initial orientation
        fk, T0e = fkCalc.forward(q[0,:])
        orient0 = T0e[0:3,0:3]
        error = np.empty_like(t)
        for i in range(q.shape[0]):
            T0e = fkCalc.forward(q[i,:])[1]
            newOrient = T0e[0:3,0:3]
            error[i] = np.linalg.norm(orient0-newOrient)
        fig = plt.figure()
        plt.plot(t,error,'k')
        plt.xlabel(r'Time $t$ [seconds]')
        plt.ylabel(r'2-norm deviation')
        plt.title(errTitleStr)
        
        plt.savefig('./figs/Error/'+errName,format='png',dpi=196)
        plt.show()
        
    # points
    # np.set_printoptions(suppress=True)
    # print(IK_velocity(q,v,omega,joint))
