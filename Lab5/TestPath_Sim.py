#!/usr/bin/python2
from copy import deepcopy
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand

from sys import path
from os import getcwd
path.append(getcwd() + "/../Core")

from arm_controller import ArmController
from loadmap import loadmap
from potentialFieldPath import potentialFieldPath

if __name__=='__main__':
    # Update map location with the location of the target map
    map_struct = loadmap("maps/map1.txt")
    start = np.array([0,  0, 0, 0, 0, 0])
    goal = np.array([0, 0, 1.1, 0, 0, 0])

    # Run Astar code
    path = potentialFieldPath(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

    # start ROS
    lynx = ArmController()
    sleep(1) # wait for setup
    collision = False

    # Time in number of 0.1 sec intervals to wait before sending next command
    wait_count = 50

    # iterate over target waypoints
    for q in path:
        print("Goal:")
        print(q)

        lynx.set_pos(q)
        reached_target = False

        # Count is number of time steps waited
        count = 0
        while not reached_target:
            # Check if robot is collided then wait
            collision = collision or lynx.is_collided()
            sleep(0.1)

            # iterate count and check if should send next command
            count = count + 1
            if count > wait_count:
                reached_target = True
                count = 0

        print("Current Configuration:")
        pos, vel = lynx.get_state()
        print(pos)

    if collision:
        print("Robot collided during move")
    else:
        print("No collision detected")

    lynx.stop()
