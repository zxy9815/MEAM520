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
from astar import Astar
from loadmap import loadmap
from rrt import rrt

import timeit


if __name__=='__main__':
    # Update map location with the location of the target map
    map_struct = loadmap("maps/map1.txt")
    start = np.array([0,  0, 0, 0, 0, 0])
    goal = np.array([0, 0, 1.1, 0, 0, 0])

    #Start timing
    start_time = timeit.default_timer()

    # Run Astar code
    #path = Astar(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

    # or run rrt code
    #path = rrt(deepcopy(map_struct), deepcopy(goal), deepcopy(start))
    path = rrt(deepcopy(map_struct), deepcopy(start), deepcopy(goal))

    #End Timing
    elapsed = timeit.default_timer() - start_time
    print("Time of Execution: ", elapsed)

    # start ROS
    lynx = ArmController()
    sleep(1) # wait for setup
    collision = False

    # iterate over target waypoints
    for q in path:
        print("Goal:")
        print(q)

        lynx.set_pos(q)
        reached_target = False

        # Enter user defined variables here
        count = 0

        while not reached_target:
            # Check if robot is collided then wait
            collision = collision or lynx.is_collided()
            sleep(1)

            # Add Student code here to decide if controller should send next
            # target or continue to wait. Do NOT add additional sleeps to control
            # loop. You will likely want to use lynx.get_state() to decide when to
            # move to the next target.
            count = count + 1

            pos, vel = lynx.get_state()
            if(np.sum(np.abs(pos - q)) < 0.01):
                reached_target = True
                print("going to the next target")

            elif(collision):
                sleep(3)
                break

            elif(count > 8):
                reached_target = True

            # End of student code

        print("Current Configuration:")
        pos, vel = lynx.get_state()
        print(pos)

    if collision:
        print("Robot collided during move")
    else:
        print("No collision detected")

    lynx.stop()
