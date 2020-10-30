#!/usr/bin/python2
from time import sleep
import numpy as np
import rospy
import sys
from random import random as rand

from sys import path
from os import getcwd
path.append(getcwd() + "/../Core")

from arm_controller import ArmController

dq = [.5,-0.5,-0.5, -0.1,1.2,0]

if __name__=='__main__':

    lynx = ArmController()
    sleep(1) # wait for setup

    print("Returning to Start Position:")
    lynx.set_pos([0,0,0,0,0,0])
    sleep(5)

    print("Target velocity:")
    print(dq)

    lynx.set_vel(dq)

    sleep(2)

    lynx.set_vel([0,0,0,0,0,0])

    pos, vel = lynx.get_state()

    print("Result:")
    print(pos)

    lynx.stop()