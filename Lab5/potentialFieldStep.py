import numpy as np

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
