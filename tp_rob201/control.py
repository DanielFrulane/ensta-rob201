""" A set of robotics control functions """

import random
import numpy as np


def reactive_obst_avoid(lidar):
    """
    Simple obstacle avoidance
    lidar : placebot object with lidar data
    """
    # TODO for TP1

    speed = 0.0
    rotation_speed = 0.0
    values = lidar.get_sensor_values()

    if(values[180]<50):
        speed=0.0
        rotation_speed=0.5
    else:
        speed=0.5
        rotation_speed=0.0

    command = {"forward": speed,
               "rotation": rotation_speed}

    return command


def potential_field_control(lidar, current_pose, goal_pose):
    """
    Control using potential field for goal reaching and obstacle avoidance
    lidar : placebot object with lidar data
    current_pose : [x, y, theta] nparray, current pose in odom or world frame
    goal_pose : [x, y, theta] nparray, target pose in odom or world frame
    Notes: As lidar and odom are local only data, goal and gradient will be defined either in
    robot (x,y) frame (centered on robot, x forward, y on left) or in odom (centered / aligned
    on initial pose, x forward, y on left)
    """
    # TODO for TP2

    '''values = lidar.get_sensor_values()
    angles = lidar.get_ray_angles()
    nabla = np.array([0,0,0])
    K_goal = 1
    threasholdDistance = 1

    speed = 0.0
    rotation_speed = 0.0

    distance = np.sqrt((current_pose[0] - goal_pose[0])**2 + (current_pose[1] - goal_pose[1])**2)
    for i in range(len(nabla)):
        result = (K_goal/distance)*(goal_pose[i]-current_pose[i])
        print("r",i,result)
        nabla[i] = result
        print("n", nabla[i])

    rotation_speed=nabla[2]
    speed=nabla[0]

    command = {"forward": speed,
               "rotation": rotation_speed}

    return command'''
    
