""" A simple robotics navigation code including SLAM, exploration, planning"""

import cv2
import numpy as np

from occupancy_grid import OccupancyGrid


class TinySlam:
    """Simple occupancy grid SLAM"""

    def __init__(self, occupancy_grid: OccupancyGrid):
        self.grid = occupancy_grid

        # Origin of the odom frame in the map frame
        self.odom_pose_ref = np.array([0, 0, 0])

    def _score(self, lidar, pose):
        """
        Computes the sum of log probabilities of laser end points in the map
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, position of the robot to evaluate, in world coordinates
        """
        # TODO for TP4

        score = 0

        return score

    def get_corrected_pose(self, odom_pose, odom_pose_ref=None):
        """
        Compute corrected pose in map frame from raw odom pose + odom frame pose,
        either given as second param or using the ref from the object
        odom : raw odometry position
        odom_pose_ref : optional, origin of the odom frame if given,
                        use self.odom_pose_ref if not given
        """
        # TODO for TP4
        corrected_pose = odom_pose

        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        best_score = 0

        return best_score

    def update_map(self, lidar, pose):
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """
        # TODO for TP3 
        # angles = lidar.get_ray_angles()
        # array[array > 3] = 3
        '''ranges = np.random.rand(360)
        ray_angles = np.arange(-np.pi,np.pi,np.pi/180)
        points_x = ranges * np.cos(ray_angles)
        points_y = ranges * np.sin(ray_angles)
        points = np.vstack((points_x, points_y))

        bigValue = 30
        smallValue = 3
        noValue = 0

        values = lidar.get_sensor_values()

        xVectors = np.array([values[i]*np.cos(i) for i in range(len(values))])
        yVectors = np.array([values[i]*np.sin(i) for i in range(len(values))])
        points = np.array([[xVectors[i] + pose[0], yVectors[i] + pose[1]] for i in values])

        self.grid.add_map_points(xVectors,yVectors,bigValue)
        for i in range(len(values)):
            self.grid.add_map_line(pose[0],pose[1],points[0],points[1],smallValue)'''

        PROBABILITY_BASE = 1
        SECURITY_RANGE = 10
        probability_big = 0.4
        probability_small = -0.05

        angles = lidar.get_ray_angles()
        values = lidar.get_sensor_values()

        # position
        x = pose[0] + values * np.cos(pose[2] + angles)
        y = pose[1] + values * np.sin(pose[2] + angles)
        x_free = pose[0] + (values - SECURITY_RANGE) * np.cos(pose[2] + angles)
        y_free = pose[1] + (values - SECURITY_RANGE) * np.sin(pose[2] + angles)
        

        # correction

        # repaire absolut
        points_free = np.vstack((x_free, y_free))

        for tuple in points_free.T:
            self.grid.add_map_line(pose[0],pose[1],tuple[0],tuple[1],probability_small)

        self.grid.add_map_points(x,y,probability_big)

        # display
        self.grid.occupancy_map[self.grid.occupancy_map < -PROBABILITY_BASE] = -PROBABILITY_BASE
        self.grid.occupancy_map[self.grid.occupancy_map > PROBABILITY_BASE] = PROBABILITY_BASE
        self.grid.display_cv(pose)

    def compute(self):
        """ Useless function, just for the exercise on using the profiler """
        # Remove after TP1

        """ranges = np.random.rand(3600)
        ray_angles = np.arange(-np.pi, np.pi, np.pi / 1800)

        # Poor implementation of polar to cartesian conversion
        points = []
        for i in range(3600):
            pt_x = ranges[i] * np.cos(ray_angles[i])
            pt_y = ranges[i] * np.sin(ray_angles[i])
            points.append([pt_x, pt_y])"""
