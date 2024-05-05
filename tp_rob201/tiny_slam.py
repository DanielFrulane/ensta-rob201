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
              
        angles = lidar.get_ray_angles()
        values = lidar.get_sensor_values()

        x = pose[0] + values * np.cos(pose[2] + angles)
        y = pose[1] + values * np.sin(pose[2] + angles)

        # Apply log function to each value in the vector
        log_x = np.log(x)
        log_y = np.log(y)

        score = np.sum(log_x) + np.sum(log_y)

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
        if odom_pose_ref == None:
            odom_pose_ref = ([0]*len(odom_pose[0]),0*len(odom_pose[1])) # only zeros

        x = odom_pose_ref[0] + odom_pose[0] * np.cos(odom_pose_ref[2] + odom_pose[2])
        y = odom_pose_ref[1] + odom_pose[1] * np.sin(odom_pose_ref[2] + odom_pose[2])
        
        corrected_pose = [x,y]

        return corrected_pose

    def localise(self, lidar, raw_odom_pose):
        """
        Compute the robot position wrt the map, and updates the odometry reference
        lidar : placebot object with lidar data
        odom : [x, y, theta] nparray, raw odometry position
        """
        # TODO for TP4

        best_score = 0

        # best_score = _score(lidar, raw_odom_pose)

        return best_score

    def update_map(self, lidar, pose):
        """
        Bayesian map update with new observation
        lidar : placebot object with lidar data
        pose : [x, y, theta] nparray, corrected pose in world coordinates
        """
        # TODO for TP3 

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
