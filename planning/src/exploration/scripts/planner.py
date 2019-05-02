#!/usr/bin/env python
from __future__ import absolute_import, division, print_function
import rospy
import numpy as np
from scipy.ndimage import morphology
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose2D
from exploration.srv import PlanPath, PlanPathRequest, PlanPathResponse
import tf2_ros
import geometry_msgs.msg
from exploration import utils
"""
Here are imports that you are most likely will need. However, you may wish to remove or add your own import.
"""


class PathPlanner():

    def __init__(self):
        # Initialize the node
        rospy.init_node("path_planner")

        # Get some useful parameters
        self.robotDiameter = float(rospy.get_param("~robot_diameter", 0.2))
        self.occupancyThreshold = int(rospy.get_param("/occupancy_threshold", 10))

        # Helper variable to determine if grid was received at least once
        self.gridReady = False

        # You may wish to listen to the transformations of the robot
        self.tfBuffer = tf2_ros.Buffer()
        # Use the tfBuffer to obtain transformation as needed
        listener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribe to grid
        self.gridSubscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)

    def planPath(self, request):
        """ Plan and return path from the robot position to the requested goal """
        # Get the position of the goal (real-world)
        print("dest:")
        goalPos = np.array([request.goal.x, request.goal.y], dtype=np.float)

        goalPosition = utils.mapToGridCoodrinates(goalPos,self.gridInfo)
        print(goalPos)
        print(goalPosition)


        # TODO:
        # TODO: First, you should try to obtain the robots coordinates
        print("pose:")
        self.getRobotCoordinates()##
        print(self.robotPosition)
        # TODO: Then, copy the occupancy grid into some temporary variable and inflate the obstacles
        #print(self.grid)
        boundaries = (int(np.round((self.robotDiameter)/self.gridInfo.resolution)))
        
        inflated_grid = morphology.grey_dilation(self.grid,footprint=np.ones(((boundaries,boundaries))))####
        print(inflated_grid)
        # TODO: Compute the path, i.e. run some graph-based search algorithm
        #print(self.grid)
        #print(inflated_grid)

        path = utils.astar(inflated_grid,self.robotPosition.astype(int), goalPosition.astype(int))
        #print("path:")
        print (path)
        tmp = [utils.gridToMapCoordinates(np.array(waypoint),self.gridInfo) for waypoint in path]
        print(tmp)
        real_path = [Pose2D(pos[0], pos[1], 0) for pos in tmp]
        response = PlanPathResponse(real_path)
        return response

    def getRobotCoordinates(self):
        """ Get the current robot position in the grid """
        try:
            trans = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
        else:
            #grid_x = (unsigned int)((trans[0] - self.origin.position.x) / self.resolution)##
            #grid_y = (unsigned int)((trans[1] - self.origin.position.y) / self.resolution)##
            self.robotPosition = utils.getRobotGridPosition(trans,self.gridInfo)#[grid_x,grid_y] ## 

    def extractGrid(self, msg):
        tmp = msg.data##
        array = np.asarray(tmp)
        np.set_printoptions(threshold=np.inf,linewidth=np.inf)
        #print(array)
        array[array > 0] = 1
        #print(array)
        self.grid = np.reshape(array, [msg.info.width, msg.info.height]).T
        return self


    def grid_cb(self, msg):
        self.extractGrid(msg)
        if not self.gridReady:
            self.gridInfo = msg.info
            """self.resolution = msg.info.resolution##
            self.width = msg.info.width##
            self.heigth = msg.info.heigth##
            self.origin = msg.info.origin##"""
            # Create services
            self.plan_service = rospy.Service('plan_path', PlanPath, self.planPath)
            print("heyoo")
            self.gridReady = True


if __name__ == "__main__":
    pp = PathPlanner()

    rospy.spin()
