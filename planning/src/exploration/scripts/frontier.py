#!/usr/bin/env python
from __future__ import division, print_function
import rospy
import numpy as np
from scipy.ndimage import morphology
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Pose2D
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
import tf2_ros
import geometry_msgs.msg
from exploration import utils
"""
Here are imports that you are most likely will need. However, you may wish to remove or add your own import.
"""


class FrontierExplorer():

    def __init__(self):
        # Initialize the node
        rospy.init_node("frontier_explorer")

        # Get some useful parameters
        self.robotDiameter = float(rospy.get_param("~robot_diameter", 0.2))
        self.occupancyThreshold = int(rospy.get_param("/occupancy_threshold", 10))

        # Helper variable to determine if grid was received at least once
        self.gridReady = False

        # You may wish to listen to the transformations of the robot
        self.tfBuffer = tf2_ros.Buffer()
        # Use the tfBuffer to obtain transformation as needed
        tfListener = tf2_ros.TransformListener(self.tfBuffer)

        # Subscribe to grid
        self.gridSubscriber = rospy.Subscriber('occupancy', OccupancyGrid, self.grid_cb)

        # TODO: you may wish to do initialization of other variables
    def hasOpenSpace(self, point):
        for open_space in [point+[0, -1], point+[0, 1], point+[-1, 0], point+[1, 0]]:
            if open_space[0] > (len(self.inflated_grid) - 2) or open_space[0] < 0 or open_space[1] > (len(self.inflated_grid) -2) or open_space[1] < 0:
                continue
            if self.inflated_grid[open_space[0]][open_space[1]] == 0:
                return True
        return False
    def check(self,test,array):
        return any(np.array_equal(x, test) for x in array)

    def computeWFD(self):
        """ Run the Wavefront detector """

        # TODO: Then, copy the occupancy grid into some temporary variable and inflate the obstacles
        self.inflated_grid = morphology.grey_dilation(self.grid,footprint=np.ones((1,1)))
        # TODO: Run the WFD algorithm - see the presentation slides for details on how to implement it
        frontiers = []
        # TODO:
        # TODO: First, you should try to obtain the robots coordinates
        mapOpenList = []
        frontierOpenList=[]
        frontierClosedList = []
        mapCosedList = []
        self.getRobotCoordinates()
        pose = self.robotPosition.astype(int)
        queuem = []
        queuem.append(pose)
        mapOpenList.append(pose)
        while len(queuem) > 0:
            newFrontier = None
            p = queuem.pop(0)
            if self.check(p, mapCosedList):
                continue

            if self.inflated_grid[p[0]][p[1]] == -1:
                if self.hasOpenSpace(p):
                    queuer = []
                    frontier = []
                    queuer.append(p)
                    frontierOpenList.append(p)
                    while len(queuer) > 0:
                        #print(queuer)
                        #print("here")
                        q = queuer.pop(0)
                        #print("popped")
                        #print(queuer)
                        if self.check(q, mapCosedList + frontierClosedList):
                            continue
                        #("here2")
                        #print(q)
                        if self.inflated_grid[q[0]][q[1]] == -1:
                            if self.hasOpenSpace(q):
                                #print(q)
                                newFrontier = q
                                #print(newFrontier)
                                for neigbours in [q+[0, -1], q+[0, 1], q+[-1, 0], q+[1, 0]]:
                                    if neigbours[0] > (len(self.inflated_grid) - 1) or neigbours[0] < 0 or neigbours[1] > (len(self.inflated_grid) -1) or neigbours[1] < 0:
                                        continue
                                    if self.check(neigbours, frontierOpenList + frontierClosedList + mapCosedList):
                                        queuer.append(neigbours)
                                        frontierOpenList.append(neigbours)
                        frontierClosedList.append(q)
                    
                    if newFrontier is not None:
                        frontiers.append(newFrontier)
            for neigbours in [p+[0, -1], p+[0, 1], p+[-1, 0], p+[1, 0]]:
                if neigbours[0] > (len(self.inflated_grid) - 1) or neigbours[0] < 0 or neigbours[1] > (len(self.inflated_grid) -1) or neigbours[1] < 0:
                    continue
                if not self.check(neigbours, mapOpenList):
                    if not self.check(neigbours, mapCosedList):
                        if self.hasOpenSpace(neigbours):
                            queuem.append(neigbours)
                            mapOpenList.append(neigbours)
            mapCosedList.append(p)
        print("fronteers:")
        print(frontiers)
        return frontiers

    def anyFrontiersLeft(self, request):
        """ Check if there are any frontiers left """
        # Run the WFD
        frontiers = self.computeWFD()
        # Return True if there are any frontiers, False otherwise
        return AnyFrontiersLeftResponse(any_frontiers_left=bool(len(frontiers) > 0))

    def getRandomFrontier(self, request):
        """ Return random frontier """
        # TODO
        frontiers = self.computeWFD()
        frontier = frontiers[np.random.choice(len(frontiers))]

        frontierCenter = 0  # TODO: compute center of the randomly drawn frontier here
        x, y = 0, 0  # TODO: transform the coordinates from grid to real-world coordinates (in meters)
        print("frontier")
        print(frontier)
        print(self.grid[frontier[0]][frontier[1]])
        x ,y = utils.gridToMapCoordinates(frontier,self.gridInfo)
        #print(utils.gridToMapCoordinates(frontier,self.gridInfo))
        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        return response

    def getClosestFrontier(self, request):
        """ Return frontier closest to the robot """
        # TODO
        frontiers = self.computeWFD()
        bestFrontier = utils.getDist(frontiers[0], self.robotPosition)
        print("bestFrontier")
        print(type(bestFrontier))   
        for op in frontiers:
            print(bestFrontier)
            utils.getDist(op, self.robotPosition)
            if (utils.getDist(op, self.robotPosition)< bestFrontier).all() :
                bestFrontier = op
       
        frontier = bestFrontier

        frontierCenter = 0  # TODO: compute the center of the chosen frontier
        x ,y = utils.gridToMapCoordinates(frontier,self.gridInfo)
        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        return response

    def getRobotCoordinates(self):
        """ Get the current robot position in the grid """
        try:
            trans = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
        else:
            self.robotPosition = utils.getRobotGridPosition(trans,self.gridInfo)  # TODO: transform the robot coordinates from real-world (in meters) into grid
    
    def extractGrid(self, msg):
        tmp = msg.data##
        array = np.asarray(tmp)
        np.set_printoptions(threshold=np.inf,linewidth=np.inf)  
        self.grid = np.reshape(array, [msg.info.width, msg.info.height]).T
        return self

    def grid_cb(self, msg):
        self.extractGrid(msg)
        if not self.gridReady:
            self.gridInfo = msg.info
            # TODO: Do some initialization of necessary variables

            # Create services
            self.afl_service = rospy.Service('any_frontiers_left', AnyFrontiersLeft, self.anyFrontiersLeft)
            self.grf_service = rospy.Service('get_random_frontier', GenerateFrontier, self.getRandomFrontier)
            self.gcf_service = rospy.Service('get_closest_frontier', GenerateFrontier, self.getClosestFrontier)
            self.gridReady = True


if __name__ == "__main__":
    fe = FrontierExplorer()

    rospy.spin()
