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
"""
Here are imports that you are most likely will need. However, you may wish to remove or add your own import.
"""
from exploration import utils
import sys

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
        goalPosition = np.array([request.goal.x, request.goal.y], dtype=np.float)
        print("goal pos="+str(request.goal.x)+" , " + str(request.goal.y))

        # TODO:
        # TODO: First, you should try to obtain the robots coordinates
        self.getRobotCoordinates()
        print("rob pos ="+str(self.robotPosition))
        #gridPos=self.grid[self.robotPosition[0],self.robotPosition[1]]
        #print("rob grid="+str(gridPos))

        # TODO: Then, copy the occupancy grid into some temporary variable and inflate the obstacles
        threshold = 0
        tmpGrid = np.array(self.grid) # -1 = unseen, 0 = empty (i.e True = empty), 1..100 = full (i.e False = full)
        #np.set_printoptions(threshold=sys.maxsize)
        tmpGrid=np.reshape(tmpGrid,(self.gridInfo.height,self.gridInfo.width))
        #print(tmpGrid)
        print("shape="+str(tmpGrid.shape))
        inflated_grid = morphology.grey_dilation(tmpGrid,size=(4,4))
        print("INFLATED")
        #print(inflated_grid)
        #print("tmpGrid="+str(tmpGrid.shape))
        #tmpGrid = tmpGrid <= threshold
        #print("grid="+str(tmpGrid))
        #print("<>="+str(tmpGrid <= 0))
        tmpGrid = np.reshape(inflated_grid,self.gridInfo.height*self.gridInfo.width)
        print("shape="+str(tmpGrid.shape))
        tmpGrid = tmpGrid <= threshold
        #print("GRID=\n"+str(tmpGrid))



        # TODO: Compute the path, i.e. run some graph-based search algorithm
        rows = self.gridInfo.height
        cols = self.gridInfo.width
        goalPositionGrid = utils.getGridPosition(goalPosition, self.gridInfo)
        goalPositionGrid = np.array([int(goalPositionGrid[0]),int(goalPositionGrid[1])])
        print("goal="+str(goalPosition))
        print("goalGrid="+str(goalPositionGrid))
        #path=[]
        path = utils.AstarSearch(self.robotPosition,goalPositionGrid, tmpGrid, rows, cols)
        print(path)

        """ transform each point into real-world coordinates """
        real_path = [Pose2D(pos[0], pos[1], 0) for pos in [utils.gridToMapCoordinates(waypoint, self.gridInfo) for waypoint in path]]
        response = PlanPathResponse(real_path)
        return response

    def getRobotCoordinates(self):
        """ Get the current robot position in the grid """
        try:
            print("try")
            trans = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(0.5))
            print("TRANSFORMATIONS MAP->ROBOT")
            print(trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
            print("except")
        else:
            self.robotPosition = utils.getRobotGridPosition(trans, self.gridInfo)  # TODO: transform the robot coordinates from real-world (in meters) into grid
            #print("rob="+str(self.robotPosition[0,0])+","+str(self.robotPosition[0,1]))
            #print("rob="+str(self.robotPosition[0])+","+str(se,lf.robotPosition[1]))
            self.robotPosition = np.array([int(self.robotPosition[0]),int(self.robotPosition[1])])
            #print("rob2="+str(self.robotPosition))
            print("else")

    def extractGrid(self, msg):
        # TODO: extract grid from msg.data and other usefull information
        self.grid = msg.data
        self.gridInfo = msg.info
        pass

    def grid_cb(self, msg):
        print("OCCUPANCY GRID"),
        #print(msg)
        #print("msg data="+str(msg.data))
        print("size="+str(len(msg.data)))
        self.extractGrid(msg)
        if not self.gridReady:
            # TODO: Do some initialization of necessary variables

            # Create services
            self.plan_service = rospy.Service('plan_path', PlanPath, self.planPath)
            self.gridReady = True


if __name__ == "__main__":
    pp = PathPlanner()
    print("planner running")

    #pp.getRobotCoordinates()


    rospy.spin()
