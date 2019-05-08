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
from sensor_msgs.msg import Image
import sys
"""
Here are imports that you are most likely will need. However, you may wish to remove or add your own import.
"""
from exploration import utils
import matplotlib.pyplot as plt

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
        self.image_pub = rospy.Publisher("/front_img", Image, queue_size=1)

    def computeWFD(self):
        """ Run the Wavefront detector """
        frontiers = []
        # TODO:
        # TODO: First, you should try to obtain the robots coordinates
        self.getRobotCoordinates()
        print("rob pos ="+str(self.robotPosition))

        # TODO: Then, copy the occupancy grid into some temporary variable and inflate the obstacles
        threshold = 50
        tmpGrid = np.array(self.grid) # -1 = unseen, 0 = empty (i.e True = empty), 1..100 = full (i.e False = full)
        tmpGrid=np.reshape(tmpGrid,(self.gridInfo.height,self.gridInfo.width))

        ######### kod pro zpiceny tycky na robotu co vidi lidar - turtle02 ########
        rob = self.robotPosition
        indexes = range(-4,5) # od -3 po 3
        for i in indexes:
            for j in indexes:
                if ((rob[1]+i)>self.gridInfo.height) or ((rob[0]+j)>self.gridInfo.width):
                    print("mazu mimo pole")
                    continue
                tmpGrid[rob[1]+i][rob[0]+j]=0


        inflated_grid = morphology.grey_dilation(tmpGrid,size=(9,9))
        tmpGrid = np.reshape(inflated_grid,self.gridInfo.height*self.gridInfo.width)
        #tmpGrid = tmpGrid <= threshold
        ##print("grid="+str(tmpGrid))

        # TODO: Run the WFD algorithm - see the presentation slides for details on how to implement it
        print("detecting")
        hi = self.gridInfo.height
        wi = self.gridInfo.width
        frontiers = utils.detectFrontiers(self.robotPosition,tmpGrid,self.gridInfo.height,self.gridInfo.width,threshold)
        ##print("Transformed:")
        print("transforming")
        fronti=[utils.gridToMapCoordinates(waypoint, self.gridInfo) for waypoint in frontiers]
        ##print(fronti)

        """im2=np.array([50 if x==-1 else x for x in tmpGrid])
        for a in frontiers:
            im2[(a[1])*50 + a[0]] = 75

        im2 = im2.reshape(50,50)
        self.image = im2"""
        """im=[]
        for x in tmpGrid:
            if x==-1:
                x=50
            else:
                if x>threshold: # i.e. obstacle
                    x=100
                else:
                    x=0
            im.append(x)
        im2=np.array(im)
        im2 = im2.reshape(self.gridInfo.height,self.gridInfo.width)
        self.image = im2
        plt.imshow(im2)
        plt.show()"""

        im=[]
        print("tmpGrid:"+str(len(tmpGrid)))
        for x in tmpGrid:
            if x==-1:
                x=50
            else:
                if x>threshold: # i.e. obstacle
                    x=100
                else:
                    x=0
            im.append(x)
        im2=np.array(im)
        #im2=np.array([50 if x==-1 else x for x in tmpGrid])
        self.image=[]
        if True:
            try:
                print("kresli frontiery")
                for a in frontiers:
                    if ((a[1])*wi + a[0]) >= len(im2):
                        print("bullshit")
                        continue
                    im2[(a[1])*wi + a[0]] = 75

                print("im2:"+str(len(im2)))
                print("reshaping image")
                print("wi:"+str(wi)+" hi:"+str(hi))
                if (hi*wi) > len(im2):
                    print("ani hovno")
                    return frontiers
                im2 = np.reshape(im2,(hi,wi))
                print("after reshape")
                im2[rob[1]][rob[0]]=90
                print("after rob")
            except:
                print("obrazek v pici")
            print("prirazeni")
            self.image = im2

            print("uloz image")
            plt.imshow(im2)
            print("after imshow")
            strIm = "xend.png"
            plt.savefig(strIm)
            print("after save")

        print("vraci frontiery")

        return frontiers

    def anyFrontiersLeft(self, request):
        """ Check if there are any frontiers left """
        # Run the WFD
        frontiers = self.computeWFD()
        #plt.imshow(self.image)
        #plt.show()
        # Return True if there are any frontiers, False otherwise
        return AnyFrontiersLeftResponse(any_frontiers_left=bool(len(frontiers) > 0))

    def getRandomFrontier(self, request):
        """ Return random frontier """
        # TODO
        frontiers = self.computeWFD()
        if not (len(frontiers)>0):
            print("frontieri nenalezeni!")
            return []
        print("Frontiers="+str(frontiers))
        indexes = range(len(frontiers))
        #print(indexes)
        #frontier = np.random.choice(frontiers)
        frontierInd = np.random.choice(indexes)
        frontier = frontiers[frontierInd]
        print("Random="+str(frontier))

        if True:
            print("making image")
            wi=self.gridInfo.width
            if True :#not (((frontier[1])*wi + frontier[0])>= len(self.image)):
                print("nebudu to delat")
                self.image[frontier[1],frontier[0]] = 25
                #self.image[(frontier[1])*wi + frontier[0]] =25
                plt.imshow(self.image)
                #plt.show()
                strIm = "fronti"+str(frontier[1])+"I"+str(frontier[0])+".png"
                plt.savefig(strIm)

        frontierCenter = (frontier[0]+frontier[1])/2  # TODO: compute center of the randomly drawn frontier here
        x, y = utils.gridToMapCoordinates(frontier, self.gridInfo)  # TODO: transform the coordinates from grid to real-world coordinates (in meters)
        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        print("koncim")
        return response

    def getClosestFrontier(self, request):
        """ Return frontier closest to the robot """
        # TODO
        frontiers = self.computeWFD()
        if not (len(frontiers)>0):
            print("frontieri nenalezeni!")
            return []
        print("find closest")
        bestFrontierIdx = utils.findClosestFrontier(self.robotPosition,frontiers)  # TODO: compute the index of the best frontier
        frontier = frontiers[bestFrontierIdx]
        print("Best="+str(frontier))

        if True:
            print("making image")
            wi=self.gridInfo.width
            if True: #not (((frontier[1])*wi + frontier[0])>= len(self.image)):
                print("nebudu to delat")
                self.image[frontier[1],frontier[0]] = 25
                #self.image[(frontier[1])*wi + frontier[0]] =25
                plt.imshow(self.image)
                strIm = "fronti"+str(frontier[1])+"I"+str(frontier[0])+".png"
                plt.savefig(strIm)
        #plt.show()
        """msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.data = self.image.tostring() #in_image.tostring()
        msg.height = self.image.shape[0]
        msg.width = self.image.shape[1]
        msg.step = sys.getsizeof(self.image.shape[1])
        msg.encoding = 'rgb8'
        msg.is_bigendian = 0
        self.image_pub.publish(msg)"""



        frontierCenter = (frontier[0]+frontier[1])/2  # TODO: compute the center of the chosen frontier
        x, y = utils.gridToMapCoordinates(frontier, self.gridInfo)  # TODO: compute the index of the best frontier
        response = GenerateFrontierResponse(Pose2D(x, y, 0.0))
        print("koncim")
        return response

    def getRobotCoordinates(self):
        """ Get the current robot position in the grid """
        try:
            trans = self.tfBuffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(0.5))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logwarn("Cannot get the robot position!")
            self.robotPosition = None
        else:
            self.robotPosition = utils.getRobotGridPosition(trans, self.gridInfo)  # TODO: transform the robot coordinates from real-world (in meters) into grid
            self.robotPosition = np.array([int(self.robotPosition[0]),int(self.robotPosition[1])])

    def extractGrid(self, msg):
        # TODO: extract grid from msg.data and other usefull information
        self.grid = msg.data
        self.gridInfo = msg.info

    def grid_cb(self, msg):
        self.extractGrid(msg)
        if not self.gridReady:
            # TODO: Do some initialization of necessary variables

            # Create services
            self.afl_service = rospy.Service('any_frontiers_left', AnyFrontiersLeft, self.anyFrontiersLeft)
            self.grf_service = rospy.Service('get_random_frontier', GenerateFrontier, self.getRandomFrontier)
            self.gcf_service = rospy.Service('get_closest_frontier', GenerateFrontier, self.getClosestFrontier)
            self.gridReady = True


if __name__ == "__main__":
    fe = FrontierExplorer()
    print("frontier exporer running")

    rospy.spin()
