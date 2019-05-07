#!/usr/bin/env python
import rospy
from nav_msgs.msg import Path, OccupancyGrid
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from exploration.srv import AnyFrontiersLeft, AnyFrontiersLeftRequest, AnyFrontiersLeftResponse, GenerateFrontier, GenerateFrontierResponse
from exploration.srv import PlanPath,  PlanPathRequest, PlanPathResponse
import numpy as np
import tf.transformations as tft
import tf2_ros
def grid_cb(msg):
    robotPose = getRobotCoordinates(tfBuffer, msg.info)
    print(robotPose)
    gridPub = OccupancyGrid()
    gridPub.info = msg.info

    gridPub.data = msg.data
    gridPublisher.publish(gridPub)

def getRobotGridPosition(transMsg, gridInfo):
    pos = np.array([transMsg.transform.translation.x - gridInfo.origin.position.x, transMsg.transform.translation.y - gridInfo.origin.position.y, 0, 1])
    quat = gridInfo.origin.orientation
    mat = tft.quaternion_matrix(tft.quaternion_inverse([quat.x, quat.y, quat.z, quat.w]))
    gridPos = (mat.dot(pos[np.newaxis].T).flatten()[:2]) / gridInfo.resolution
    roundedPos = np.round(gridPos)
    pos = roundedPos if np.allclose(gridPos, roundedPos) else np.floor(gridPos)
    return pos    

def getRobotCoordinates(tfBuffer, gridInfo):
    """ Get the current robot position in the grid """
    try:
        trans = tfBuffer.lookup_transform("map", "base_link", rospy.Time(), rospy.Duration(0.5))
        print(trans)
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.logwarn("Cannot get the robot position!")
        robotPosition = None
    else:
        robotPosition = getRobotGridPosition(trans, gridInfo)  # TODO: transform the robot coordinates from real-world (in meters) into grid
        robotPosition = np.array([int(robotPosition[0]),int(robotPosition[1])])

    return robotPosition


if __name__ == "__main__":
    rospy.init_node("frontier_planner_cam")
    print("start")

    gridSubscriber = rospy.Subscriber('occupancy', OccupancyGrid, grid_cb)
    gridPublisher = rospy.Publisher('occupancy_cam',OccupancyGrid,queue_size=2)
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    rospy.spin()


